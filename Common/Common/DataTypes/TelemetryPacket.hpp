/* Telemetry packet header file
 * Author: Eugene Lo
 */
#pragma once

#include <stdlib.h>
#include <string.h>
#include <limits>

namespace TelemetryPacket {
/********************************
 ** GENERAL METHODS/DEFINITIONS **
 ********************************/
enum PacketType {
  PACKET_TYPE_QUAD_TELEMETRY_PT1 = 0,
  PACKET_TYPE_QUAD_TELEMETRY_PT2 = 1,
  PACKET_TYPE_GENERIC_FLOAT = 100
};

//warnings that the vehicle can report. Each is a bit (8 options)
enum TelemetryWarnings {
  WARN_LOW_BATT = 0x01,  //battery has been seen to be low at least once
  WARN_CMD_RATE = 0x02,  //not receiving commands at the expected rate
  WARN_UWB_RESET = 0x04,  //recent onboard estimator reset
  WARN_ONBOARD_FREQ = 0x08,  //Cannot execute the main loop as fast as planned
  WARN_CMD_BATCH_DROP = 0x10,  //dropping packets in large batches
  WARN_RESERVED_6 = 0x20,  //for future use
  WARN_RESERVED_CUSTOM_1 = 0x40,  //Never assigned in the main branch, feel free to use for anything
  WARN_RESERVED_CUSTOM_2 = 0x80,  //Never assigned in the main branch, feel free to use for anything
};

struct data_packet_t {  // Used for sending the packet to the radio
  uint8_t type;
  uint8_t packetNumber;
  uint16_t data[14];
}__attribute__((packed));

/* Map x from [a,b] to [-1,1] */
inline float MapToOnesRange(float x, float a, float b) {
  return ((x - a) / (b - a)) * 2 - 1;
}

/* Map x from [-1,1] to [a,b] */
inline float MapToAB(float x, float a, float b) {
  return ((x + 1) / 2) * (b - a) + a;
}

/* Encode values from range [-1,1] into 2 bytes.
 Input t is mapped to 0 iff t is outside of [-1,1].

 Maximum value of 2 byte uint is 2^16 - 1 = 65535.
 To encode a float from [-1,1], we scale the range up to
 [-2^15-1, 2^15-1] and then offset by +2^15 (since uints are non-negative).
 */
inline uint16_t EncodeOnesRange(float t) {
  uint16_t out;
  if (t < -1 || t > 1) {
    out = 0;
  } else {
    out = 32768 + 32767 * t;
  }
  return out;
}

/* Decode above encoding */
inline float DecodeOnesRange(uint16_t t) {
  if (t == 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return (t - 32768) / float(32768);
}

/**********************
 ** TELEMETRY PACKETS **
 **********************/
/* Range limits for packets used in QuadcopterLogic.cpp
 These limits are needed to perform the encoding/decoding from floats to ints
 (see MapToOnesRange, EncodeOnesRange above)
 Limits were determined empirically. */
enum TelemetryRanges {
  TEL_RANGE_ACC_MAX = 30,
  TEL_RANGE_ACC_MIN = -TEL_RANGE_ACC_MAX,
  TEL_RANGE_GYRO_MAX = 35,
  TEL_RANGE_GYRO_MIN = -TEL_RANGE_GYRO_MAX,
  TEL_RANGE_FORCE_MAX = 10,
  TEL_RANGE_FORCE_MIN = 0,
  TEL_RANGE_BATTVOLTAGE_MAX = 15,
  TEL_RANGE_BATTVOLTAGE_MIN = 0,
  TEL_RANGE_POS_MAX = 30,
  TEL_RANGE_POS_MIN = -TEL_RANGE_POS_MAX,
  TEL_RANGE_VEL_MAX = 30,
  TEL_RANGE_VEL_MIN = -TEL_RANGE_VEL_MAX,
  TEL_RANGE_ATT_MAX = 1,
  TEL_RANGE_ATT_MIN = -TEL_RANGE_ATT_MAX,
  TEL_RANGE_GENERIC_MAX = 100,
  TEL_RANGE_GENERIC_MIN = -TEL_RANGE_GENERIC_MAX,

};

struct TelemetryPacket {
  enum {
    NUM_DEBUG_FLOATS = 6,
  };
  // Header Info
  uint8_t type;
  uint8_t packetNumber;  // The ID of the packet, is shared amongst all sub-packets.
  /* seqNum = 0 -> packet includes accel, gyro */
  float accel[3];
  float gyro[3];
  float motorForces[4];
  float position[3];
  float battVoltage;
  /* seqNum = 1 -> packet includes position, attitude, velocity, panicReason */
  float velocity[3];
  float attitude[3];
  float debugVals[NUM_DEBUG_FLOATS];
  uint8_t panicReason;
  uint8_t warnings;
};

/* Encode a TelemetryPacket into a data_packet_t */
inline void EncodeTelemetryPacket(TelemetryPacket const &src,
                                  data_packet_t &out) {
  out.type = src.type;
  out.packetNumber = src.packetNumber;
  if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT1) {
    for (int i = 0; i < 3; i++) {
      out.data[i + 0] = EncodeOnesRange(
          MapToOnesRange(src.accel[i], TEL_RANGE_ACC_MIN, TEL_RANGE_ACC_MAX));
      out.data[i + 3] = EncodeOnesRange(
          MapToOnesRange(src.gyro[i], TEL_RANGE_GYRO_MIN, TEL_RANGE_GYRO_MAX));
    }
    for (int i = 0; i < 4; i++) {
      out.data[i + 6] = EncodeOnesRange(
          MapToOnesRange(src.motorForces[i], TEL_RANGE_FORCE_MIN,
                         TEL_RANGE_FORCE_MAX));
    }
    for (int i = 0; i < 3; i++) {
      out.data[i + 10] = EncodeOnesRange(
          MapToOnesRange(src.position[i], TEL_RANGE_POS_MIN,
                         TEL_RANGE_POS_MAX));
    }
    out.data[13] = EncodeOnesRange(
        MapToOnesRange(src.battVoltage, TEL_RANGE_BATTVOLTAGE_MIN,
                       TEL_RANGE_BATTVOLTAGE_MAX));

  } else if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT2) {
    for (int i = 0; i < 3; i++) {
      out.data[i + 0] = EncodeOnesRange(
          MapToOnesRange(src.velocity[i], TEL_RANGE_VEL_MIN,
                         TEL_RANGE_VEL_MAX));
      out.data[i + 3] = EncodeOnesRange(
          MapToOnesRange(src.attitude[i], TEL_RANGE_ATT_MIN,
                         TEL_RANGE_ATT_MAX));
    }
    for (int i = 0; i < TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
      out.data[i + 6] = EncodeOnesRange(
          MapToOnesRange(src.debugVals[i], TEL_RANGE_GENERIC_MIN,
                         TEL_RANGE_GENERIC_MAX));
    }

    memcpy(&out.data[12], &src.panicReason, 1);
    memcpy(&out.data[13], &src.warnings, 1);
  }
  return;
}

/* Decode a data_packet_t into a TelemetryPacket */
inline void DecodeTelemetryPacket(data_packet_t const &in,
                                  TelemetryPacket &out) {
  out.type = in.type;
  out.packetNumber = in.packetNumber;
  if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT1) {
    for (int i = 0; i < 3; i++) {
      out.accel[i] = MapToAB(DecodeOnesRange(in.data[i + 0]), TEL_RANGE_ACC_MIN,
                             TEL_RANGE_ACC_MAX);
      out.gyro[i] = MapToAB(DecodeOnesRange(in.data[i + 3]), TEL_RANGE_GYRO_MIN,
                            TEL_RANGE_GYRO_MAX);
    }
    for (int i = 0; i < 4; i++) {
      out.motorForces[i] = MapToAB(DecodeOnesRange(in.data[i + 6]),
                                   TEL_RANGE_FORCE_MIN, TEL_RANGE_FORCE_MAX);
    }
    for (int i = 0; i < 3; i++) {
      out.position[i] = MapToAB(DecodeOnesRange(in.data[i + 10]),
                                TEL_RANGE_POS_MIN, TEL_RANGE_POS_MAX);
    }
    out.battVoltage = MapToAB(DecodeOnesRange(in.data[13]),
                              TEL_RANGE_BATTVOLTAGE_MIN,
                              TEL_RANGE_BATTVOLTAGE_MAX);

  } else if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT2) {
    for (int i = 0; i < 3; i++) {
      out.velocity[i] = MapToAB(DecodeOnesRange(in.data[i + 0]),
                                TEL_RANGE_VEL_MIN, TEL_RANGE_VEL_MAX);
      out.attitude[i] = MapToAB(DecodeOnesRange(in.data[i + 3]),
                                TEL_RANGE_ATT_MIN, TEL_RANGE_ATT_MAX);
    }
    for (int i = 0; i < TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
      out.debugVals[i] = MapToAB(DecodeOnesRange(in.data[i + 6]),
                                 TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    }
    memcpy(&out.panicReason, &in.data[12], 1);
    memcpy(&out.warnings, &in.data[13], 1);
  }
  return;
}

/******************
 ** FLOAT PACKETS **
 ******************/
/* Reverts uints to floats in range [-1,1] */
inline void DecodeFloatPacket(data_packet_t const &packetIn, float out[],
                              int const numFloats) {
  for (int i = 0; i < numFloats; i++) {
    if (i >= 14)
      break;
    out[i] = DecodeOnesRange(packetIn.data[i]);
  }
}

/* Assume floats in range [-1,1] */
inline void EncodeFloatPacket(data_packet_t &packetOut, float const floats[],
                              int const numFloats) {
  packetOut.type = PACKET_TYPE_GENERIC_FLOAT;

  int i;
  for (i = 0; i < numFloats; i++) {
    if (i >= 14)
      break;
    packetOut.data[i] = EncodeOnesRange(floats[i]);
  }

  for (; i < 14; i++) {
    packetOut.data[i] = EncodeOnesRange(0);
  }
}

}
