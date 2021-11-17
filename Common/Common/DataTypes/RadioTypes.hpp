#pragma once

#include "Common/Math/Vec3.hpp"

//Definitions for use with the radio

/* Assume that we transmit as follows:
 *
 * First byte  = message type
 * second byte = generic bits
 * third byte  = first "data" value
 * etc.
 */

namespace RadioTypes {

enum Type {
  invalid = 0,
  reserved_future = 1,//replace this next time we make larger changes here...
  emergencyKill = 2,
  positionCommand = 3,
  externalAccelerationCmd = 4,
  externalRatesCmd = 5,
  idleCommand = 6,
};

//Reserved flag bits. Must be between 1 and 8.
enum ReservedFlags {
  calibrateMotors = 0x01,
  disableOnboardStateSafetyChecks = 0x02,
  flagNo3 = 0x04,  //(use as you like)
  flagNo4 = 0x08,  //(use as you like)
  flagNo5 = 0x10,  //(use as you like)
  flagNo6 = 0x20,  //(use as you like)
  flagNo7 = 0x40,  //(use as you like)
  flagNo8 = 0x80,  //(use as you like)
};

struct RadioMessageDecoded {
 public:
  enum {
    IDX_TYPE = 0,
    IDX_RESERVED = IDX_TYPE + 1,  //For future use
    IDX_FLAGS = IDX_RESERVED + 1,
    IDX_FLOATS = IDX_FLAGS + 1,
    RADIO_FLOAT_ENCODED_SIZE = 2,  //number of bytes we use to encode a float
    RADIO_FLOAT_ENCODED_MAX = 1 << (RADIO_FLOAT_ENCODED_SIZE * 8),
    RADIO_FLOAT_ENCODED_HALF = RADIO_FLOAT_ENCODED_MAX / 2,
    NUM_RADIO_FLOAT_FIELDS = 10,
    RAW_PACKET_SIZE = IDX_FLOATS
        + RADIO_FLOAT_ENCODED_SIZE * NUM_RADIO_FLOAT_FIELDS,
  };

  enum {
    MAX_VAL_CMD_THRUST = 35,
    MAX_VAL_CMD_ANG_RATES = 35,
    MAX_VAL_CMD_POS = 20,
    MAX_VAL_CMD_VEL = 10,
    MAX_VAL_CMD_ACCELERATION = 30,
    MAX_VAL_DEFAULT = 1,
  };

  enum {
    I_POS = 0,
    I_VEL = 3,
    I_ACC = 6,
  };

  struct RawMessage {
    uint8_t raw[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  };

  static void encodeToRadioByte(float const valIn, float const limit,
                                unsigned const indx,
                                uint8_t bytes[RAW_PACKET_SIZE]) {
    int out;
    if ((valIn > -limit) && (valIn < limit)) {
      //in acceptable range
      out = int(valIn * RADIO_FLOAT_ENCODED_HALF / limit + 0.5f)
          + RADIO_FLOAT_ENCODED_HALF;
    } else if (valIn > -limit) {
      //max value:
      out = RADIO_FLOAT_ENCODED_MAX - 1;
    } else if (valIn < limit) {
      //min value:
      out = 0;
    } else {
      //NAN:
      out = 0;
    }

    //This is probably super non-portable, and a total hack...
    for (int i = 0; i < RADIO_FLOAT_ENCODED_SIZE; i++) {
      if (indx + i >= RAW_PACKET_SIZE) {
        //make sure we only write in allowed memory
        break;
      }
      bytes[indx + i] = (out >> ((RADIO_FLOAT_ENCODED_SIZE - i - 1) * 8)) % 256;  //the last 8 bits.
    }
    return;
  }

  static float decodeFromRadioBytes(uint8_t const bytesIn[RAW_PACKET_SIZE],
                                    unsigned const indx, float const limit) {
    int out = 0;
    //This is probably super non-portable, and a total hack...
    for (int i = 0; i < RADIO_FLOAT_ENCODED_SIZE; i++) {
      if (indx + i >= RAW_PACKET_SIZE) {
        //make sure we only write in allowed memory
        break;
      }
      out += bytesIn[indx + i] << ((RADIO_FLOAT_ENCODED_SIZE - 1 - i) * 8);
    }
    return limit * (out - RADIO_FLOAT_ENCODED_HALF)
        / float(RADIO_FLOAT_ENCODED_HALF);
  }

  RadioMessageDecoded() {
    type = invalid;
    flags = 0;
  }

  static inline void CreateKillCommand(uint8_t const flags,
                                       uint8_t rawOut[RAW_PACKET_SIZE]) {
    rawOut[IDX_TYPE] = emergencyKill;
    rawOut[IDX_RESERVED] = 0;  //just a placeholder
    rawOut[IDX_FLAGS] = flags;
  }

  static inline void CreateIdleCommand(uint8_t const flags,
                                       uint8_t rawOut[RAW_PACKET_SIZE]) {
    rawOut[IDX_TYPE] = idleCommand;
    rawOut[IDX_RESERVED] = 0;  //just a placeholder
    rawOut[IDX_FLAGS] = flags;
  }

  static inline void CreatePositionCommand(uint8_t const flags,
                                           Vec3f const desPos,
                                           Vec3f const desVel,
                                           Vec3f const desAcc,
                                           uint8_t rawOut[RAW_PACKET_SIZE]) {
    rawOut[IDX_TYPE] = positionCommand;
    rawOut[IDX_RESERVED] = 0;  //just a placeholder
    rawOut[IDX_FLAGS] = flags;
    for (int i = 0; i < 3; i++) {
      encodeToRadioByte(desPos[i], MAX_VAL_CMD_POS,
                        IDX_FLOATS + (I_POS + i) * RADIO_FLOAT_ENCODED_SIZE,
                        rawOut);
      encodeToRadioByte(desVel[i], MAX_VAL_CMD_VEL,
                        IDX_FLOATS + (I_VEL + i) * RADIO_FLOAT_ENCODED_SIZE,
                        rawOut);
      encodeToRadioByte(desAcc[i], MAX_VAL_CMD_ACCELERATION,
                        IDX_FLOATS + (I_ACC + i) * RADIO_FLOAT_ENCODED_SIZE,
                        rawOut);
    }
  }

  static inline void CreateRatesCommand(uint8_t const flags,
                                        float const desTotalThrust,
                                        Vec3f const desAngVel,
                                        uint8_t rawOut[RAW_PACKET_SIZE]) {
    rawOut[IDX_TYPE] = externalRatesCmd;
    rawOut[IDX_RESERVED] = 0;  //just a placeholder
    rawOut[IDX_FLAGS] = flags;
    encodeToRadioByte(desTotalThrust, MAX_VAL_CMD_THRUST, IDX_FLOATS, rawOut);
    for (int i = 0; i < 3; i++) {
      encodeToRadioByte(desAngVel[i], MAX_VAL_CMD_ANG_RATES,
                        IDX_FLOATS + (i + 1) * RADIO_FLOAT_ENCODED_SIZE,
                        rawOut);
    }
  }

  static inline void CreateAccelerationCommand(
      uint8_t const flags, Vec3f const acc, float const yawRate,
      uint8_t rawOut[RAW_PACKET_SIZE]) {
    rawOut[IDX_TYPE] = externalAccelerationCmd;
    rawOut[IDX_RESERVED] = 0;  //just a placeholder
    rawOut[IDX_FLAGS] = flags;
    encodeToRadioByte(acc.x, MAX_VAL_CMD_ACCELERATION,
                      IDX_FLOATS + 0 * RADIO_FLOAT_ENCODED_SIZE, rawOut);
    encodeToRadioByte(acc.y, MAX_VAL_CMD_ACCELERATION,
                      IDX_FLOATS + 1 * RADIO_FLOAT_ENCODED_SIZE, rawOut);
    encodeToRadioByte(acc.z, MAX_VAL_CMD_ACCELERATION,
                      IDX_FLOATS + 2 * RADIO_FLOAT_ENCODED_SIZE, rawOut);
    encodeToRadioByte(yawRate, MAX_VAL_CMD_ANG_RATES,
                      IDX_FLOATS + 3 * RADIO_FLOAT_ENCODED_SIZE, rawOut);
  }

  inline RadioMessageDecoded(uint8_t const raw[RAW_PACKET_SIZE]) {
    type = raw[IDX_TYPE];
    flags = raw[IDX_FLAGS];

    //scale message:
    switch (type) {
      case positionCommand:
        for (int i = I_POS; i < I_POS + 3; i++) {
          floats[i] = decodeFromRadioBytes(
              raw, IDX_FLOATS + i * RADIO_FLOAT_ENCODED_SIZE, MAX_VAL_CMD_POS);
        }
        for (int i = I_VEL; i < I_VEL + 3; i++) {
          floats[i] = decodeFromRadioBytes(
              raw, IDX_FLOATS + i * RADIO_FLOAT_ENCODED_SIZE, MAX_VAL_CMD_VEL);
        }
        for (int i = I_ACC; i < I_ACC + 3; i++) {
          floats[i] = decodeFromRadioBytes(
              raw, IDX_FLOATS + i * RADIO_FLOAT_ENCODED_SIZE,
              MAX_VAL_CMD_ACCELERATION);
        }
        break;

      case externalRatesCmd:
        floats[0] = decodeFromRadioBytes(
            raw, IDX_FLOATS + 0 * RADIO_FLOAT_ENCODED_SIZE, MAX_VAL_CMD_THRUST);
        for (int i = 1; i < NUM_RADIO_FLOAT_FIELDS; i++) {
          floats[i] = decodeFromRadioBytes(
              raw, IDX_FLOATS + i * RADIO_FLOAT_ENCODED_SIZE,
              MAX_VAL_CMD_ANG_RATES);
        }
        break;

      case externalAccelerationCmd:
        for (int i = 0; i < 3; i++) {
          floats[i] = decodeFromRadioBytes(
              raw, IDX_FLOATS + i * RADIO_FLOAT_ENCODED_SIZE,
              MAX_VAL_CMD_ACCELERATION);
        }
        floats[3] = decodeFromRadioBytes(
            raw, IDX_FLOATS + 3 * RADIO_FLOAT_ENCODED_SIZE,
            MAX_VAL_CMD_ANG_RATES);
        break;

      default:  //if you're sending some other type of radio command
        for (int i = 0; i < NUM_RADIO_FLOAT_FIELDS; i++) {
          floats[i] = decodeFromRadioBytes(
              raw, IDX_FLOATS + i * RADIO_FLOAT_ENCODED_SIZE, MAX_VAL_DEFAULT);
        }
        break;

    }
  }

  uint8_t type;
  uint8_t flags;
  float floats[NUM_RADIO_FLOAT_FIELDS];
 private:
};

}

