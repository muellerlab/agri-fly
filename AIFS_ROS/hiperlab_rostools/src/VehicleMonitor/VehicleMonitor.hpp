#pragma once

#include <iostream>
#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"
#include "hiperlab_rostools/telemetry.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Time/Timer.hpp"
#include "Common/Misc/TerminalColors.hpp"

#include "Components/Logic/PanicReason.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"

class VehicleMonitor {
 public:
  enum MsgType {
    MoCap,
    Cmd,
    Joystick,
    Telemetry,
  };

  VehicleMonitor(int id, ros::NodeHandle &n, BaseTimer* const t);

  static void PrintTitles();

  bool RunAndPrint();

  static bool IsRateOK(float in, MsgType t);
 private:
  static void GetRateBounds(const MsgType t, float &lwr, float &upr);

  void CallbackMocap(const hiperlab_rostools::mocap_output& msg);
  void CallbackTelemetry(const hiperlab_rostools::telemetry& msg);
  void CallbackCommand(const hiperlab_rostools::radio_command& msg);

  std::shared_ptr<ros::Subscriber> _subMocap, _subTel, _subCmd;

  Timer _timeSinceLastRun;

  unsigned volatile _numMocap, _numTelemetry, _numCmd;
  int _id;
  double _lastTelBattV;
  unsigned _lastTelPanicReason;
  volatile uint8_t _lastTelWarnings;
};
