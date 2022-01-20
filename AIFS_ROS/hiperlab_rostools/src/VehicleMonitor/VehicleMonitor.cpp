#include "VehicleMonitor.hpp"

#include "Common/Time/HardwareTimer.hpp"

using namespace TerminalColors;

VehicleMonitor::VehicleMonitor(int id, ros::NodeHandle &n, BaseTimer* const t)
    : _timeSinceLastRun(t) {
  _numMocap = 0;
  _numTelemetry = 0;
  _numCmd = 0;
  _lastTelBattV = 0;
  _lastTelPanicReason = 0;
  _lastTelWarnings = 0;

  _id = id;
  //set up networking stuff:
  _subMocap.reset(
      new ros::Subscriber(
          n.subscribe("mocap_output" + std::to_string(_id), 1,
                      &VehicleMonitor::CallbackMocap, this)));
  _subTel.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &VehicleMonitor::CallbackTelemetry, this)));
  _subCmd.reset(
      new ros::Subscriber(
          n.subscribe("radio_command" + std::to_string(_id), 1,
                      &VehicleMonitor::CallbackCommand, this)));
}

void VehicleMonitor::GetRateBounds(const VehicleMonitor::MsgType t, float &lwr,
                                   float &upr) {
  //all in Hz
  switch (t) {
    case MoCap:
      lwr = 195;
      upr = 205;
      return;
    case Cmd:
      lwr = 45;
      upr = 55;
      return;
    case Joystick:
      lwr = 95;
      upr = 105;
      return;
    case Telemetry:
      lwr = 50;
      upr = 170;
      return;
  }
}

bool VehicleMonitor::IsRateOK(float in, VehicleMonitor::MsgType t) {
  float lwr, upr;
  GetRateBounds(t, lwr, upr);
  if (in < lwr) {
    return false;
  }
  if (in > upr) {
    return false;
  }
  return true;
}

void VehicleMonitor::CallbackMocap(const hiperlab_rostools::mocap_output& msg) {
  _numMocap++;
}

void VehicleMonitor::CallbackTelemetry(
    const hiperlab_rostools::telemetry& msg) {
  _numTelemetry++;
  _lastTelBattV = msg.batteryVoltage;
  _lastTelPanicReason = msg.panicReason;
  _lastTelWarnings |= msg.warnings;
}

void VehicleMonitor::CallbackCommand(
    const hiperlab_rostools::radio_command& msg) {
  _numCmd++;
}

bool VehicleMonitor::RunAndPrint() {
  double dt = _timeSinceLastRun.GetSeconds_d();
  _timeSinceLastRun.Reset();

  if (!_numMocap and !_numTelemetry and !_numCmd) {
    //vehicle not seen
    return false;
  }

  float rateMocap = _numMocap / dt;
  float rateTel = _numTelemetry / dt;
  float rateCmd = _numCmd / dt;

  bool isOK_mocap = IsRateOK(rateMocap, MoCap);
  bool isOK_tel = IsRateOK(rateTel, Telemetry);
  bool isOK_cmd = IsRateOK(rateCmd, Cmd);

  bool allOK = false;
  if (isOK_mocap and isOK_tel and isOK_cmd) {
    allOK = true;
  }

  printf("|");
  SetTerminalColor(allOK ? GREEN : RED);
  printf("%*d", 3, _id);
  ResetTerminalColor();
  printf("|");
  SetTerminalColor(isOK_mocap ? GREEN : RED);
  printf("%*d", 5, int(0.5 + rateMocap));
  ResetTerminalColor();
  printf("|");
  SetTerminalColor(isOK_cmd ? GREEN : RED);
  printf("%*d", 3, int(0.5 + rateCmd));
  ResetTerminalColor();
  printf("|");
  SetTerminalColor(isOK_tel ? GREEN : RED);
  printf("%*d", 3, int(0.5 + rateTel));
  ResetTerminalColor();
  printf("|");

  //battery level:
  if (!_numTelemetry) {
    //never seen this vehicle's telemetry
    SetTerminalColor(YELLOW);
    printf(" -- ");
    ResetTerminalColor();
    printf("|");
  } else {
    printf("%4.1f|", _lastTelBattV);

    if (_lastTelPanicReason) {
      SetTerminalColor(RED);
      printf(
          "%s",
          Onboard::GetPanicReasonString(
              Onboard::PanicReason(_lastTelPanicReason)));
    }

    //print warnings:
    SetTerminalColor(YELLOW);
    if (_lastTelWarnings & TelemetryPacket::WARN_LOW_BATT) {
      printf(" WARN_LOW_BATT");
    }
    if (_lastTelWarnings & TelemetryPacket::WARN_CMD_RATE) {
      printf(" WARN_CMD_RATE");
    }
    if (_lastTelWarnings & TelemetryPacket::WARN_UWB_RESET) {
      printf(" WARN_UWB_RESET");
    }
    if (_lastTelWarnings & TelemetryPacket::WARN_ONBOARD_FREQ) {
      printf(" WARN_ONBOARD_FREQ");
    }
    if (_lastTelWarnings & TelemetryPacket::WARN_CMD_BATCH_DROP) {
      printf(" WARN_CMD_BATCH_DROP");
    }
    if (_lastTelWarnings & TelemetryPacket::WARN_RESERVED_6) {
      printf(" WARN_RESERVED_6");
    }
    if (_lastTelWarnings & TelemetryPacket::WARN_RESERVED_CUSTOM_1) {
      printf(" WARN_RESERVED_CUSTOM_1");
    }
    if (_lastTelWarnings & TelemetryPacket::WARN_RESERVED_CUSTOM_2) {
      printf(" WARN_RESERVED_CUSTOM_2");
    }

    ResetTerminalColor();
  }
  _lastTelWarnings = 0;

  _numMocap = 0;
  _numTelemetry = 0;
  _numCmd = 0;
  printf("\n");
  return true;
}

void VehicleMonitor::PrintTitles() {
  printf("  all rates in [Hz], batt in [V]\n");
  //         123 12345 123 123 123
  printf("+---+-----+---+---+----+---------------------\n");
  printf("|ID |Mocap|Cmd|Tel|Batt|Info\n");
  printf("+---+-----+---+---+----+---------------------\n");
}
