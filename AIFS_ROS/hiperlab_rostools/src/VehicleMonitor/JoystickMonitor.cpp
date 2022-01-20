#include "JoystickMonitor.hpp"

#include "Common/Time/HardwareTimer.hpp"

using namespace TerminalColors;

JoystickMonitor::JoystickMonitor(ros::NodeHandle &n, BaseTimer* const t)
    : _timeSinceLastRun(t) {
  _numJs = 0;
  //set up networking stuff:
  _subJS.reset(
      new ros::Subscriber(
          n.subscribe("joystick_values", 1, &JoystickMonitor::CallbackJoystick,
                      this)));
}

void JoystickMonitor::CallbackJoystick(
    const hiperlab_rostools::joystick_values& msg) {
  _numJs++;
}

void JoystickMonitor::RunAndPrint() {
  double dt = _timeSinceLastRun.GetSeconds_d();
  _timeSinceLastRun.Reset();

  if (!_numJs) {
    SetTerminalColor(RED);
    printf("  No joystick!\n");
    ResetTerminalColor();
    return;
  }

  float rateJS = _numJs / dt;

  bool isOK_js = VehicleMonitor::IsRateOK(rateJS, VehicleMonitor::Joystick);

  printf("  JS @");
  SetTerminalColor(isOK_js ? GREEN : RED);
  printf("%*d", 3, int(0.5 + rateJS));
  ResetTerminalColor();
  printf("Hz\n");

  _numJs = 0;
}
