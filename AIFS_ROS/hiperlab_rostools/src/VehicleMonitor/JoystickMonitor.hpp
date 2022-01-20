#pragma once

#include "VehicleMonitor.hpp"

class JoystickMonitor {
 public:
  JoystickMonitor(ros::NodeHandle &n, BaseTimer* const t);

  void RunAndPrint();

 private:
  void CallbackJoystick(const hiperlab_rostools::joystick_values& msg);

  std::shared_ptr<ros::Subscriber> _subJS;
  Timer _timeSinceLastRun;

  unsigned volatile _numJs;
};
