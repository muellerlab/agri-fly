#include <iostream>
#include <thread>
#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"
#include "hiperlab_rostools/telemetry.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"
#include "Common/Misc/TerminalColors.hpp"

#include "Components/Logic/PanicReason.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"

#include "VehicleMonitor.hpp"
#include "JoystickMonitor.hpp"

using namespace std;

const unsigned MAX_VEHICLE_ID = 50;

int main(int argc, char **argv) {
  ros::init(argc, argv, "vehicle_monitor");
  ros::NodeHandle n;
  std::vector<VehicleMonitor*> vMons;

  HardwareTimer hwTimer;
  for (int vehicleId = 1; vehicleId < MAX_VEHICLE_ID; vehicleId++) {
    if (vehicleId <= 0 || vehicleId > 255) {
      printf("ERROR: invalid vehicle ID: %s\n", argv[vehicleId]);
      return -1;
    }
    vMons.push_back(new VehicleMonitor(vehicleId, n, &hwTimer));
  }

  JoystickMonitor jsMon(n, &hwTimer);

  double const mainLoopFrequency = 1;  //Hz

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Rate loop_rate(mainLoopFrequency);

  Timer tPrint(&hwTimer);
  while (ros::ok()) {
    //      12345678901234567890123456789012345678901234567890
    for (int i = 0; i < 20; i++) {
      printf("\n");
    }
    jsMon.RunAndPrint();
    VehicleMonitor::PrintTitles();
    unsigned nVehicles = 0;
    for (auto v : vMons) {
      if (v->RunAndPrint()) {
        nVehicles++;
      }
    }
    if (!nVehicles) {
      printf(" (no vehicles)\n");
    }

    loop_rate.sleep();
  }

  return 0;
}
