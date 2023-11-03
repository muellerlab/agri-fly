#include <iostream>
#include <thread>

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "ExampleVehicleStateMachine.hpp"

using namespace Offboard;
using namespace std;

//Definitions:
double const mainLoopFrequency = 50;    //Hz
double const systemLatencyTime = 30e-3;  // latency in measurements & commands[s]

bool useJoystick = true;
bool volatile jsButtonStart = false;
bool volatile jsButtonStop = false;

ManualTimer timer;
ros::Time lastRosTime;
ros::Time currentRosTime;

void callback_joystick(const hiperlab_rostools::joystick_values &msg) {
  jsButtonStart = msg.buttonStart > 0;
  jsButtonStop = msg.buttonRed > 0;
}

void callback_ros_time(const rosgraph_msgs::Clock &msg) {
  currentRosTime = ros::Time::now();
  // Convert the duration to microseconds
  double duration_in_microseconds = (currentRosTime-lastRosTime).toSec() * 1e6;
  timer.AdvanceMicroSeconds(duration_in_microseconds); // Advance the timer by the ROS duration.
  lastRosTime = currentRosTime;
}

void rosThreadFn() {
  // We run this function as a separate thread, allows us to continuously service any subscribers.
  ros::spin();
}

int main(int argc, char **argv) {

  if (argc < 2) {
    printf("ERROR: Must specify the vehicle ID\n");
    return -1;
  }

  int const vehicleId = atol(argv[1]);
  if (vehicleId <= 0 || vehicleId > 255) {
    printf("ERROR: invalid vehicle ID\n");
    return -1;
  }

  ros::init(argc, argv, "quad_rappids_rates_control" + std::to_string(vehicleId));

  for (int i = 2; i < argc; i++) {
    if (!strcmp(argv[i], "--no-js")) {
      printf("Disabling joystick use.\n");
      useJoystick = false;
    }
  }

  ros::NodeHandle n;
  ros::Subscriber subJoystick = n.subscribe("joystick_values", 1,
                                            callback_joystick);
  ros::Subscriber subRosTimer = n.subscribe("clock", 1, callback_ros_time);
                                          
  //Set everything up.

  ExampleVehicleStateMachine veh;
  veh.Initialize(vehicleId, "rates vehicle", n, &timer, systemLatencyTime);

  Vec3d desiredPosition(0.0, 0.0, 2.0);
  double desiredYawAngle = 0.0 * M_PI / 180.0;

  cout << "Desired position setpoint = <" << desiredPosition.x << ","
       << desiredPosition.y << "," << desiredPosition.z << ">\n";

  veh.SetDesiredPosition(desiredPosition);
  veh.SetDesiredYaw(desiredYawAngle);

  ros::Rate loop_rate(mainLoopFrequency);

  Timer emergencyTimer(&timer);
  double const EMERGENCY_BUTTON_PERIOD = 0.5;  //if you hold the land button down, it triggers an emergency.

  cout << "Starting main controller.\n";

//  thread rosThread(rosThreadFn);
  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();
  timer.ResetMicroseconds(0);
  lastRosTime = ros::Time::now();
  currentRosTime = ros::Time::now();

  bool firstPanic = true;

  Vec3d initPosition;
  bool shouldQuit = false;

  cout << "Waiting for estimator init...\n";
  while (ros::ok()) {
    loop_rate.sleep();
    if (veh.GetIsGPSEstInitialized() &&
        veh.GetIsOdometryEstInitialized()) {
      break;
    }
  }
  cout << "Est initialized.\n";

  cout << "Waiting for joystick start button...\n";
  while (ros::ok()) {
    loop_rate.sleep();
    if (jsButtonStart) {
      //force to release button:
      while (jsButtonStart) {
        loop_rate.sleep();
      }
      break;
    }
  }
  cout << "Continuing. Hit start again to take off.\n";

  while (ros::ok() && !shouldQuit) {
    if (!jsButtonStop) {
      emergencyTimer.Reset();
    }

    if (emergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD) {
      printf("Panic button!\n");
      veh.SetExternalPanic();
    }

    veh.Run(jsButtonStart, jsButtonStop);

    if (veh.GetIsReadyToExit()) {
      break;
    }

    loop_rate.sleep();
  }
  spinner.stop();
  ros::shutdown();
//  rosThread.join();
  return 0;
}
