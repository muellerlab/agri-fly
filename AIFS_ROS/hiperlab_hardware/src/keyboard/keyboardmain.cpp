/*
 * keyboardmain.cpp
 *
 *  Created on: May 24, 2018
 *      Author: nlbucki
 *
 * This node is meant to emulate joystick input. It should
 * NOT be used in actual flight, only for testing in simulation.
 *
 */

#include <ros/ros.h>
#include <ros/master.h>
#include <termios.h>
#include <stdio.h>
#include <regex>
#include <mutex>
#include <thread>
#include "Common/Misc/TerminalColors.hpp"

#include "Common/Math/Vec3.hpp"
#include "Common/Time/Timer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "hiperlab_rostools/joystick_values.h"

#define S_KEY 0X73
#define A_KEY 0x61
#define B_KEY 0x62
#define X_KEY 0x78
#define Y_KEY 0x79

using namespace std;

double const mainLoopFrequency = 20;  //Hz
volatile bool shouldExit = false;
int kfd = 0;
struct termios cooked, raw;

mutex keyboardMutex;
struct JSValues {
  float thrust, yaw, pitch, roll;
  bool buttonStart, buttonStop, buttonYellow, buttonBlue, buttonGreen;
};

volatile JSValues keyvals;

void readKeyThread() {
  char c;
  while (!shouldExit) {
    // Wait for keypress (NOTE: read() blocks thread execution until a key is pressed)
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    // Find witch key was pressed
//    printf("value: 0x%02X\n", c); // Uncomment to find key values
    {
      // Make sure we don't write values while the other thread is reading them
      std::lock_guard < std::mutex > guard(keyboardMutex);
      switch (c) {
        case S_KEY:
          keyvals.buttonStart = true;
          break;
        case A_KEY:
          keyvals.buttonGreen = true;
          break;
        case B_KEY:
          keyvals.buttonStop = true;
          break;
        case X_KEY:
          keyvals.buttonBlue = true;
          break;
        case Y_KEY:
          keyvals.buttonYellow = true;
          break;
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_controller");

  ros::NodeHandle n;
  ros::Publisher pubJs = n.advertise < hiperlab_rostools::joystick_values
      > ("joystick_values", 1);
  ros::master::V_TopicInfo master_topics;

  printf("Using keyboard to emulate joystick. NOT FOR USE IN FLIGHT\n");
  printf("Start = 's', ");
  TerminalColors::SetTerminalColor(TerminalColors::GREEN);
  cout << "Green = 'a', ";
  TerminalColors::SetTerminalColor(TerminalColors::RED);
  cout << "Stop = 'b', ";
  TerminalColors::SetTerminalColor(TerminalColors::BLUE);
  cout << "Blue = 'x', ";
  TerminalColors::SetTerminalColor(TerminalColors::YELLOW);
  cout << "Yellow = 'y'\n";
  TerminalColors::ResetTerminalColor();

  printf(
      "This node will NOT publish joystick commands unless a simulator node is running.\n\n");

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  // Start a separate thread to wait for key presses
  thread keyboardThread(readKeyThread);

  ros::Rate loop_rate(mainLoopFrequency);
  while (ros::ok()) {

    // Check that a simulation is running (i.e. prevent this node from being used in flight)
    bool simulator_running = false;
    ros::master::getTopics(master_topics);
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin();
        it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
      if (regex_match(info.name, regex(".*simulator.*"))
          || regex_match(info.name, regex(".*gazebo.*"))) {
        // At least one simulator is running
        simulator_running = true;
      }
    }

    // Read button input values
    bool buttonStart, buttonStop, buttonYellow, buttonBlue, buttonGreen;
    {
      std::lock_guard < std::mutex > guard(keyboardMutex);
      if (keyvals.buttonStart) {
        if (simulator_running) {
          buttonStart = true;
          keyvals.buttonStart = false;
          printf("Start\n");
        } else {
          buttonStart = false;
          keyvals.buttonStart = false;
          printf("No simulator node found, ignoring input.\n");
        }
      } else {
        buttonStart = false;
      }
      if (keyvals.buttonGreen) {
        if (simulator_running) {
          buttonGreen = true;
          keyvals.buttonGreen = false;
          printf("Green\n");
        } else {
          buttonGreen = false;
          keyvals.buttonGreen = false;
          printf("No simulator node found, ignoring input.\n");
        }
      } else {
        buttonGreen = false;
      }
      if (keyvals.buttonStop) {
        if (simulator_running) {
          buttonStop = true;
          keyvals.buttonStop = false;
          printf("Stop\n");
        } else {
          buttonStop = false;
          keyvals.buttonStop = false;
          printf("No simulator node found, ignoring input.\n");
        }
      } else {
        buttonStop = false;
      }
      if (keyvals.buttonBlue) {
        if (simulator_running) {
          buttonBlue = true;
          keyvals.buttonBlue = false;
          printf("Blue\n");
        } else {
          buttonBlue = false;
          keyvals.buttonBlue = false;
          printf("No simulator node found, ignoring input.\n");
        }
      } else {
        buttonBlue = false;
      }
      if (keyvals.buttonYellow) {
        if (simulator_running) {
          buttonYellow = true;
          keyvals.buttonYellow = false;
          printf("Yellow\n");
        } else {
          buttonYellow = false;
          keyvals.buttonYellow = false;
          printf("No simulator node found, ignoring input.\n");
        }
      } else {
        buttonYellow = false;
      }
    }

    //Publish the commands:
    hiperlab_rostools::joystick_values jv;
    jv.header.stamp = ros::Time::now();
    jv.axes[0] = 0;  // thrust
    jv.axes[1] = 0;  // yaw
    jv.axes[2] = 0;  // pitch
    jv.axes[3] = 0;  // roll

    jv.buttonStart = buttonStart;
    jv.buttonRed = buttonStop;
    jv.buttonGreen = buttonGreen;
    jv.buttonBlue = buttonBlue;
    jv.buttonYellow = buttonYellow;
    pubJs.publish(jv);

    loop_rate.sleep();
  }

  printf("Press any key to exit.\n");
  shouldExit = true;
  keyboardThread.join();
  tcsetattr(kfd, TCSANOW, &cooked);  // Reset terminal?
  printf("Done.\n");

  return 0;
}
