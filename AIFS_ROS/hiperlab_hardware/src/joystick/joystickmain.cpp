#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include <mutex>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Common/Math/Vec3.hpp"
#include "Common/Time/Timer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "hiperlab_rostools/joystick_values.h"

#define JOY_DEV "/dev/input/js0"

using namespace std;
double const mainLoopFrequency = 100;  //Hz
float const JS_THRUST_SCALE = 1.5f * 9.81f;
float const JS_LATERAL_ACC_SCALE = 1.5f * 9.81f;
float const JS_YAW_RATE_SCALE = 3.0f;

//identified with the joystick_identification program (just compile & run that)
enum XBOX360 {
  axis_thrust = 1,
  axis_yaw = 0,
  axis_pitch = 4,
  axis_roll = 3,
  sign_thrust = -1,
  sign_yaw = +1,
  sign_pitch = -1,
  sign_roll = +1,
  button_red = 1,
  button_start = 7,
  button_yellow = 3,
  button_green = 0,
  button_blue = 2,
};

struct JSValues {
  float thrust, yaw, pitch, roll;
  bool buttonStart, buttonStop, buttonYellow, buttonBlue, buttonGreen;
};

struct {
  mutex jsMutex;
  volatile JSValues jsValues;
} currentJS;

volatile bool shouldExit = false;

void readJSThread(int joy_fd, js_event js, int num_of_axis,
                  int num_of_buttons) {

  int *axis = (int *) calloc(num_of_axis, sizeof(int));
  char *button = (char *) calloc(num_of_buttons, sizeof(char));

  while (!shouldExit) {
    // read the joystick state
    //TODO: more elegant would be a poll(.) call...
    read(joy_fd, &js, sizeof(struct js_event));

    /* see what to do with the event */
    switch (js.type & ~JS_EVENT_INIT) {
      case JS_EVENT_AXIS:
        axis[js.number] = js.value;
        break;
      case JS_EVENT_BUTTON:
        button[js.number] = js.value;
        break;
    }

    //normalize all axes to [-1,1]
    float thrust = axis[axis_thrust] / float(1 << 15) * float(sign_thrust);
    float yaw = axis[axis_yaw] / float(1 << 15) * float(sign_yaw);
    float pitch = axis[axis_pitch] / float(1 << 15) * float(sign_pitch);
    float roll = axis[axis_roll] / float(1 << 15) * float(sign_roll);

    bool buttonStart = button[button_start];
    bool buttonStop = button[button_red];
    bool buttonYellow = button[button_yellow];
    bool buttonGreen = button[button_green];
    bool buttonBlue = button[button_blue];
    //Copy the values:
    {
      std::lock_guard<std::mutex> guard(currentJS.jsMutex);
      currentJS.jsValues.thrust = thrust;
      currentJS.jsValues.yaw = yaw;
      currentJS.jsValues.pitch = pitch;
      currentJS.jsValues.roll = roll;
      currentJS.jsValues.buttonStart = buttonStart;
      currentJS.jsValues.buttonStop = buttonStop;
      currentJS.jsValues.buttonBlue = buttonBlue;
      currentJS.jsValues.buttonGreen = buttonGreen;
      currentJS.jsValues.buttonYellow = buttonYellow;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_controller");

  ros::NodeHandle n;
  ros::Publisher pubJs = n.advertise<hiperlab_rostools::joystick_values>(
      "joystick_values", 1);

  int joy_fd, num_of_axis = 0, num_of_buttons = 0;
  char name_of_joystick[80];
  struct js_event js;

  {
    stringstream failedJoystickNames;
    int numJsTried = 0;
    string joystickDeviceBase("/dev/input/js");
    bool jsOpenedOK = false;
    for (int i = 0; i < 100; i++) {
      stringstream jsDevice;
      jsDevice << joystickDeviceBase << i;
      if ((joy_fd = open(jsDevice.str().c_str(), O_RDONLY)) == -1) {
        //can't open, so end.
        break;
      }
      ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);
      if (0 == string(name_of_joystick).find("Microsoft X-Box")) {
        cout << "Wired joystick detected.\n";
        jsOpenedOK = true;
        break;
      } else if (0 == string(name_of_joystick).find("Xbox 360 Wireless Receiver")) {
        cout << "Wireless joystick detected.\n";
        jsOpenedOK = true;
        break;
      } else if (0 == string(name_of_joystick).find("Generic X-Box pad")) {
        cout << "Wireless joystick detected.\n";
        jsOpenedOK = true;
        break;
      }
      failedJoystickNames << "  - " << name_of_joystick << "\n";
      numJsTried++;
    }

    if (!jsOpenedOK) {
      cout << "Failed to open joystick!\n";
      if (numJsTried) {
        cout << "Tried:\n";
        cout << failedJoystickNames.str();
      } else {
        cout << "No joysticks found.\n";
      }
      cout
          << "Make sure the Xbox controller is plugged in.\n";
      return -1;
    }
  }

  ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
  ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);

  printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n", name_of_joystick,
         num_of_axis, num_of_buttons);

  fcntl(joy_fd, F_SETFL, O_NONBLOCK); /* use non-blocking mode */

  thread threadJS(readJSThread, joy_fd, js, num_of_axis, num_of_buttons);

  ros::Rate loop_rate(mainLoopFrequency);
  Timer tPrint(new HardwareTimer);
  double printPeriod = 0.05;
  while (ros::ok()) {
    //normalize all axes to [-1,1]

    //get the latest JS values:
    float thrust, yaw, pitch, roll;
    bool buttonStart, buttonStop, buttonYellow, buttonBlue, buttonGreen;
    {
      std::lock_guard<std::mutex> guard(currentJS.jsMutex);
      thrust = currentJS.jsValues.thrust;
      yaw = currentJS.jsValues.yaw;
      pitch = currentJS.jsValues.pitch;
      roll = currentJS.jsValues.roll;
      buttonStart = currentJS.jsValues.buttonStart;
      buttonStop = currentJS.jsValues.buttonStop;
      buttonBlue = currentJS.jsValues.buttonBlue;
      buttonGreen = currentJS.jsValues.buttonGreen;
      buttonYellow = currentJS.jsValues.buttonYellow;
    }

    Vec3f desAcc = Vec3f(pitch * fabsf(pitch) * JS_LATERAL_ACC_SCALE,
                         -roll * fabsf(roll) * JS_LATERAL_ACC_SCALE,
                         thrust * JS_THRUST_SCALE - 9.81f);

    float desYawRate = -yaw * JS_YAW_RATE_SCALE;

    if (tPrint.GetSeconds<double>() > printPeriod) {
      tPrint.Reset();
      printf(
          "T = %02.03f, Y=%02.03f, P=%02.03f, R=%02.03f, acc = <%02.03f, %02.03f, %02.03f>; buttons: ",
          thrust, yaw, pitch, roll, desAcc.x, desAcc.y,
          desAcc.z);
      if(buttonStart){
        printf("Start ");
      }
      if(buttonStop){
        printf("Stop ");
      }
      if(buttonYellow){
        printf("Ylw ");
      }
      if(buttonBlue){
        printf("Blu ");
      }
      if(buttonGreen){
        printf("Grn");
      }
      printf("------\r");

      fflush(stdout);
    }

    //Publish the commands:
    hiperlab_rostools::joystick_values jv;
    jv.header.stamp = ros::Time::now();
    jv.axes[0] = thrust;
    jv.axes[1] = yaw;
    jv.axes[2] = pitch;
    jv.axes[3] = roll;

    jv.buttonStart = buttonStart;
    jv.buttonRed = buttonStop;
    jv.buttonGreen = buttonGreen;
    jv.buttonBlue = buttonBlue;
    jv.buttonYellow = buttonYellow;
    pubJs.publish(jv);

    loop_rate.sleep();
  }

  printf("Ending JS thread\n");
  shouldExit = true;
  threadJS.join();
  printf("Done.");

  close(joy_fd);
  return 0;
}
