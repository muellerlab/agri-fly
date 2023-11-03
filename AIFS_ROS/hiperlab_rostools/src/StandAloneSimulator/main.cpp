/* A simple simulator.
 *
 */

#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <stdio.h>
#include <unistd.h>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ctime>
#include <random>
#include <string>

#include <ros/ros.h>

#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/Quadcopter_T.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"

#include <fstream>
#include "hiperlab_rostools/gps_output.h"
#include "hiperlab_rostools/imu_output.h"
#include "hiperlab_rostools/simulator_truth.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"
#include <rosgraph_msgs/Clock.h>

// AirSim includes
#include <iostream> // for image saving

// Code for image publisher
#include "std_msgs/Bool.h"

// Code for VIO
#include <nav_msgs/Odometry.h>

using namespace std;

mutex cmdRadioChannelMutex;  //protect against concurrency problems

class SimVehicle {
 public:
  struct {
    std::shared_ptr<
        Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage>> queue;
  } cmdRadioChannel;

  std::shared_ptr<ros::Subscriber> subRadioCmd;
  std::shared_ptr<ros::Publisher> pubSimTruth;
  std::shared_ptr<ros::Publisher> pubTelemetry;
  std::shared_ptr<ros::Publisher> pubImagePoll;
  std::shared_ptr<ros::Publisher> pubOdometry;
  std::shared_ptr<ros::Publisher> pubGPS;
  std::shared_ptr<ros::Publisher> pubIMU;
  std::shared_ptr<ros::Publisher> pubSimTime;
  rosgraph_msgs::Clock simTimeMsg;

  int id;
  Vec3d _initPos;
  Rotationd _R_wo;  //Rotation from body frame to odometry frame
  // See Intel Real Sense T265 for definition of the odometry frame.
  // We assume in our case, the negative z axis of the odometry frame aligns with the positive x of the world frame

  std::shared_ptr<Simulation::SimulationObject6DOF> vehicle;

  void callbackRadioCmd(const hiperlab_rostools::radio_command &msg) {
    std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
    //todo: should be a nicer way to do this, using e.g. memcpy...
    RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
      rawMsg.raw[i] = msg.raw[i];
    }
    cmdRadioChannel.queue->AddMessage(rawMsg);
    return;
  }
};

std::vector<std::shared_ptr<SimVehicle>> vehicles;

template<typename Real>
string toCSV(const Vec3<Real> v) {
  stringstream ss;
  ss << v.x << "," << v.y << "," << v.z << ",";
  return ss.str();
}

int main(int argc, char **argv) {

  ////////////////////////////////////////////////////////////////
  //ROS setup
  ////////////////////////////////////////////////////////////////

  // TODO make this robust for use with ROS launch and terminal
  int numVehicles = argc - 1;  // ROS passes extra args when using a .launch file (change to - 3)
  if (numVehicles < 1) {
    cout << "ERROR: Must specify at least one vehicle ID\n";
    cout
        << "\t You can input a list of vehicle IDs, and the simulator will spawn many vehicles.\n";
    return -1;
  }

  cout << numVehicles << "\n";

  ros::init(argc, argv, "simulator");
  ros::NodeHandle n;

  int const vehicleId = atol(argv[1]);
  if (vehicleId <= 0 || vehicleId > 255) {
    cout << "\n\n\n";
    cout << "ERROR: invalid vehicle ID <" << argv[1] << ">\n";
    return -1;
  }

  std::shared_ptr<SimVehicle> v;
  v.reset(new SimVehicle());
  v->id = vehicleId;
  v->subRadioCmd.reset(
      new ros::Subscriber(
          n.subscribe("radio_command" + std::to_string(vehicleId), 1,
                      &SimVehicle::callbackRadioCmd, v.get())));
  v->pubSimTruth.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::simulator_truth>(
              "simulator_truth" + std::to_string(vehicleId), 1)));

  v->pubGPS.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::gps_output>(
              "gps_output" + std::to_string(vehicleId), 1)));
  v->pubIMU.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::imu_output>(
              "imu_output" + std::to_string(vehicleId), 1)));
  v->pubTelemetry.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::telemetry>(
              "telemetry" + std::to_string(vehicleId), 1)));

  //Publish simulated Odometry info.
  v->pubOdometry.reset(
      new ros::Publisher(
          n.advertise<nav_msgs::Odometry>("/camera/t265/odom/sample", 1)));

  v->pubImagePoll.reset(
      new ros::Publisher(n.advertise<std_msgs::Header>("imagePoll", 1)));

  //Publish time info to overwrite the system clock
  v->pubSimTime.reset(
      new ros::Publisher(n.advertise<rosgraph_msgs::Clock>("/clock", 1)));

  cout << "Publisher setup.\n";
  vehicles.push_back(v);

  ////////////////////////////////////////////////////////////////
  //Simulator setup
  ////////////////////////////////////////////////////////////////
  //Basic timing:
  const double frequencySimulation = 500.0;  //run the simulation at this rate
  const double frequencyDepthImage = 30.0;  //[Hz]
  const double frequencyOdometry = 250.0;  //[Hz] Update rate at 250Hz. 
  const double frequencyGPSOutput = 200.0; //[Hz] Update rate at 100Hz.
  const double frequencyTelemetry = 500.0; //[Hz] Update rate at 500Hz.
  const double frequencyIMUOutput = 500.0; //[Hz] Update rate at 100Hz.
  HardwareTimer simTimer;

  //The communication transport delay:
  double const timeDelayOffboardControlLoop = 20e-3;

  //create the vehicles
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(v->id);

  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  //Create the quadcopter:
  double const mass = vehConsts.mass;                                   //[kg]
  double const inertia_xx = vehConsts.inertia_xx;                  //[kg.m**2]
  double const inertia_yy = inertia_xx;                            //[kg.m**2]
  double const inertia_zz = vehConsts.inertia_zz;                  //[kg.m**2]
  double armLength = vehConsts.armLength;                                //[m]
  double propThrustFromSpeedSqr = vehConsts.propellerThrustFromSpeedSqr;  //[N/(rad/s)**2]
  double propTorqueFromSpeedSqr = vehConsts.propellerTorqueFromThrust
      * vehConsts.propellerThrustFromSpeedSqr;

  double motorTimeConst = vehConsts.motorTimeConst;  // [s]
  double motorInertia = vehConsts.motorInertia;     // [kg.m**2]
  double motorMinSpeed = vehConsts.motorMinSpeed;   // [rad/s]
  double motorMaxSpeed = vehConsts.motorMaxSpeed;   // [rad/s]

  Vec3d centreOfMassError = Vec3d(0, 0, 0);

  Eigen::Matrix<double, 3, 3> inertiaMatrix;
  inertiaMatrix << inertia_xx, 0, 0, 0, inertia_yy, 0, 0, 0, inertia_zz;

  //drag coefficients
  Vec3d linDragCoeffB = Vec3d(vehConsts.linDragCoeffBx,
                              vehConsts.linDragCoeffBy,
                              vehConsts.linDragCoeffBz);

  v->vehicle.reset(
      new Simulation::Quadcopter(&simTimer, mass, inertiaMatrix, armLength,
                                 centreOfMassError, motorMinSpeed,
                                 motorMaxSpeed, propThrustFromSpeedSqr,
                                 propTorqueFromSpeedSqr, motorTimeConst,
                                 motorInertia, linDragCoeffB, v->id,
                                 quadcopterType, 1.0 / frequencySimulation));


  // set initial positions
  double initPosx = 0;
  double initPosy = 0;

  Vec3d const initErrPos = Vec3d(initPosx, initPosy, 0);
  Rotationd const initErrAtt = Rotationd::FromEulerYPR(0.0 * M_PI / 180.0,
                                                       0.0 * M_PI / 180.0, 0);

  cout << "Adding vehicle # " << int(v->id) << " ("
       << Onboard::QuadcopterConstants::GetNameString(quadcopterType)
       << ") at <" << toCSV(initErrPos) << ">\n";
  v->vehicle->SetPosition(initErrPos);
  v->vehicle->SetAttitude(initErrAtt);
  v->_initPos = initErrPos;

  //Seems that the VIO cooridnate and world coordinate are the same. Double check this.
  v->_R_wo = Rotationd::FromEulerYPR(0.0 * M_PI / 180.0, 0.0 * M_PI / 180.0,
                                     0.0 * M_PI / 180.0);

  v->cmdRadioChannel.queue.reset(
      new Simulation::CommunicationsDelay<
          RadioTypes::RadioMessageDecoded::RawMessage>(
          &simTimer, timeDelayOffboardControlLoop));

  cout << "Starting simulation\n";
  Timer t(&simTimer);
  double timePrintNextInfo = 0;
  double timePublishNextGPS = 0;
  double timePublishNextTelemetry = 0;
  double timePublishNextIMU = 0;
  double timePublishNextOdometry = 0;
  double timeGetNextDepthImage = 0;

  ros::Rate loop_rate(frequencySimulation);

  Timer cmdRadioTimer(&simTimer);

  //where we want the quadcopter to fly to:
  while (ros::ok()) {
    ros::spinOnce();
    //want to fly a sinusoid:

    ros::Time currentTime(t.GetSeconds_f());  //Ros time takes floating point seconds
    v->simTimeMsg.clock = currentTime;
    v->pubSimTime->publish(v->simTimeMsg);  //Publish current time to roscore so simulated time is advanced


    for (auto v : vehicles) {
      v->vehicle->Run();
    }

    if (t.GetSeconds<double>() > timePrintNextInfo) {
      timePrintNextInfo += 1;
      cout << "Current sim time = " << int(t.GetSeconds<double>() + 0.5)
           << "s; ";
      for (auto v : vehicles) {
        cout << "(" << v->id << ") pos = <" << toCSV(v->vehicle->GetPosition())
             << ">";
        cout << "\t(" << v->id << ") att = <"
             << toCSV(v->vehicle->GetAttitude().ToEulerYPR()) << ">";
      }
      cout << "\n";
    }

    {
      //see if there are any new radio messages. We put this in {braces} for the scoped mutex trick
      std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
      if (v->cmdRadioChannel.queue->HaveNewMessage()) {
        v->vehicle->SetCommandRadioMsg(v->cmdRadioChannel.queue->GetMessage());
      }
    }


    if (t.GetSeconds<double>() > timePublishNextOdometry) {
      timePublishNextOdometry += 1 / frequencyOdometry;
      nav_msgs::Odometry OdometryOuMsg;
      OdometryOuMsg.header.stamp = ros::Time::now();
      OdometryOuMsg.header.frame_id = "odom";
      //TODO: noise!
      //TODO: Double check this: T265 Odometry use the same coordinate as the traditional ros coordinate (and hence same as our coordinate)
      Vec3d odometryPos = v->vehicle->GetPosition() - v->_initPos;
      Rotationd odometryAtt = v->vehicle->GetAttitude();

      Vec3d measVel_world = v->vehicle->GetVelocity();
      Vec3d measVel_Body = (v->vehicle->GetAttitude()).Inverse()
          * measVel_world;
      Vec3d measAngVel_Body = v->vehicle->GetAngularVelocity();

      // Odometry publishes position and orientation in inertia frame. The initial position is seen as the origin of the frame
      OdometryOuMsg.pose.pose.position.x = odometryPos.x;
      OdometryOuMsg.pose.pose.position.y = odometryPos.y;
      OdometryOuMsg.pose.pose.position.z = odometryPos.z;

      OdometryOuMsg.pose.pose.orientation.w = odometryAtt[0];
      OdometryOuMsg.pose.pose.orientation.x = odometryAtt[1];
      OdometryOuMsg.pose.pose.orientation.y = odometryAtt[2];
      OdometryOuMsg.pose.pose.orientation.z = odometryAtt[3];

      // Odometry publishes velocity and angular velocity with respect to the vehicle frame
      // TODO !! For now we assume the camera is mounted close to center of mass.
      OdometryOuMsg.child_frame_id = "base_link";
      OdometryOuMsg.twist.twist.linear.x = measVel_Body.x;
      OdometryOuMsg.twist.twist.linear.y = measVel_Body.y;
      OdometryOuMsg.twist.twist.linear.z = measVel_Body.z;

      OdometryOuMsg.twist.twist.angular.x = measAngVel_Body.x;
      OdometryOuMsg.twist.twist.angular.y = measAngVel_Body.y;
      OdometryOuMsg.twist.twist.angular.z = measAngVel_Body.z;
      v->pubOdometry->publish(OdometryOuMsg);
    }

    if (t.GetSeconds<double>() > timePublishNextGPS) {
      timePublishNextGPS += 1 / frequencyGPSOutput;
      hiperlab_rostools::gps_output gpsOutMsg;
      gpsOutMsg.header.stamp = ros::Time::now();
      gpsOutMsg.vehicleID = v->id;

      //TODO: add noise!
      Vec3d measPos = v->vehicle->GetPosition();
      gpsOutMsg.posx = measPos.x;
      gpsOutMsg.posy = measPos.y;
      gpsOutMsg.posz = measPos.z;
      v->pubGPS->publish(gpsOutMsg);
    }

    if (t.GetSeconds<double>() > timePublishNextIMU) {
      timePublishNextIMU += 1 / frequencyIMUOutput;
      hiperlab_rostools::imu_output imuOutMsg;
      imuOutMsg.header.stamp = ros::Time::now();
      imuOutMsg.vehicleID = v->id;
      Vec3d accMeasIMU;
      
      v->vehicle->GetAccelerometer(accMeasIMU);
      Vec3d gyroMeasIMU;

      v->vehicle->GetRateGyro(gyroMeasIMU);
      imuOutMsg.accmeasx = accMeasIMU.x;
      imuOutMsg.accmeasy = accMeasIMU.y;
      imuOutMsg.accmeasz = accMeasIMU.z;

      imuOutMsg.gyromeasx = gyroMeasIMU.x;
      imuOutMsg.gyromeasy = gyroMeasIMU.y;
      imuOutMsg.gyromeasz = gyroMeasIMU.z;
      v->pubIMU->publish(imuOutMsg);
    }



    //Publish also the simulation truth:
    hiperlab_rostools::simulator_truth simTruthMsg;
    simTruthMsg.header.stamp = ros::Time::now();
    simTruthMsg.posx = v->vehicle->GetPosition().x;
    simTruthMsg.posy = v->vehicle->GetPosition().y;
    simTruthMsg.posz = v->vehicle->GetPosition().z;

    simTruthMsg.velx = v->vehicle->GetVelocity().x;
    simTruthMsg.vely = v->vehicle->GetVelocity().y;
    simTruthMsg.velz = v->vehicle->GetVelocity().z;

    simTruthMsg.attq0 = v->vehicle->GetAttitude()[0];
    simTruthMsg.attq1 = v->vehicle->GetAttitude()[1];
    simTruthMsg.attq2 = v->vehicle->GetAttitude()[2];
    simTruthMsg.attq3 = v->vehicle->GetAttitude()[3];
    v->vehicle->GetAttitude().ToEulerYPR(simTruthMsg.attyaw,
                                         simTruthMsg.attpitch,
                                         simTruthMsg.attroll);

    simTruthMsg.angvelx = v->vehicle->GetAngularVelocity().x;
    simTruthMsg.angvely = v->vehicle->GetAngularVelocity().y;
    simTruthMsg.angvelz = v->vehicle->GetAngularVelocity().z;

    v->pubSimTruth->publish(simTruthMsg);


    if (t.GetSeconds<double>() > timePublishNextTelemetry) {
      //Telemetry message
      timePublishNextTelemetry += 1 / frequencyTelemetry;

      hiperlab_rostools::telemetry telMsgOut;
      //Fill out the telemetry package
      TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
      v->vehicle->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2);

      TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2;
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1);
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket2);

      telMsgOut.packetNumber = dataPacket1.packetNumber;
      for (int i = 0; i < 3; i++) {
        telMsgOut.accelerometer[i] = dataPacket1.accel[i];
        telMsgOut.rateGyro[i] = dataPacket1.gyro[i];
        telMsgOut.position[i] = dataPacket1.position[i];
      }

      for (int i = 0; i < 4; i++) {
        telMsgOut.motorForces[i] = dataPacket1.motorForces[i];
      }
      telMsgOut.batteryVoltage = dataPacket1.battVoltage;

      for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS;
          i++) {
        telMsgOut.debugVals[i] = dataPacket2.debugVals[i];
      }

      Vec3f attYPR = Rotationf::FromVectorPartOfQuaternion(
          Vec3f(dataPacket2.attitude[0], dataPacket2.attitude[1],
                dataPacket2.attitude[2])).ToEulerYPR();
      for (int i = 0; i < 3; i++) {
        telMsgOut.velocity[i] = dataPacket2.velocity[i];
        telMsgOut.attitude[i] = dataPacket2.attitude[i];
        telMsgOut.attitudeYPR[i] = attYPR[i];
      }
      telMsgOut.panicReason = dataPacket2.panicReason;
      telMsgOut.warnings = dataPacket2.warnings;

      telMsgOut.header.stamp = ros::Time::now();

      v->pubTelemetry->publish(telMsgOut);
    }

    loop_rate.sleep();
  }
  cout << "Done.\n";
}
