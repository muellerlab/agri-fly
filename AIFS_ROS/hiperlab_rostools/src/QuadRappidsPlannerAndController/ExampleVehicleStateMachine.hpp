#pragma once

#include "ros/ros.h"

#include "Common/Math/Vec3.hpp"
#include "Common/Math/Trajectory.hpp"
#include "Components/TrajectoryGenerator/RapidTrajectoryGenerator.hpp"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Offboard/MocapStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"
#include "Components/DepthImagePlanner/DepthImagePlanner.hpp"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"
#include "hiperlab_rostools/telemetry.h"

#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <stdio.h>
#include <unistd.h>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

// Code for image publisher
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Message for debugging purpose
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "hiperlab_rostools/planner_diagnostics.h"
#include "hiperlab_rostools/controller_diagnostics.h"

// Code for VIO
#include <nav_msgs/Odometry.h>
namespace Offboard {

// Tool to translate our vector to ros geometry msg
inline geometry_msgs::Vector3 Vec3dToVector3(const Vec3d &rhs) {
  geometry_msgs::Vector3 lhs;
  lhs.x = rhs.x;
  lhs.y = rhs.y;
  lhs.z = rhs.z;
  return lhs;
}

inline geometry_msgs::Quaternion RotationdToQuaternion(const Rotationd &rhs) {
  geometry_msgs::Quaternion lhs;
  lhs.w = rhs[0];
  lhs.x = rhs[1];
  lhs.y = rhs[2];
  lhs.z = rhs[3];
  return lhs;
}

inline geometry_msgs::Quaternion RotationfToQuaternion(const Rotationf &rhs) {
  geometry_msgs::Quaternion lhs;
  lhs.w = rhs[0];
  lhs.x = rhs[1];
  lhs.y = rhs[2];
  lhs.z = rhs[3];
  return lhs;
}

class ExampleVehicleStateMachine {
 public:
  enum FlightStage {
    StageWaitForStart,
    StageSpoolUp,
    StageTakeoff,
    StageHover,
    StageFlight,
    StageLanding,
    StageComplete,
    StageEmergency,
  };

  ExampleVehicleStateMachine();

  bool GetIsEstInitialized(void) const {
    return _est->GetIsInitialized();
  }

  bool GetIsReadyToExit(void) const {
    return _vehicleIsReadyForProgramToExit;
  }

  void Initialize(int id, std::string name, ros::NodeHandle &n,
                  BaseTimer *timer, double systemLatencyTime);

  void CallbackEstimator(const hiperlab_rostools::mocap_output &msg);
  void CallbackTelemetry(const hiperlab_rostools::telemetry &msg);
  void CallbackDepthImages(const sensor_msgs::ImageConstPtr &msg);
  void CallbackOdometry(const nav_msgs::Odometry &msg);

  void Run(bool shouldStart, bool shouldStop);

  void PublishEstimate(MocapStateEstimator::MocapEstimatedState estState);

  hiperlab_rostools::radio_command RunControllerAndUpdateEstimator(
      MocapStateEstimator::MocapEstimatedState estStatCameraSubscribere,
      Vec3d desPos, Vec3d desVel, Vec3d desAcc);

  hiperlab_rostools::radio_command RunTrackingControllerAndUpdateEstimator(
      MocapStateEstimator::MocapEstimatedState estState, Vec3d refPos,
      Vec3d refVel, Vec3d refAcc, double refThrust, Vec3d refAngVel,
      double &cmdThrustOutput, Rotationf &cmdAttOutput, Vec3d &cmdAngVelOutput);

  void SetDesiredPosition(Vec3d newPos) {
    _desiredPosition = newPos;
  }

  void SetDesiredYaw(double newYaw) {
    _desiredYawAngle = newYaw;
  }

  void SetExternalPanic() {
    _safetyNet->SetUnsafe();
  }

  static double GetTrajCostWrapper(
      void *ptr2obj,
      RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &traj) {
    ExampleVehicleStateMachine *veh = (ExampleVehicleStateMachine*) ptr2obj;
    return veh->GetTrajCost(traj);
  }

  double GetTrajCost(
      RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &traj) {
    double duration = traj.GetFinalTime();
    Vec3d Pi_C = traj.GetPosition(duration);  // position of camera at end of trajectory relative to start written in camera frame

    Vec3d G_W = _goalWorld;                               // goal in world frame
    Vec3d G_C = _depthCamAtt.Inverse() * _estAtt.Inverse()
        * (_goalWorld - _estPos);  // goal in camera frame
    Vec3d S_C = Vec3d(0, 0, 0);                // starting point in camera frame

    double SG = (G_C - S_C).GetNorm2();
    double PiG = (G_C - Pi_C).GetNorm2();
    return -(SG - PiG) / duration;
  }

 private:
  bool HaveLowBattery() const {
    return _lastTelWarnings & TelemetryPacket::WARN_LOW_BATT;
  }
  int _id;
  std::shared_ptr<MocapStateEstimator> _est;  // This is be a misnomer as we will be using tracking camera for correction update instead of mocap
  double _systemLatencyTime;  //[s] as used by estimator
  std::shared_ptr<QuadcopterController> _ctrl;
  std::shared_ptr<SafetyNet> _safetyNet;
  std::string _name;
  std::shared_ptr<ros::Subscriber> _subMocap, _subTelemetry, _subOdometry;
  std::shared_ptr<ros::Publisher> _pubEstimate, _pubCmd, _pubPlannerDiagnotics,
      _pubControllerDiagnotics;

  image_transport::Subscriber _subDepthImages;

//state info:
  FlightStage _flightStage, _lastFlightStage;
  std::shared_ptr<Timer> _stageTimer;  //keep track of time we've been in current stage

  volatile uint8_t _lastTelWarnings;

  Vec3d _initPosition;
  Vec3d _desiredPosition;
  double _desiredYawAngle;

  bool _vehicleIsReadyForProgramToExit;

  // Values from previous time step / stage
  Vec3d _lastPos;
  Vec3d _lastVel;
  Vec3d _lastAcc;
  double _lastYaw;
  double _cmdYawAngle;  // Might want to use this for some trajectories

  // Variables for depth image planner
  double _depthScale;
  Rotationd _depthCamAtt;
  double _focalLength;
  unsigned _depthImageHeight;
  unsigned _depthImageWidth;
  Rotationd _trajAtt;

  unsigned _depthImageCount;

  //This estimation global variables will be used for planning
  Rotationd _estAtt;
  Vec3d _estPos;
  Vec3d _estVel;  //In world frame
  Vec3d _estAngVel;  //In body frame

  bool _imageReady;
  bool _startPlan;
  bool _firstTrajReady;
  unsigned _plannedTrajCount;
  double _physicalVehicleRadius;
  double _vehicleRadiusPlanning;
  double _minCollisionDist;
  double _previousThrust;
  Vec3d _trajOffset;  //Offset position of the the traj from camera frame to world frame
  Vec3d _goalWorld;
  std::vector<RectangularPyramidPlanner::TrajectoryTest> _depthPlannedTraj;
  std::shared_ptr<RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator> _plannedTraj;
  std::shared_ptr<Timer> _timeOnPlannedTraj;  //keep track of time we've been on the current trajectory.

  // Variables keep track of the planned trajectories
  double _plannedTrajDuration;
  double _lookAheadTime;

  std::ofstream _imageReceiveLog;
  std::ofstream _plannedTrajLog;

  ros::Time _trajResetTime;

  // Debugging thread publisher

};

}  // namespace Offboard
