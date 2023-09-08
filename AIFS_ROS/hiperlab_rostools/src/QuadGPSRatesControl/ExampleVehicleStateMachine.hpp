#pragma once

#include "ros/ros.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Offboard/GPSStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"
#include "Components/Offboard/EstimatedState.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"

#include "hiperlab_rostools/gps_output.h"
#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"
#include "hiperlab_rostools/telemetry.h"

namespace Offboard {

class ExampleVehicleStateMachine {
 public:
  enum FlightStage {
    StageWaitForStart,
    StageSpoolUp,
    StageTakeoff,
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
                  BaseTimer* timer, double systemLatencyTime);

  void CallbackEstimator(const hiperlab_rostools::gps_output& msg);
  void CallbackTelemetry(const hiperlab_rostools::telemetry& msg);

  void Run(bool shouldStart, bool shouldStop);

  void PublishEstimate(EstimatedState estState);

  hiperlab_rostools::radio_command RunControllerAndUpdateEstimator(EstimatedState estState, Vec3d desPos,
      Vec3d desVel, Vec3d desAcc);

  void SetDesiredPosition(Vec3d newPos) {
    _desiredPosition = newPos;
  }

  void SetDesiredYaw(double newYaw) {
    _desiredYawAngle = newYaw;
  }

  void SetExternalPanic() {
    _safetyNet->SetUnsafe();
  }

 private:
  bool HaveLowBattery() const {
    return _lastTelWarnings & TelemetryPacket::WARN_LOW_BATT;
  }
  int _id;
  std::shared_ptr<GPSStateEstimator> _est;
  double _systemLatencyTime;  //[s] as used by estimator
  std::shared_ptr<QuadcopterController> _ctrl;
  std::shared_ptr<SafetyNet> _safetyNet;
  std::string _name;

  std::shared_ptr<ros::Subscriber> _subGPS, _subTelemetry;
  std::shared_ptr<ros::Publisher> _pubEstimate, _pubCmd;

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
};

}  // namespace Offboard
