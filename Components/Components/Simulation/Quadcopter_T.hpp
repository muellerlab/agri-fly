#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include <random>

#include <Eigen/Dense>

#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"
#include "Common/Time/BaseTimer.hpp"

#include "Components/Simulation/SimulationObject6DOF.hpp"
#include "Components/Simulation/Motor.hpp"
#include "Components/Simulation/UWBRadio.hpp"
#include "Components/Logic/QuadcopterLogic.hpp"

namespace Simulation {
template <class logicType>
class Quadcopter_T : public SimulationObject6DOF {
 public:

  Quadcopter_T(BaseTimer* const masterTimer, double mass,
             Eigen::Matrix<double, 3, 3> inertiaMatrix, double armLength,
             Vec3d centreOfMassError, double motorMinSpeed,
             double motorMaxSpeed, double propThrustFromSpeedSqr,
             double propTorqueFromSpeedSqr, double motorTimeConst,
             double motorInertia, Vec3d linDragCoeffB,
             uint8_t id,
             Onboard::QuadcopterConstants::QuadcopterType quadcopterType,
             double onboardLogicPeriod);

  virtual ~Quadcopter_T() {
  }

  virtual void Run();

  double GetMotorForce(unsigned i) const {
    //todo: this is a bit of a hack! should do something nicer than just z-component.
    return _motors[i].GetForce().z;
  }

  //! Set an external force, that remains active until replaced. Expressed in world frame
  void SetExternalForce(Vec3d in) {
    _externalForce = in;
  }

  void SetExternalTorque(Vec3d in) {
    _externalTorque = in;
  }

  void GetEstimate(Vec3f &pos, Vec3f &vel, Rotationf &att,
                   Vec3f &angVel) const {
    _logic.GetEstimate(pos, vel, att, angVel);
  }

  virtual void AddUWBRadioTarget(uint8_t id, Vec3f pos) {
    _logic.AddRangingTargetId(id, pos);
  }

  virtual void SetCommandRadioMsg(
      RadioTypes::RadioMessageDecoded::RawMessage const raw) {
    RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
        raw.raw);
    _logic.SetRadioMessage(msg);
  }

  virtual void GetTelemetryDataPackets(TelemetryPacket::data_packet_t &dataPacket1,
                               TelemetryPacket::data_packet_t &dataPacket2) {
    _logic.GetTelemetryDataPackets(dataPacket1, dataPacket2);
    return;
  }

  virtual void GetAccelerometer(Vec3d &acc) {
    acc = Vec3d(_logic.GetAccelerometer());
    return;
  }

  virtual void GetRateGyro(Vec3d &rateGyro) {
    rateGyro = Vec3d(_logic.GetRateGyro());
    return;
  }


 protected:
  //NOTE! You must add them in the order of the enum, above (FR>LE>RE>RI)
  void AddMotor(Vec3d position, Vec3d spinDirection,
                Motor::PropellerHandedness handedness, double minSpeed,
                double maxSpeed, double thrustFromSpeedSqr,
                double torqueFromSpeedSqr, double timeConst, double inertia) {
    _motors.push_back(
        Motor(_integrationTimer.GetMasterTimer(), position, spinDirection,
              handedness, minSpeed, maxSpeed, thrustFromSpeedSqr,
              torqueFromSpeedSqr, timeConst, inertia));
    _motorSpeedCommands.push_back(0);
  }

  logicType _logic;
  std::vector<float> _motorSpeedCommands;

 private:

  Eigen::Matrix<double, 3, 3> _inertiaMatrix;
  Eigen::Matrix<double, 3, 3> _inertiaMatrixInv;
  double _mass;

  Vec3d _externalForce;  //external force acting on vehicle, in [N], and expressed in world frame
  Vec3d _externalTorque; //external torque acting on vehicle, in [Nm], and expressed in world frame

  // Drag coefficients
  Vec3d _linDragCoeffB; // [N*s/m] For x, y and z components of body frame

  std::vector<Motor> _motors;

  float _battVoltage;
  float _battCurrent;

  //Noise characteristics:
  double _stdDevAccNoise, _stdDevRateGyroNoise;

  std::default_random_engine _generator;
  std::normal_distribution<double> _normalDistribution;

  Timer _timerOnboardLogic;
  double _onboardLogicPeriod;

  float _IMU_yaw; //[rad]
  float _IMU_pitch;
  float _IMU_roll;
  Matrix<float, 3, 3> _R_inverse; //rotation matrix correponds to IMU's rotation
};

typedef Quadcopter_T<Onboard::QuadcopterLogic> Quadcopter;

}  //namespace Simulation
