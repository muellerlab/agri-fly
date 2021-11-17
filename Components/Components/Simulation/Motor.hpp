#pragma once

#include "Common/Math/Vec3.hpp"
#include "Components/Simulation/SimulationObject.hpp"

namespace Simulation {

//we assume that the thrust always points up in body frame (positive z-axis)
//all vectors expressed in body-fixed-frame
//we limit thrust to be positive.
//torque is about the vehicle's COM
class Motor : SimulationObject {
 public:
  enum PropellerHandedness {
    PROP_CLOCKWISE,  // looking from "above" the propeller (so the thrust points down)
    PROP_COUNTERCLOCKWISE,  // looking from "above" the propeller (so the thrust points down)
  };

  Motor(BaseTimer* const timer, Vec3d position, Vec3d rotAxis,
        PropellerHandedness handedness, double minSpeed, double maxSpeed,
        double thrustFromSpeedSqr, double torqueFromSpeedSqr, double timeConst,
        double inertia);

  void SetSpeedCommand(double cmd) {
    _speedCmd = cmd;
  }

  virtual void Run();

  void SetSpeed(double in) {
    _speed = in;
  }

  inline Vec3d GetForce() const {
    return _thrust;
  }
  inline Vec3d GetTorque() const {
    return _torque;
  }
  inline Vec3d GetAngularMomentum() const {
    return _angularMomentum;
  }
  inline double GetPowerConsumption() const {
    return _powerConsumptionInstantaneous;
  }

 private:

  double _minSpeed, _maxSpeed;  //the achievable speed range (>0) [rad/s]
  double _thrustFromSpeedSqr;  //[N/(rad/s)**2]
  double _torqueFromSpeedSqr;  //[N.m/(rad/s)**2]
  double _timeConstant;  //how quickly the motor responds [s]
  double _inertia;  //MMOI about rotation axis [kg.m**2]

  double _speed;  //current speed [rad/s]
  Vec3d _position;  //position of propeller w.r.t. body centro of mass
  Vec3d _rotAxis, _thrustAxis;
  Vec3d _thrust;  //vector [N]
  Vec3d _torque;  //vector [N.m]
  Vec3d _angularMomentum;  //vector [kg.mm*2*rad/s]

  double _powerConsumptionInstantaneous;  //[W]

  double _speedCmd;

};

}  //namespace Simulation
