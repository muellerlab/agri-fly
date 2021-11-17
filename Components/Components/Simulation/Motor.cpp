#include <assert.h>

#include "Components/Simulation/Motor.hpp"

using namespace Simulation;

Motor::Motor(BaseTimer* const timer, Vec3d position, Vec3d rotAxis,
             PropellerHandedness handedness, double minSpeed, double maxSpeed,
             double thrustFromSpeedSqr, double torqueFromSpeedSqr,
             double timeConst, double inertia)
    : SimulationObject(timer),
      _minSpeed(minSpeed),
      _maxSpeed(maxSpeed),
      _thrustFromSpeedSqr(thrustFromSpeedSqr),
      _torqueFromSpeedSqr(torqueFromSpeedSqr),
      _timeConstant(timeConst),
      _inertia(inertia),
      _speed(0),
      _position(position),
      _rotAxis(rotAxis),
      _thrustAxis(Vec3d(0, 0, 0)),
      _thrust(Vec3d(0, 0, 0)),
      _torque(Vec3d(0, 0, 0)),
      _angularMomentum(Vec3d(0, 0, 0)),
      _powerConsumptionInstantaneous(0),
      _speedCmd(0) {
  assert(thrustFromSpeedSqr >= 0);
  assert(torqueFromSpeedSqr >= 0);
  assert(maxSpeed > minSpeed);
  assert(fabs(rotAxis.GetNorm2() - 1) < 1e-6);  //expect a unit vector.

  if (handedness == PROP_CLOCKWISE) {
    _thrustAxis = _rotAxis;
  } else {
    _thrustAxis = -_rotAxis;
  }
}

void Motor::Run() {
  const double dt = _integrationTimer.GetSeconds<double>();
  if (dt < 1e-6) {
    return;
  }
  _integrationTimer.Reset();

  double oldSpeed = _speed;

  if (_speedCmd < 0) {
    _speedCmd = 0;
  }

  //discretize time constant
  double c;
  if (_timeConstant == 0) {
    c = 0;
  } else {
    c = exp(-dt / _timeConstant);
  }

  _speed = c * _speed + (1 - c) * _speedCmd;

  if (_speed > _maxSpeed) {
    _speed = _maxSpeed;
  } else if (_speed < _minSpeed) {
    _speed = _minSpeed;
  }

  _angularMomentum = _speed * _inertia * _rotAxis;

  _thrust = _thrustFromSpeedSqr * _speed * fabs(_speed) * _thrustAxis;
  _torque = Vec3d(0, 0, 0);
  //aero torque
  _torque += -_torqueFromSpeedSqr * _speed * fabs(_speed) * _rotAxis;

  //torque from thrust acting at distance from com:
  _torque += _position.Cross(_thrust);
  //add moment due to acceleration of propeller:
  double angularAcceleration = (_speed - oldSpeed) / dt;  //scalar
  _torque -= angularAcceleration * _inertia * _rotAxis;

  _powerConsumptionInstantaneous = _speed * _torque.GetNorm2();
  //TODO: Should add the change in kinetic energy of the propeller, difference between speedup & slowdown.
  return;
}
