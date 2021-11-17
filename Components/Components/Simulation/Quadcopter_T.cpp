#include "Components/Simulation/Quadcopter_T.hpp"

using namespace Simulation;

double const ACCELEROMETER_NOISE_STD_DEV = 0.2;
double const RATE_GYRO_NOISE_STD_DEV = 0.1;

template<class logicType>
Quadcopter_T<logicType>::Quadcopter_T(
    BaseTimer* const masterTimer, double mass,
    Eigen::Matrix<double, 3, 3> inertiaMatrix, double armLength,
    Vec3d centreOfMassError, double motorMinSpeed, double motorMaxSpeed,
    double propThrustFromSpeedSqr, double propTorqueFromSpeedSqr,
    double motorTimeConst, double motorInertia, Vec3d linDragCoeffB, uint8_t id,
    Onboard::QuadcopterConstants::QuadcopterType quadcopterType,
    double onboardLogicPeriod)
    : SimulationObject6DOF(masterTimer),
      _logic(masterTimer, float(onboardLogicPeriod)),
      _inertiaMatrix(inertiaMatrix),
      _inertiaMatrixInv(inertiaMatrix.inverse()),
      _mass(mass),
      _externalForce(0, 0, 0),
      _externalTorque(0, 0, 0),
      _linDragCoeffB(linDragCoeffB),
      _stdDevAccNoise(ACCELEROMETER_NOISE_STD_DEV),  //No idea if this is accurate or not...
      _stdDevRateGyroNoise(RATE_GYRO_NOISE_STD_DEV),  //guesstimated from some flight data. Probably not very accurate
      _generator(),
      _normalDistribution(0, 1),
      _timerOnboardLogic(masterTimer),
      _onboardLogicPeriod(onboardLogicPeriod) {

  // Create the motors
  //For the crazyflie, we have:

  /*
   *       x
   *       ^
   *   3   |    0
   *       |
   * y<----+-----
   *       |
   *   2   |    1
   */

  Vec3d const spinDir(0, 0, 1);
  //Front right
  AddMotor(armLength / sqrt(2) * Vec3d(+1, -1, 0) + centreOfMassError, +spinDir,
           Motor::PROP_CLOCKWISE, motorMinSpeed, motorMaxSpeed,
           propThrustFromSpeedSqr, propTorqueFromSpeedSqr, motorTimeConst,
           motorInertia);
  //Rear right
  AddMotor(armLength / sqrt(2) * Vec3d(-1, -1, 0) + centreOfMassError, -spinDir,
           Motor::PROP_COUNTERCLOCKWISE, motorMinSpeed, motorMaxSpeed,
           propThrustFromSpeedSqr, propTorqueFromSpeedSqr, motorTimeConst,
           motorInertia);
  //Rear left
  AddMotor(armLength / sqrt(2) * Vec3d(-1, +1, 0) + centreOfMassError, +spinDir,
           Motor::PROP_CLOCKWISE, motorMinSpeed, motorMaxSpeed,
           propThrustFromSpeedSqr, propTorqueFromSpeedSqr, motorTimeConst,
           motorInertia);
  //Front left
  AddMotor(armLength / sqrt(2) * Vec3d(+1, +1, 0) + centreOfMassError, -spinDir,
           Motor::PROP_COUNTERCLOCKWISE, motorMinSpeed, motorMaxSpeed,
           propThrustFromSpeedSqr, propTorqueFromSpeedSqr, motorTimeConst,
           motorInertia);

  //create the radio
  _radio.reset(new UWBRadio(masterTimer, id));

  //TODO: Battery simulation to be done.
  Onboard::QuadcopterConstants consts(quadcopterType);
  _battVoltage = 1.2 * consts.lowBatteryThreshold;
  _battCurrent = -1.0;

  _IMU_yaw = consts.IMU_yaw;
  _IMU_pitch = consts.IMU_pitch;
  _IMU_roll = consts.IMU_roll;
  _R_inverse =
      Rotationf::FromEulerYPR(_IMU_yaw, _IMU_pitch, _IMU_roll).Inverse()
          .GetRotationMatrix();

  _logic.Initialise(quadcopterType, id);
}

template<class logicType>
void Quadcopter_T<logicType>::Run() {
  const double dt = _integrationTimer.GetSeconds<double>();
  if (dt < 1e-6) {
    return;
  }
  _integrationTimer.Reset();
  //force & torque in body-fixed frame
  Vec3d totalForce_b(0, 0, 0);
  Vec3d totalTorque_b(0, 0, 0);

  int i = 0;
  for (auto m = _motors.begin(); m != _motors.end(); m++) {
    m->SetSpeedCommand(double(_motorSpeedCommands[i]));
    m->Run();
    i++;

    totalForce_b += m->GetForce();
    totalTorque_b += m->GetTorque();
  }

  totalTorque_b += _att.Inverse() * _externalTorque;

  //add noise
  //TODO FIXME
  //totalForce_b +=
  //totalTorque_b +=

  Vec3d angMomentum = _inertiaMatrix * _angVel;

  for (auto m = _motors.begin(); m != _motors.end(); m++) {
    angMomentum += m->GetAngularMomentum();
  }

  Vec3d angAcceleration = _inertiaMatrixInv
      * (totalTorque_b - _angVel.Cross(angMomentum));

  // drag force
  Rotationd R_wb = _att;
  Vec3d vel_b = R_wb.Inverse() * _vel;
  Vec3d dragForce_b = Vec3d(_linDragCoeffB.x * (-vel_b.x),
                            _linDragCoeffB.y * (-vel_b.y),
                            _linDragCoeffB.z * (-vel_b.z));
  totalForce_b += dragForce_b;

  //do the integration:
  Vec3d acc = Vec3d(0, 0, -9.81);  //gravity
  acc += (_att * totalForce_b + _externalForce) / _mass;

  //temp variables
  Vec3d pos = _pos;
  Vec3d vel = _vel;
  Rotationd att = _att;
  Vec3d angVel = _angVel;

  Vec3d newpos = pos + vel * dt + 0.5 * acc * dt * dt;
  Vec3d newvel = vel + acc * dt;
  Rotationd newatt = att * Rotationd::FromRotationVector(angVel * dt);
  Vec3d newangVel = angVel + angAcceleration * dt;
  //Test for ground contact:

  if ((newpos.z <= 0) and (newvel.z < 0)) {
    newpos.z = 0;
    newvel.z = 0;
    acc.z = 0;
    newangVel = Vec3d(0, 0, 0);
  }

  _pos = newpos;
  _vel = newvel;
  _att = newatt;
  _angVel = newangVel;

  //Run the onboard logic:
  if (_timerOnboardLogic.GetSeconds<double>() > _onboardLogicPeriod) {
    _timerOnboardLogic.AdjustTimeBySeconds(-_onboardLogicPeriod);

    //TODO: worry about timing for these sensors (should they really be updating every loop?)
    _logic.SetBatteryMeasurement(_battVoltage, _battCurrent);

    Vec3f rateGyroMeas(_angVel);  // TODO + noise, scaling error, biases
    rateGyroMeas = _R_inverse * rateGyroMeas;  // Rotate IMU measurements based on IMU orientation
    rateGyroMeas += Vec3f(float(_normalDistribution(_generator)),
                          float(_normalDistribution(_generator)),
                          float(_normalDistribution(_generator)))
        * float(_stdDevRateGyroNoise);
    _logic.SetIMUMeasurementRateGyro(rateGyroMeas.x, rateGyroMeas.y,
                                     rateGyroMeas.z);

    Vec3f accMeas = Vec3f(_att.Inverse() * (acc + Vec3d(0, 0, 9.81)));  // TODO + noise, scaling error, biases
    accMeas = _R_inverse * accMeas;  // Rotate IMU measurements based on IMU orientation
    accMeas += Vec3f(float(_normalDistribution(_generator)),
                     float(_normalDistribution(_generator)),
                     float(_normalDistribution(_generator)))
        * float(_stdDevAccNoise);
    _logic.SetIMUMeasurementAccelerometer(accMeas.x, accMeas.y, accMeas.z);

    float const TEMP_MEAS = 25;  //made up temperature for simulation
    _logic.SetIMUMeasurementTemperature(TEMP_MEAS);

    _logic.Run();
    //output the motor forces:
    for (unsigned int i = 0; i < 4; i++) {
      _motorSpeedCommands[i] = _logic.GetMotorSpeedCmd(i);
    }

    if (_radio) {
      //Talk to the UWB radio:
      _radio->SetPosition(_pos);
      _radio->SetNextRangingTarget(_logic.GetNextUWBRangingTarget());
      if (_radio->GetHaveNewMeasurement()) {
        UWBRadio::RangingMeasurement meas = _radio->GetMeasurement();
        _logic.SetUWBMeasurement(meas.range, meas.responderId, meas.failure);
      }
    }
  }

  return;
}

namespace Simulation {
template class Quadcopter_T<Onboard::QuadcopterLogic> ;
}
