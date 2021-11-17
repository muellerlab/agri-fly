#pragma once

#include "Common/Math/Matrix.hpp"
#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"

#include "Common/Time/Timer.hpp"

namespace Onboard {

/**! A kalman filter that tracks a 6DOF object, using IMU and UWB ranges.
 * No assumptions on the dynamics are made.
 *
 * The 6DOF are three in position, three in orientation.
 *
 * We use the parametrization & tools as described in
 * "Covariance correction step for Kalman filtering with an attitude",
 *  available here: http://muellerlab.berkeley.edu/wp-content/uploads/2016/11/P_2016_CovarianceCorrectionStepForKalmanFilteringWithAnAttitude.pdf
 *
 * The filter takes as input the accelerometer & rate gyroscope (used to do the
 * filter prediction step), and UWB ranges (used for the measurement update).
 * The rate gyro is directly used as the estimate of the angular velocity.
 */
class KalmanFilter6DOF {
  enum {
    I_POS = 0,  // where in the state vector is the position
    I_VEL = 3,  // where in the state vector is the velocity
    I_ATT = 6,  // where in the state vector is the attitude components
    NUM_STATES = 9,  // size of state vector
  };

 public:
  KalmanFilter6DOF(BaseTimer* const masterTimer);

  Vec3f GetPosition() const {
    return _pos;
  }

  Vec3f GetVelocity() const {
    return _vel;
  }

  Rotationf GetAttitude() const {
    return _att;
  }

  Vec3f GetAngularVelocity() const {
    return _angVel;
  }

  unsigned GetMeasurementRejectionCount() const {
    return _numMeasRejected;
  }

  unsigned GetResetCount() const {
    return _numResets;
  }

  unsigned GetWasResetSinceLastCheck() {
    bool change = _lastCheckNumResets != _numResets;
    _lastCheckNumResets = _numResets;
    return change;
  }

  void SetPosition(Vec3f in) {
    _pos = in;
  }

  void SetVelocity(Vec3f in) {
    _vel = in;
  }

  void SetAttitude(Rotationf in) {
    _att = in;
  }

  void SetAngularVelocity(Vec3f in) {
    _angVel = in;
  }

  void Predict(Vec3f const gyro, Vec3f const acc);  //Kalman filter prediction step.
  void UpdateWithRangeMeasurement(Vec3f const targetPosition,
                                  float const range);  //Kalman measurement update, using range measurement from anchor at targetPosition.

  uint64_t GetTimeSinceLastGoodMeasurement_us() const {
    return _timerLastGoodMeasUpdate.GetMicroSeconds();
  }

  bool GetIsIMUInitialized() const {
    return _IMUInitialized;
  }

  bool GetIsUWBInitialized() const {
    return _UWBInitialized;
  }

  void Reset();  //Hard reset of all internal states

 private:
  void MakeCovarianceSymmetric();  //need to call every once in a while to ensure the covariance matrix remains symmetric.

  Timer _estimateTimer;  //when our estimate is valid.
  Timer _timerLastGoodMeasUpdate;

  bool _IMUInitialized, _UWBInitialized;

  //our current state estimate:
  Vec3f _pos, _vel, _angVel;
  Rotationf _att;  //<vector in world frame> = _att*<vector in body frame>
  Vec3f _lastMeasUpdateAttCorrection;
  //covariance:
  Matrix<float, NUM_STATES, NUM_STATES> _cov;

  //Constants:

  float _initStdDevPos;  //[m]
  float _initStdDevVel;  //[m/s]
  float _initStdDevAtt_aboutGravity, _initStdDevAtt_perpToGravity;  //[rad]
  float _measNoiseStdDevRateGyro;  //[rad/s]
  float _measNoiseStdDevAccelerometer;  //[rad/s]
  float _measNoiseStdDevRangeMeasurements;  //[m]

  float _outlierDetectionStatisticalDist;  //Mahalanobis distance to reject measurements [1]
  unsigned _numMeasRejected;
  unsigned _maxNumMeasRejectedSequentially, _numMeasRejectedSequentially;
  unsigned _numResets;
  unsigned _lastCheckNumResets;
};

}  //namespace Onboard
