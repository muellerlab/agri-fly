/* An estimator that uses measurements from the GPS, and generates
 * an estimate of the vehicle's position, velocity, orientation, and angular velocity.
 */

#pragma once
#include <mutex>
#include <deque>
#include <Eigen/Dense>
#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"
#include "Common/Time/Timer.hpp"
#include "Common/Time/ManualTimer.hpp"
#include "PredictionPipe.hpp"
#include "EstimatedState.hpp"

//TODO FIXME: this needs to be thread-safe, mutex

namespace Offboard {
/**! A state estimator that tracks a 6DOF object, integrating IMU signal for prediction step + GPS for measurement update.
 */
class GPSIMUStateEstimator {

  enum {
    I_POS = 0,  // where in the state vector is the position
    I_VEL = 3,  // where in the state vector is the velocity
    I_ATT = 6,  // where in the state vector is the attitude components
    NUM_STATES = 9,  // size of state vector
  };

 public:
  GPSIMUStateEstimator(BaseTimer* const masterTimer, unsigned const id);

  Vec3d GetPosition() const {
    return _pos;
  }

  Vec3d GetVelocity() const {
    return _vel;
  }

  Rotationd GetAttitude() const {
    return _att;
  }

  Vec3d GetAngularVelocity() const {
    return _angVel;
  }


  unsigned GetResetCount() const {
    return _numResets;
  }


  void Predict(Vec3d const acc, Vec3d const gyro);  //Kalman filter prediction step.

  void Reset();  //Hard reset of all internal states

  void SetStatistics(double measNoiseStdDevPos, double procNoiseStdDevAcc, double procNoiseStdDevAngVel) {
    _measNoiseStdDevPos = measNoiseStdDevPos;
    _measNoiseStdDevAccelerometer = procNoiseStdDevAcc;
    _measNoiseStdDevRateGyro = procNoiseStdDevAngVel;
  }

  unsigned GetID() const {
    return _id;
  }


  void SetPosition(Vec3d in) {
    std::lock_guard<std::mutex> guard(_mutexEstimate);
    _pos = in;
  }

  void SetVelocity(Vec3d in) {
    std::lock_guard<std::mutex> guard(_mutexEstimate);
    _vel = in;
  }

  void SetAttitude(Rotationd in) {
    std::lock_guard<std::mutex> guard(_mutexEstimate);
    _att = in;
  }

  void SetAngularVelocity(Vec3d in) {
    std::lock_guard<std::mutex> guard(_mutexEstimate);
    _angVel = in;
  }

  void UpdateWithMeasurement(Vec3d const position);

  EstimatedState GetCurrentEstimate();

  double GetTimeSinceLastGoodMeasurement() const {
    return _lastGoodMeasUpdate.GetSeconds<double>();
  }

  bool GetIsInitialized() const {
    return _initialized;
  }

  void ResetVariance();

  //Thread safety:
  std::mutex mutable _mutexEstimate;

 private:
  unsigned _id;

  Timer _estimateTimer;  //time at which estimate is valid
  Timer _lastGoodMeasUpdate;   //time since we last used a measurement
  bool _initialized; 
  unsigned _numResets;

  //our state estimate, updated with measurements, true at time _estimateTimer
  Vec3d _pos, _vel, _angVel;
  Rotationd _att;  // <vector in world frame> = _att*<vector in body frame>

  //filter variance
  Eigen::Matrix<double, 9, 9> _cov; //covariance matrix
  Rotationd _lastMeasAtt;  //for "measuring" of angular velocity
  Vec3d _lastMeasUpdateAttCorrection;

  //Constants:
  double _timeConstantTrackAngVel;  // the time constant with which the system tracks the predicted angular velocity.
  double _measRejectDist;  //[std deviations] how unlikely a measurement do we accept?
  double _systemLatency;  //[s]

  //Noise model:

  double _initStdDevPos;  //[m]
  double _initStdDevVel;  //[m/s]
  double _initStdDevAtt;  //[rad]


  double _measNoiseStdDevPos;  // [m]
  double _measNoiseStdDevAccelerometer;  // translational acceleration [m/s**2]
  double _measNoiseStdDevRateGyro;  // rotational velocity [rad/s]

};
} // namespace Offboard