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

/**! A state estimator that tracks a 6DOF object, using a 3D position measurements
 * In translation, we don't do anything funny. For attitude, we assumea  first order
 * system is tracking attitude commands.
 */
class GPSStateEstimator {

  enum {
    I_POS = 0,  // where in the state vector is the position
    I_VEL = 3,  // where in the state vector is the velocity
    I_ATT = 6,  // where in the state vector is the attitude components
    NUM_STATES = 9,  // size of state vector
  };

 public:

  GPSStateEstimator(BaseTimer* const masterTimer, unsigned const id,
                      double standardCommunicationsDelay);

  void SetStatistics(double measNoiseStdDevPos, double procNoiseStdDevAcc, double procNoiseStdDevAngVel) {
    _measNoiseStdDevPos = measNoiseStdDevPos;
    _procNoiseStdDevAcc = procNoiseStdDevAcc;
    _procNoiseStdDevAngVel = procNoiseStdDevAngVel;
  }

  unsigned GetID() const {
    return _id;
  }
  // This just predicts the states by running the dynamic for the dt second latency between estimator & controller.
  EstimatedState GetPrediction(double const dt) const;

  unsigned GetPredictedMeasurementRejectionCount() const {
    std::lock_guard<std::mutex> guard(_mutexEstimate);
    return _numMeasRejected;
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

  void SetPredictedValues(Vec3d angVel, Vec3d acceleration) {
    PredictionType p;
    p.acc = acceleration;
    p.angVel = angVel;
    p.ballistic = false;
    _predictionPipe.AddMessage(p);
  }

  void SetBallistic() {
    PredictionType p;
    p.acc = Vec3d(0, 0, -9.81);
    p.angVel = Vec3d(0, 0, 0);
    p.ballistic = true;
    _predictionPipe.AddMessage(p);
  }

  void UpdateWithMeasurement(Vec3d const position);

  double GetTimeSinceLastGoodMeasurement() const {
    return _lastGoodMeasUpdate.GetSeconds<double>();
  }

  bool GetIsInitialized() const {
    return _initialized;
  }

  void Reset();  //Hard reset of all internal states
  void ResetVariance();

  void SetAngularVelocityTimeConstant(double const tau) {
    _timeConstantTrackAngVel = tau;
  }

 private:
  unsigned _id;
  ManualTimer _estimateTimer;  //time at which estimate is valid
  Timer _timer;  //wall clock
  Timer _lastGoodMeasUpdate;  //time since we last used a measurement
  bool _initialized; 

  //our state estimate, updated with measurements, true at time _estimateTimer
  Vec3d _pos, _vel, _angVel;
  Rotationd _att;  // <vector in world frame> = _att*<vector in body frame>

  //filter variance
  Eigen::Matrix<double, 9, 9> _cov; //covariance matrix
  Rotationd _lastMeasAtt;  //for "measuring" of angular velocity
  unsigned _numMeasRejected, _numMeasRejectedConsecutively;
  Vec3d _lastMeasUpdateAttCorrection;

  //Constants:
  double _timeConstantTrackAngVel;  // the time constant with which the system tracks the predicted angular velocity.
  double _measRejectDist;  //[std deviations] how unlikely a measurement do we accept?
  double _systemLatency;  //[s]

  //Noise model:

  double   _initStdDevPos;  //[m]
  double _initStdDevVel;  //[m/s]
  double _initStdDevAtt;  //[rad]
  double _measNoiseStdDevPos;  // [m]
  double _procNoiseStdDevAcc;  // translational acceleration [m/s**2]
  double _procNoiseStdDevAngVel;  // rotational velocity [rad/s]

  //Our prediction info:
  struct PredictionType {
    Vec3d acc;
    Vec3d angVel;
    bool ballistic;  //set to true if we assume ballistic free-fall (ignore acc/angvel)
  };
  
  PredictionPipe<PredictionType> _predictionPipe;

  //Thread safety:
  std::mutex mutable _mutexEstimate;
};

}
