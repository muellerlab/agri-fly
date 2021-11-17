/* An estimator that uses measurements from the motion capture system, and generates
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

//TODO FIXME: this needs to be thread-safe, mutex

namespace Offboard {

template<class T_msg>
class PredictionPipe {
  static constexpr double SMALL_TIME = 1e-6;  // [s]
 public:
  PredictionPipe(BaseTimer* const timer, double delayTime)
      : _timer(timer),
        _delayTime(delayTime) {
    //Do nothing
  }

  void AddMessage(T_msg msg) {
    TimedMessage tmsg;
    tmsg.timeActive = _timer.GetSeconds<double>() + _delayTime;  //message is sent now, will be active after _delayTime
    tmsg.msg = msg;
    _messages.push_back(tmsg);
  }

  bool GetActiveMessage(double t, T_msg &out, double &timeRemaining) const {
    if (!_messages.size()) {
      return false;
    }
    //beginning at the end, find the newest message that's valid now.
    // Returns false if no valid message exists.
    auto msg = _messages.cend();
    double tLastMsg = 1e10;
    for (;;) {
      msg--;
      if ((t + SMALL_TIME) >= msg->timeActive) {
        out = msg->msg;
        timeRemaining = tLastMsg - msg->timeActive;
        return true;
      }
      tLastMsg = msg->timeActive;
      if (msg == _messages.cbegin()) {
        break;
      }
    }
    return false;
  }

  void ClearExpiredMessages(double currentTime) {
    int N = _messages.size();
    //this is inelegant
    for (int i = 0; i < N; i++) {
      if (_messages.size() < 2) {
        //last message always stays active.
        return;
      }

      if (_messages[1].timeActive <= currentTime) {
        _messages.pop_front();
      }
    };
  }

 private:
  struct TimedMessage {
    double timeActive;  //[s]
    T_msg msg;
  };

  Timer _timer;
  double _delayTime;
  std::deque<TimedMessage> _messages;

};

/**! A state estimator that tracks a 6DOF object, using mocap measurements
 * The estimator totally decouples position from attitude, and treats each position axis
 * separately.
 *
 * In translation, we don't do anything funny. For attitude, we assumea  first order
 * system is tracking attitude commands.
 */
class MocapStateEstimator {
 public:
  struct MocapEstimatedState {
    Vec3d pos, vel, angVel;
    Rotationd att;
  };

  MocapStateEstimator(BaseTimer* const masterTimer, unsigned const id,
                      double standardCommunicationsDelay);

  void SetStatistics(double measNoiseStdDevPos, double measNoiseStdDevAtt,
                     double procNoiseStdDevPos, double procNoiseStdDevAtt) {
    _measNoiseStdDevPos = measNoiseStdDevPos;
    _measNoiseStdDevAtt = measNoiseStdDevAtt;
    _procNoiseStdDevPos = procNoiseStdDevPos;
    _procNoiseStdDevAtt = procNoiseStdDevAtt;
  }

  unsigned GetID() const {
    return _id;
  }
  // This just predicts the states by running the dynamic for the dt second latency between estimator & controller.
  MocapEstimatedState GetPrediction(double const dt) const;

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

  // This is the whole process of state estimation including initialization, prediction, and measurement update.
  void UpdateWithMeasurement(Vec3d const position, Rotationd const attitude);

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
  Rotationd _att;  //<vector in world frame> = _att*<vector in body frame>

  //filter variance
  Eigen::Matrix<double, 2, 2> _variancePosition;
  Eigen::Matrix<double, 2, 2> _varianceAttitude;

  Rotationd _lastMeasAtt;  //for "measuring" of angular velocity

  unsigned _numMeasRejected, _numMeasRejectedConsecutively;

  //Constants:
  double _timeConstantTrackAngVel;  // the time constant with which the system tracks the predicted angular velocity.
  double _measRejectDist;  //[std deviations] how unlikely a measurement do we accept?
  double _systemLatency;  //[s]

  //Noise model:
  double _measNoiseStdDevPos;  // [m]
  double _measNoiseStdDevAtt;  // [rad]
  double _procNoiseStdDevPos;  // translational acceleration [m/s**2]
  double _procNoiseStdDevAtt;  // angular velocity [rad/s**2]

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
