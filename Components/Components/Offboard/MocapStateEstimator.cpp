#include "MocapStateEstimator.hpp"
#include <iostream>

using namespace Offboard;

unsigned const MAX_NUM_CONSECUTIVE_REJECTION = 10;  //TODO: make this a parameter
double const SMALL_TIME = 1e-6;  // [s]

MocapStateEstimator::MocapStateEstimator(BaseTimer* const masterTimer,
                                         unsigned const id,
                                         double standardCommunicationsDelay)
    : _id(id),
      _estimateTimer(),
      _timer(masterTimer),
      _lastGoodMeasUpdate(masterTimer),
      _initialized(false),
      _predictionPipe(masterTimer, standardCommunicationsDelay) {
  _numMeasRejected = 0;
  _numMeasRejectedConsecutively = 0;

  _measRejectDist = 6.0;

  _timeConstantTrackAngVel = 0.04;  //TODO!
  _systemLatency = 0.0;

  //rough guesses!
  _measNoiseStdDevPos = 0.02;  //[m]
  _measNoiseStdDevAtt = 5 * M_PI / 180;  //[rad]

  _procNoiseStdDevPos = 1.0 * 9.81;  // [m/s**2]
  _procNoiseStdDevAtt = 200;  // [rad/s**2]

  Reset();
}

void MocapStateEstimator::Reset() {
  _initialized = false;
  _pos = Vec3d(0, 0, 0);
  _vel = Vec3d(0, 0, 0);
  _att = Rotationd::Identity();
  _lastMeasAtt = Rotationd::Identity();
  _angVel = Vec3d(0, 0, 0);

  ResetVariance();

  //estimate is valid *now*:
  _estimateTimer.ResetMicroseconds(_timer.GetMicroSeconds());
  _lastGoodMeasUpdate.Reset();
}

void MocapStateEstimator::ResetVariance() {
  _variancePosition(0, 0) = 25.0;  //Magic numbers, but should get drowned out quickly by measurements
  _variancePosition(1, 1) = 25.0;  //Magic numbers, but should get drowned out quickly by measurements
  _variancePosition(0, 1) = _variancePosition(1, 0) = 0.0;

  _varianceAttitude(0, 0) = 1.0;  //Magic numbers, but should get drowned out quickly by measurements
  _varianceAttitude(1, 1) = 400;  //Magic numbers, but should get drowned out quickly by measurements
  _varianceAttitude(0, 1) = _varianceAttitude(1, 0) = 0.0;
}

MocapStateEstimator::MocapEstimatedState MocapStateEstimator::GetPrediction(
    double const dt) const {

  //we want to predict for "dt" and whatever the estimate is behind the wall clock.
  double tStart;
  double tEnd = dt + _timer.GetSeconds<double>();

  MocapEstimatedState est;

  {
    std::lock_guard<std::mutex> guard(_mutexEstimate);
    tStart = _estimateTimer.GetSeconds<double>();
    est.pos = _pos;
    est.vel = _vel;
    est.att = _att;
    est.angVel = _angVel;
  }

  double t = tStart;
  while ((t + SMALL_TIME) < tEnd) {
    double predictionTime = 0;
    PredictionType cmd;
    if (!_predictionPipe.GetActiveMessage(t, cmd, predictionTime)) {
      //no messages
      cmd.acc = Vec3d(0, 0, 0);
      cmd.angVel = Vec3d(0, 0, 0);
      cmd.ballistic = true;
      predictionTime = 1e10;
    }

    //make sure the prediction is valid for the whole period:
    double dtInt = tEnd - t;  //integration time remaining
    if (dtInt > (predictionTime + SMALL_TIME)) {  //add 1e-6 to ensure no shenanigans due to float precision
      dtInt = predictionTime;
    }

    Vec3d newPos = est.pos + _vel * dtInt + cmd.acc * dtInt * dtInt / 2;
    Vec3d newVel = est.vel + cmd.acc * dtInt;
    Rotationd newAtt = est.att * Rotationd::FromRotationVector(_angVel * dtInt);

    double discreteTimeConstAngVel = exp(-dtInt / _timeConstantTrackAngVel);

    if (cmd.ballistic) {
      discreteTimeConstAngVel = 1;  //ignore cmd
    }
    Vec3d newAngVel = discreteTimeConstAngVel * est.angVel
        + (1 - discreteTimeConstAngVel) * cmd.angVel;

    est.pos = newPos;
    est.vel = newVel;
    est.att = newAtt;
    est.angVel = newAngVel;

    t += dtInt;
  }

  return est;
}

void MocapStateEstimator::UpdateWithMeasurement(Vec3d const measPos,
                                                Rotationd const measAtt) {
  std::lock_guard<std::mutex> guard(_mutexEstimate);
  if (!_initialized) {
    _initialized = true;
    _pos = measPos;
    _vel = Vec3d(0, 0, 0);
    _att = measAtt;
    _lastMeasAtt = measAtt;
    _angVel = Vec3d(0, 0, 0);
    _lastGoodMeasUpdate.Reset();
    ResetVariance();
    return;
  }

  double const t0 = _estimateTimer.GetSeconds<double>();
  double const tEnd = _timer.GetSeconds<double>();  //measurement is true *now*
  if (tEnd > t0) {
    for (;;) {
      double tNow = _estimateTimer.GetSeconds<double>();
      if ((tNow + SMALL_TIME) >= tEnd) {
        break;
      }
      double predictionTime = 0;
      PredictionType p;
      if (!_predictionPipe.GetActiveMessage(_estimateTimer.GetSeconds<double>(),
                                            p, predictionTime)) {
        //no messages
        p.acc = Vec3d(0, 0, 0);
        p.angVel = Vec3d(0, 0, 0);
        p.ballistic = true;
        predictionTime = 1e10;
      }

      //make sure the prediction is valid for the whole period:
      double dtInt = tEnd - _estimateTimer.GetSeconds<double>();
      if (dtInt > (predictionTime + SMALL_TIME)) {  //add some margin to ensure no shenanigans due to float precision
        dtInt = predictionTime;
      }

      Vec3d const pos(_pos);
      Vec3d const vel(_vel);
      Rotationd const att(_att);
      Vec3d const angVel(_angVel);

      _pos = pos + vel * dtInt;
      _vel = vel + p.acc * dtInt;
      _att = att * Rotationd::FromRotationVector(angVel * dtInt);
      double discreteTimeConstAngVel = exp(-dtInt / _timeConstantTrackAngVel);
      if (p.ballistic) {
        discreteTimeConstAngVel = 1;
      }
      _angVel = discreteTimeConstAngVel * angVel
          + (1 - discreteTimeConstAngVel) * p.angVel;

      _estimateTimer.AdvanceMicroSeconds(uint64_t(0.5 + dtInt * 1e6));

      //update the variance
      Eigen::Matrix<double, 2, 2> Apos, Qpos;
      Apos << 1, dtInt, 0, 1;
      Qpos << dtInt * dtInt * dtInt * dtInt * _procNoiseStdDevPos / 4, 0, 0, dtInt
          * dtInt * _procNoiseStdDevPos;
      Eigen::Matrix<double, 2, 2> Aatt, Qatt;
      Aatt << 1, dtInt, 0, 1;  //this should really have the last value be == discreteTimeConstAngVel, but that leads to poor performance
      Qatt << dtInt * dtInt * dtInt * dtInt * _procNoiseStdDevAtt / 4, 0, 0, dtInt
          * dtInt * _procNoiseStdDevAtt;

      Eigen::Matrix<double, 2, 2> newPosVar = Apos * _variancePosition
          * Apos.transpose() + Qpos;
      Eigen::Matrix<double, 2, 2> newAttVar = Aatt * _varianceAttitude
          * Aatt.transpose() + Qatt;

      _variancePosition = newPosVar;
      _varianceAttitude = newAttVar;
    }
  }

  //decide whether to reject the measurements:

  double innovationCovPos = _variancePosition(0, 0)
      + _measNoiseStdDevPos * _measNoiseStdDevPos;  //scalar measurement, so easy
  double innovationCovAtt = _varianceAttitude(0, 0)
      + _measNoiseStdDevAtt * _measNoiseStdDevAtt;  //scalar measurement, so easy

  double distMeasPos = (measPos - _pos).GetNorm2() / sqrt(3 * innovationCovPos);  //3 times because it's in 3D
  double distMeasAtt = (measAtt.Inverse() * _att).GetAngle()
      / sqrt(innovationCovAtt);

  bool shouldRejectMeas = false;
  if ((distMeasPos > _measRejectDist) or (distMeasAtt > _measRejectDist)) {
    shouldRejectMeas = true;
  }

  if (shouldRejectMeas
      && _numMeasRejectedConsecutively < MAX_NUM_CONSECUTIVE_REJECTION) {
    _numMeasRejected++;
    _numMeasRejectedConsecutively++;
  } else {
    if (_numMeasRejectedConsecutively >= MAX_NUM_CONSECUTIVE_REJECTION) {
      //we're force-accpeting this, so we'll reset the variance.
      printf("ESTIMATOR RESET!\n");
      Reset();
      innovationCovPos = _variancePosition(0, 0)
          + _measNoiseStdDevPos * _measNoiseStdDevPos;  //scalar measurement, so easy
      innovationCovAtt = _varianceAttitude(0, 0)
          + _measNoiseStdDevAtt * _measNoiseStdDevAtt;  //scalar measurement, so easy
    }
    _numMeasRejectedConsecutively = 0;
    _lastGoodMeasUpdate.Reset();
    //Compute KF gain:
    Eigen::Matrix<double, 1, 2> H;
    H << 1, 0;
    auto gainPos = _variancePosition * H.transpose() * (1 / innovationCovPos);
    auto gainAtt = _varianceAttitude * H.transpose() * (1 / innovationCovAtt);

    //update position
    Vec3d measErrPos = measPos - _pos;
    _pos += gainPos(0, 0) * measErrPos;
    _vel += gainPos(1, 0) * measErrPos;

    //update attitude (is slightly more delicate)
    Vec3d measErrAtt = (_att.Inverse() * measAtt).ToRotationVector();
    _att = _att * Rotationd::FromRotationVector(gainAtt(0, 0) * measErrAtt);  // * _att;
    _angVel += gainAtt(1, 0) * measErrAtt;

    auto newPosVar = (Eigen::Matrix<double, 2, 2>::Identity() - gainPos * H)
        * _variancePosition;
    auto newAttVar = (Eigen::Matrix<double, 2, 2>::Identity() - gainAtt * H)
        * _varianceAttitude;

    _variancePosition = newPosVar;
    _varianceAttitude = newAttVar;
  }

  //ensure symmetry:
  auto newPosVar = (_variancePosition + _variancePosition.transpose()) * 0.5;
  auto newAttVar = (_varianceAttitude + _varianceAttitude.transpose()) * 0.5;

  _variancePosition = newPosVar;
  _varianceAttitude = newAttVar;

  //clear expired messages
  _predictionPipe.ClearExpiredMessages(_estimateTimer.GetSeconds<double>());

}

