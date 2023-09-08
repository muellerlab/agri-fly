#include "GPSStateEstimator.hpp"
#include <iostream>

using namespace Offboard;

unsigned const MAX_NUM_CONSECUTIVE_REJECTION = 10;  //TODO: make this a parameter
double const SMALL_TIME = 1e-6;  // [s]

GPSStateEstimator::GPSStateEstimator(BaseTimer* const masterTimer,
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


  _initStdDevPos = 0.0;  //[m]
  _initStdDevVel = 0.0;  //[m/s]
  _lastMeasUpdateAttCorrection = Vec3d(0, 0, 0);
  _initStdDevAtt = 5.0 * M_PI / 180.0;  //[rad]

  //rough guesses!
  _measNoiseStdDevPos = 0.02;  //[m]
  _procNoiseStdDevAcc = 1.06f;  //[m/s**2]  
  _procNoiseStdDevAngVel = 0.1f;  //[rad/s]
  Reset();
}

void GPSStateEstimator::Reset() {
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

void GPSStateEstimator::ResetVariance() {
  for (int i = 0; i < 3; i++) {
    _cov(I_POS + i, I_POS + i) = _initStdDevPos * _initStdDevPos;
    _cov(I_VEL + i, I_VEL + i) = _initStdDevVel * _initStdDevVel;
    _cov(I_ATT + i, I_ATT + i) = _initStdDevAtt
      * _initStdDevAtt;
  }
}

EstimatedState GPSStateEstimator::GetPrediction(
    double const dt) const {
  //tools for feedforward estimate state prediction
  double tStart;
  double tEnd = dt + _timer.GetSeconds<double>();
  EstimatedState est;
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

void GPSStateEstimator::UpdateWithMeasurement(Vec3d const measPos) {
  std::lock_guard<std::mutex> guard(_mutexEstimate);
  if (!_initialized) {
    _initialized = true;
    _pos = measPos;
    _vel = Vec3d(0, 0, 0);
    _att = Rotationd::Identity();
    _lastMeasAtt = _att;
    _angVel = Vec3d(0, 0, 0);
    _lastGoodMeasUpdate.Reset();
    ResetVariance();
    return;
  }

  double const t0 = _estimateTimer.GetSeconds<double>();
  double const tEnd = _timer.GetSeconds<double>();  //measurement is true *now*

  // Carry out the feadforward step: clear out the prediction pipe
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

      // Compute the nominal acc in vehicle body frame (i.e. what the accelerometer is using)
      Vec3d nomAcc = _att.Inverse() * (p.acc + Vec3d(0, 0, 9.81));

      // Update the variance. Reference: “Covariance correction step for Kalman filtering with an attitude”
      double rotMat[9];
      _att.GetRotationMatrix(rotMat);
      SquareMatrix<double, NUM_STATES> f = ZeroMatrix<double, NUM_STATES, NUM_STATES>();

      //del(d(pos))/del(pos) = I
      f(I_POS + 0, I_POS + 0) = 1;
      f(I_POS + 1, I_POS + 1) = 1;
      f(I_POS + 2, I_POS + 2) = 1;

      //del(d(pos))/del(vel) = I*dt
      f(I_POS + 0, I_VEL + 0) = dtInt;
      f(I_POS + 1, I_VEL + 1) = dtInt;
      f(I_POS + 2, I_VEL + 2) = dtInt;

      //del(d(vel))/del(vel) = I
      f(I_VEL + 0, I_VEL + 0) = 1;
      f(I_VEL + 1, I_VEL + 1) = 1;
      f(I_VEL + 2, I_VEL + 2) = 1;

      //del(d(vel))/del(att)
      //dt*(accMeas(1)*Rref(0, 2) - accMeas(2)*Rref(0, 1))
      //dt*(accMeas(1)*Rref(1, 2) - accMeas(2)*Rref(1, 1))
      //dt*(accMeas(1)*Rref(2, 2) - accMeas(2)*Rref(2, 1))
      f(I_VEL + 0, I_ATT + 0) = dtInt
          * (+nomAcc.y * rotMat[3 * 0 + 2] - nomAcc.z * rotMat[3 * 0 + 1]);
      f(I_VEL + 1, I_ATT + 0) = dtInt
          * (+nomAcc.y * rotMat[3 * 1 + 2] - nomAcc.z * rotMat[3 * 1 + 1]);
      f(I_VEL + 2, I_ATT + 0) = dtInt
          * (+nomAcc.y * rotMat[3 * 2 + 2] - nomAcc.z * rotMat[3 * 2 + 1]);

      //dt*(-accMeas(0)*Rref(0, 2) + accMeas(2)*Rref(0, 0))
      //dt*(-accMeas(0)*Rref(1, 2) + accMeas(2)*Rref(1, 0))
      //dt*(-accMeas(0)*Rref(2, 2) + accMeas(2)*Rref(2, 0))
      f(I_VEL + 0, I_ATT + 1) = dtInt
          * (-nomAcc.x * rotMat[3 * 0 + 2] + nomAcc.z * rotMat[3 * 0 + 0]);
      f(I_VEL + 1, I_ATT + 1) = dtInt
          * (-nomAcc.x * rotMat[3 * 1 + 2] + nomAcc.z * rotMat[3 * 1 + 0]);
      f(I_VEL + 2, I_ATT + 1) = dtInt
          * (-nomAcc.x * rotMat[3 * 2 + 2] + nomAcc.z * rotMat[3 * 2 + 0]);

      //dt*(accMeas(0)*Rref(0, 1) - accMeas(1)*Rref(0, 0)),
      //dt*(accMeas(0)*Rref(1, 1) - accMeas(1)*Rref(1, 0)),
      //dt*(accMeas(0)*Rref(2, 1) - accMeas(1)*Rref(2, 0)),
      f(I_VEL + 0, I_ATT + 2) = dtInt
          * (+nomAcc.x * rotMat[3 * 0 + 1] - nomAcc.y * rotMat[3 * 0 + 0]);
      f(I_VEL + 1, I_ATT + 2) = dtInt
          * (+nomAcc.x * rotMat[3 * 1 + 1] - nomAcc.y * rotMat[3 * 1 + 0]);
      f(I_VEL + 2, I_ATT + 2) = dtInt
          * (+nomAcc.x * rotMat[3 * 2 + 1] - nomAcc.y * rotMat[3 * 2 + 0]);

      //del(d(att))/del(att)
      f(I_ATT + 0, I_ATT + 0) = 1;
      f(I_ATT + 1, I_ATT + 0) = -(dtInt * _angVel.z
          + _lastMeasUpdateAttCorrection.z / 2.0f);
      f(I_ATT + 2, I_ATT + 0) = +(dtInt * _angVel.y
          + _lastMeasUpdateAttCorrection.y / 2.0f);

      f(I_ATT + 0, I_ATT + 1) = +(dtInt * _angVel.z
          + _lastMeasUpdateAttCorrection.z / 2.0f);
      f(I_ATT + 1, I_ATT + 1) = 1;
      f(I_ATT + 2, I_ATT + 1) = -(dtInt * _angVel.x
          + _lastMeasUpdateAttCorrection.x / 2.0f);

      f(I_ATT + 0, I_ATT + 2) = -(dtInt * _angVel.y
          + _lastMeasUpdateAttCorrection.y / 2.0f);
      f(I_ATT + 1, I_ATT + 2) = +(dtInt * _angVel.x
          + _lastMeasUpdateAttCorrection.x / 2.0f);
      f(I_ATT + 2, I_ATT + 2) = 1;

      _lastMeasUpdateAttCorrection = Vec3d(0, 0, 0);
      //Covariance prediction:
      _cov = f * _cov * f.transpose();
      //add process noise:
      for (int i = 0; i < 3; i++) {
        _cov(I_VEL + i, I_VEL + i) += _procNoiseStdDevAcc
            * _procNoiseStdDevAcc * dtInt * dtInt;
        _cov(I_ATT + i, I_ATT + i) += _procNoiseStdDevAngVel
            * _procNoiseStdDevAngVel * dtInt * dtInt;
      }
    }
  }

	Matrix<double, 3, 9> H = ZeroMatrix<double, 3, 9>();
	H(0, 0) = 1.0f;
	H(1, 1) = 1.0f;
	H(2, 2) = 1.0f;

	SquareMatrix<double, 3> innovationCov = H * (_cov * H.transpose()) + _measNoiseStdDevPos*_measNoiseStdDevPos*IdentityMatrix<double, 3>();

  SquareMatrix<double, 3> innovationCovInv;
  try {
      innovationCovInv = MatrixInverse<double, 3>(innovationCov);
  } catch (const std::exception& e) {
      // Handle the inversion failure
      std::cerr << "Matrix inversion failed: " << e.what() << std::endl;
      _pos = measPos;
      return;
  }

	Matrix<double, 9, 3> L = _cov * H.transpose() * innovationCovInv;

	//convert _vel to matrix form
	Matrix<double, 3, 1> deltaPos;
	deltaPos(0,0) = measPos.x-_pos.x; deltaPos(1,0) = measPos.y-_pos.y; deltaPos(2,0) = measPos.z-_pos.z;
	Matrix<double, 9, 1> dx = L * (deltaPos);

	_pos = _pos + Vec3d(dx(I_POS + 0, 0), dx(I_POS + 1, 0), dx(I_POS + 2, 0));
	_vel = _vel + Vec3d(dx(I_VEL + 0, 0), dx(I_VEL + 1, 0), dx(I_VEL + 2, 0));
	_lastMeasUpdateAttCorrection = Vec3d(dx(I_ATT + 0, 0), dx(I_ATT + 1, 0),
			dx(I_ATT + 2, 0));
	_att = _att * Rotationd::FromRotationVector(_lastMeasUpdateAttCorrection);

	//covariance update:
	_cov = (IdentityMatrix<double, NUM_STATES>() - L * H) * _cov;
	_cov = 0.5 * (_cov + _cov.transpose());

  //clear expired messages
  _lastGoodMeasUpdate.Reset(); // Reset the timer. This is the last time we got a good measurement
  _predictionPipe.ClearExpiredMessages(_estimateTimer.GetSeconds<double>());
}