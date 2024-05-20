#include "GPSIMUStateEstimator.hpp"
#include <iostream>

using namespace Offboard; //TODO: this is a misnomer. We are using some "offboard" tools in the code here. Should be renamed to something more generic.

unsigned const MAX_NUM_CONSECUTIVE_REJECTION = 10; 
double const SMALL_TIME = 1e-6;  // [s]

GPSIMUStateEstimator::GPSIMUStateEstimator(BaseTimer* const masterTimer, unsigned const id)
    : _id(id),
      _estimateTimer(masterTimer),
      _lastGoodMeasUpdate(masterTimer),
      _initialized(false) {

  _numResets = 0;
  _measRejectDist = 6.0;
  _timeConstantTrackAngVel = 0.04;  //TODO!
  _systemLatency = 0.0;

  //TODO: Sensible numbers here!
  _initStdDevPos = 3.0;  //[m]
  _initStdDevVel = 3.0;  //[m/s]
  _initStdDevAtt = 10.0 * double(M_PI) / 180.0;  //[rad]

  _measNoiseStdDevAccelerometer = 5;  //[m/s**2]
  _measNoiseStdDevRateGyro = 0.1;  //[rad/s]
  _measNoiseStdDevPos = 0.25;  //[m]

  Reset();
}

void GPSIMUStateEstimator::Reset() {
  _numResets ++;
  _initialized = false;
  _pos = Vec3d(0, 0, 0);
  _vel = Vec3d(0, 0, 0);
  _att = Rotationd::Identity();
  _lastMeasAtt = Rotationd::Identity();
  _angVel = Vec3d(0, 0, 0);
  //estimate is valid *now*:
  _estimateTimer.Reset();
  _lastGoodMeasUpdate.Reset();
  ResetVariance();
}

void GPSIMUStateEstimator::ResetVariance() {
  _cov = ZeroMatrix<double, NUM_STATES, NUM_STATES>();
  for (int i = 0; i < 3; i++) {
    _cov(I_POS + i, I_POS + i) = _initStdDevPos * _initStdDevPos;
    _cov(I_VEL + i, I_VEL + i) = _initStdDevVel * _initStdDevVel;
    _cov(I_ATT + i, I_ATT + i) = _initStdDevAtt
      * _initStdDevAtt;
  }
}

EstimatedState GPSIMUStateEstimator::GetCurrentEstimate() {
  std::lock_guard<std::mutex> guard(_mutexEstimate);
  EstimatedState est;
  est.pos = _pos;
  est.vel = _vel;
  est.att = _att;
  est.angVel = _angVel;
  return est;
}

void GPSIMUStateEstimator::Predict(Vec3d const measAcc, Vec3d const measGyro) {
  if (!_initialized) {
    Reset();
    _initialized = true;
    _estimateTimer.Reset();

    /*Assume we're measuring gravity; construct a consistent initial attitude:
     *
     * TODO: note, this does not correctly initialise the attitude covariance
     * necessarily: we want large uncertainty about gravity
     */
    errno = 0;

    Vec3d const expAccelerometer = _att.Inverse() * Vec3d(0, 0, 1);
    Vec3d const accUnitVec = measAcc.GetUnitVector();
    double const cosAngleError = expAccelerometer.Dot(accUnitVec);

    Vec3d rotAx = (accUnitVec.Cross(expAccelerometer));
    if (rotAx.GetNorm2() > 1e-6f) {
      rotAx = rotAx / rotAx.GetNorm2();
    } else {
      rotAx = Vec3d(1, 0, 0);  //somewhat arbitrary
    }

    double angle = acosf(cosAngleError);
    if (errno) {
      //acos failed:
      if (cosAngleError < 0) {
        angle = double(M_PI);
      } else {
        angle = 0;
      }
    }

    _att = _att * Rotationd::FromAxisAngle(rotAx, angle);

    return;
  }


  //time since last run:
  double const dt = _estimateTimer.GetSeconds<double>();

  
  _estimateTimer.Reset();

  //mean prediction: //we're assuming that dt**2 ~= 0
  Vec3d const pos(_pos);
  Vec3d const vel(_vel);
  Rotationd const att(_att);

  Vec3d const acc = _att * measAcc + Vec3d(0, 0, -9.81f);
  _pos = pos + vel * dt;
  _vel = vel + acc * dt;
  _att = att * Rotationd::FromRotationVector(measGyro * dt);  //NOTE: we're using the rate gyro to integrate, not old estimate
  _angVel = measGyro;

  //Update the covariance matrix
  double rotMat[9];
  att.GetRotationMatrix(rotMat);
  SquareMatrix<double, NUM_STATES> f = ZeroMatrix<double, NUM_STATES, NUM_STATES>();

  //del(d(pos))/del(pos) = I
  f(I_POS + 0, I_POS + 0) = 1;
  f(I_POS + 1, I_POS + 1) = 1;
  f(I_POS + 2, I_POS + 2) = 1;

  //del(d(pos))/del(vel) = I*dt
  f(I_POS + 0, I_VEL + 0) = dt;
  f(I_POS + 1, I_VEL + 1) = dt;
  f(I_POS + 2, I_VEL + 2) = dt;

  //del(d(vel))/del(vel) = I
  f(I_VEL + 0, I_VEL + 0) = 1;
  f(I_VEL + 1, I_VEL + 1) = 1;
  f(I_VEL + 2, I_VEL + 2) = 1;

  //del(d(vel))/del(att)
  //dt*(accMeas(1)*Rref(0, 2) - accMeas(2)*Rref(0, 1))
  //dt*(accMeas(1)*Rref(1, 2) - accMeas(2)*Rref(1, 1))
  //dt*(accMeas(1)*Rref(2, 2) - accMeas(2)*Rref(2, 1))
  f(I_VEL + 0, I_ATT + 0) = dt
      * (+measAcc.y * rotMat[3 * 0 + 2] - measAcc.z * rotMat[3 * 0 + 1]);
  f(I_VEL + 1, I_ATT + 0) = dt
      * (+measAcc.y * rotMat[3 * 1 + 2] - measAcc.z * rotMat[3 * 1 + 1]);
  f(I_VEL + 2, I_ATT + 0) = dt
      * (+measAcc.y * rotMat[3 * 2 + 2] - measAcc.z * rotMat[3 * 2 + 1]);

  //dt*(-accMeas(0)*Rref(0, 2) + accMeas(2)*Rref(0, 0))
  //dt*(-accMeas(0)*Rref(1, 2) + accMeas(2)*Rref(1, 0))
  //dt*(-accMeas(0)*Rref(2, 2) + accMeas(2)*Rref(2, 0))
  f(I_VEL + 0, I_ATT + 1) = dt
      * (-measAcc.x * rotMat[3 * 0 + 2] + measAcc.z * rotMat[3 * 0 + 0]);
  f(I_VEL + 1, I_ATT + 1) = dt
      * (-measAcc.x * rotMat[3 * 1 + 2] + measAcc.z * rotMat[3 * 1 + 0]);
  f(I_VEL + 2, I_ATT + 1) = dt
      * (-measAcc.x * rotMat[3 * 2 + 2] + measAcc.z * rotMat[3 * 2 + 0]);

  //dt*(accMeas(0)*Rref(0, 1) - accMeas(1)*Rref(0, 0)),
  //dt*(accMeas(0)*Rref(1, 1) - accMeas(1)*Rref(1, 0)),
  //dt*(accMeas(0)*Rref(2, 1) - accMeas(1)*Rref(2, 0)),
  f(I_VEL + 0, I_ATT + 2) = dt
      * (+measAcc.x * rotMat[3 * 0 + 1] - measAcc.y * rotMat[3 * 0 + 0]);
  f(I_VEL + 1, I_ATT + 2) = dt
      * (+measAcc.x * rotMat[3 * 1 + 1] - measAcc.y * rotMat[3 * 1 + 0]);
  f(I_VEL + 2, I_ATT + 2) = dt
      * (+measAcc.x * rotMat[3 * 2 + 1] - measAcc.y * rotMat[3 * 2 + 0]);

  //del(d(att))/del(att)
  f(I_ATT + 0, I_ATT + 0) = 1;
  f(I_ATT + 1, I_ATT + 0) = -(dt * measGyro.z
      + _lastMeasUpdateAttCorrection.z / 2.0f);
  f(I_ATT + 2, I_ATT + 0) = +(dt * measGyro.y
      + _lastMeasUpdateAttCorrection.y / 2.0f);

  f(I_ATT + 0, I_ATT + 1) = +(dt * measGyro.z
      + _lastMeasUpdateAttCorrection.z / 2.0f);
  f(I_ATT + 1, I_ATT + 1) = 1;
  f(I_ATT + 2, I_ATT + 1) = -(dt * measGyro.x
      + _lastMeasUpdateAttCorrection.x / 2.0f);

  f(I_ATT + 0, I_ATT + 2) = -(dt * measGyro.y
      + _lastMeasUpdateAttCorrection.y / 2.0f);
  f(I_ATT + 1, I_ATT + 2) = +(dt * measGyro.x
      + _lastMeasUpdateAttCorrection.x / 2.0f);
  f(I_ATT + 2, I_ATT + 2) = 1;
  _lastMeasUpdateAttCorrection = Vec3d(0, 0, 0);

  //Covariance prediction:
  _cov = f * _cov * f.transpose();
  //add process noise:
  for (int i = 0; i < 3; i++) {
    _cov(I_VEL + i, I_VEL + i) += _measNoiseStdDevAccelerometer
        * _measNoiseStdDevAccelerometer * dt * dt;
    _cov(I_ATT + i, I_ATT + i) += _measNoiseStdDevRateGyro
        * _measNoiseStdDevRateGyro * dt * dt;
  }
}


void GPSIMUStateEstimator::UpdateWithMeasurement(Vec3d const measPos) {
  std::lock_guard<std::mutex> guard(_mutexEstimate);
  if (!_initialized) {
    _initialized = true;
    _pos = measPos;
    _vel = Vec3d(0, 0, 0);
    _att = Rotationd::Identity();
    _lastMeasAtt = Rotationd::Identity();
    _angVel = Vec3d(0, 0, 0);
    ResetVariance();
    return;
  }

	Matrix<double, 3, 9> H = ZeroMatrix<double, 3, 9>();
	H(0, 0) = 1.0;
	H(1, 1) = 1.0;
	H(2, 2) = 1.0;

	SquareMatrix<double, 3> innovationCov = H * (_cov * H.transpose()) + _measNoiseStdDevPos * _measNoiseStdDevPos * IdentityMatrix<double, 3>();

  // Check if the determinant is close to zero or if the matrix contains NaN values
  double det = innovationCov.determinant();
  if (std::abs(det) < 1e-10 || !innovationCov.allFinite()) {
      std::cerr << "Matrix inversion not possible due to singularity or NaN values." << std::endl;
      _pos = measPos;
      _vel = Vec3d(0, 0, 0);
      _att = Rotationd::Identity();
      _lastMeasAtt = Rotationd::Identity();
      _angVel = Vec3d(0, 0, 0);
      ResetVariance();
      return;
  }

  SquareMatrix<double, 3> innovationCovInv = MatrixInverse<double, 3>(innovationCov);
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
  _lastGoodMeasUpdate.Reset();
}