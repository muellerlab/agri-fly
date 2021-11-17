#include "KalmanFilter6DOF.hpp"

#include <cerrno>

using namespace Onboard;

float const TIME_CONST_ATT_CORR = 4.0f;  //[s]

KalmanFilter6DOF::KalmanFilter6DOF(BaseTimer* const masterTimer)
    : _estimateTimer(masterTimer),
      _timerLastGoodMeasUpdate(masterTimer),
      _IMUInitialized(false),
      _UWBInitialized(false) {

  //TODO: Sensible numbers here!
  _initStdDevPos = 3.0f;  //[m]
  _initStdDevVel = 3.0f;  //[m/s]
  _initStdDevAtt_perpToGravity = 10.0f * float(M_PI) / 180.0f;  //[rad]
  _initStdDevAtt_aboutGravity = 30.0f * float(M_PI) / 180.0f;  //[rad]

  _measNoiseStdDevAccelerometer = 5;  //[m/s**2]
  _measNoiseStdDevRateGyro = 0.1f;  //[rad/s]
  _measNoiseStdDevRangeMeasurements = 0.14f;  //[m]

  _outlierDetectionStatisticalDist = 3.0f;
  _numMeasRejected = 0;
  _numMeasRejectedSequentially = 0;
  _maxNumMeasRejectedSequentially = 5;  // TODO: add a set function for this
  _numResets = 0;
  _lastCheckNumResets = 0;
}

void KalmanFilter6DOF::Reset() {
  _numResets++;
  _IMUInitialized = _UWBInitialized = false;
  _pos = Vec3f(0, 0, 0);
  _vel = Vec3f(0, 0, 0);
  _att = Rotationf::Identity();
  _angVel = Vec3f(0, 0, 0);

  //Default initial covariance, should make a paramter (or something!)
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_STATES; j++) {
      _cov(i, j) = 0;
    }
  }

  for (int i = 0; i < 3; i++) {
    _cov(I_POS + i, I_POS + i) = _initStdDevPos * _initStdDevPos;
    _cov(I_VEL + i, I_VEL + i) = _initStdDevVel * _initStdDevVel;
  }
  //TODO FIXME. This is a hack. We want to encode that we're less certain of orientation
  // about gravity than other two directions. Below is OK, because _att is identity, but
  // this isn't future-proof. Nicer would be a geometric construction, using current estimate
  // of gravity.
  _cov(I_ATT + 0, I_ATT + 0) = _initStdDevAtt_perpToGravity
      * _initStdDevAtt_perpToGravity;
  _cov(I_ATT + 1, I_ATT + 1) = _initStdDevAtt_perpToGravity
      * _initStdDevAtt_perpToGravity;
  _cov(I_ATT + 2, I_ATT + 2) = _initStdDevAtt_aboutGravity
      * _initStdDevAtt_aboutGravity;

  //estimate is valid *now*:
  _estimateTimer.Reset();
  _timerLastGoodMeasUpdate.Reset();

  _lastMeasUpdateAttCorrection = Vec3f(0, 0, 0);
}

void KalmanFilter6DOF::Predict(Vec3f const measGyro, Vec3f const measAcc) {
  if (!_IMUInitialized) {
    Reset();
    _IMUInitialized = true;
    _estimateTimer.Reset();

    /*Assume we're measuring gravity; construct a consistent initial attitude:
     *
     * TODO: note, this does not correctly initialise the attitude covariance
     * necessarily: we want large uncertainty about gravity
     */
    errno = 0;

    Vec3f const expAccelerometer = _att.Inverse() * Vec3f(0, 0, 1);
    Vec3f const accUnitVec = measAcc.GetUnitVector();

    float const cosAngleError = expAccelerometer.Dot(accUnitVec);

    Vec3f rotAx = (accUnitVec.Cross(expAccelerometer));
    if (rotAx.GetNorm2() > 1e-6f) {
      rotAx = rotAx / rotAx.GetNorm2();
    } else {
      rotAx = Vec3f(1, 0, 0);  //somewhat arbitrary
    }

    float angle = acosf(cosAngleError);
    if (errno) {
      //acos failed:
      if (cosAngleError < 0) {
        angle = float(M_PI);
      } else {
        angle = 0;
      }
    }

    _att = _att * Rotationf::FromAxisAngle(rotAx, angle);

    return;
  }

  //time since last run:
  float const dt = _estimateTimer.GetSeconds<float>();
  _estimateTimer.Reset();

  if (!_UWBInitialized) {
    _angVel = measGyro;

    //do a low-pass ("complementary") estimate of attitude, using gyro & accelerometer
    Rotationf newAtt = _att * Rotationf::FromRotationVector(measGyro * dt);
    _att = newAtt;

    Vec3f const expAccelerometer = _att.Inverse() * Vec3f(0, 0, 1);
    Vec3f const accUnitVec = measAcc.GetUnitVector();

    Vec3f rotAx = (accUnitVec.Cross(expAccelerometer));
    if (rotAx.GetNorm2() > 1e-6f) {
      rotAx = rotAx / rotAx.GetNorm2();
    } else {
      rotAx = Vec3f(1, 0, 0);  //somewhat arbitrary
    }

    float const cosAngleError = expAccelerometer.Dot(accUnitVec);
    errno = 0;
    float angle = acosf(cosAngleError);
    if (errno) {
      //acos failed, or sqrtf:
      if (cosAngleError < 0) {
        angle = float(M_PI);
      } else {
        angle = 0;
      }
    }

    float const corrAngle = (dt / TIME_CONST_ATT_CORR) * angle;
    _att = _att * Rotationf::FromAxisAngle(rotAx, corrAngle);

    return;
  }

  //mean prediction: //we're assuming that dt**2 ~= 0
  Vec3f const pos(_pos);
  Vec3f const vel(_vel);
  Rotationf const att(_att);
  // Vec3f const angVel(_angVel);

  Vec3f const acc = _att * measAcc + Vec3f(0, 0, -9.81f);
  _pos = pos + vel * dt;
  _vel = vel + acc * dt;
  _att = att * Rotationf::FromRotationVector(measGyro * dt);  //NOTE: we're using the rate gyro to integrate, not old estimate
  _angVel = measGyro;

  float rotMat[9];
  att.GetRotationMatrix(rotMat);
  SquareMatrix<float, NUM_STATES> f = ZeroMatrix<float, NUM_STATES, NUM_STATES>();

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
  _lastMeasUpdateAttCorrection = Vec3f(0, 0, 0);

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

void KalmanFilter6DOF::UpdateWithRangeMeasurement(Vec3f const targetPosition,
                                                  float const range) {
  if (!_IMUInitialized) {
    return;
  }

  //Test for NaN in range:
  if (not (range == range)) {
    return;
  }

  _UWBInitialized = true;

  float const expectedRange = (_pos - targetPosition).GetNorm2();
  Vec3f const targetDirection = (_pos - targetPosition) / expectedRange;
  //Measurement matrix:
  Matrix<float, 1, NUM_STATES> H;
  for (int i = 0; i < 3; i++) {
    H(0, I_POS + i) = targetDirection[i];
    H(0, I_VEL + i) = 0;
    H(0, I_ATT + i) = 0;
  }

  //filter gain:
  float const innovationCov = (H * (_cov * H.transpose()))(0, 0)
      + _measNoiseStdDevRangeMeasurements * _measNoiseStdDevRangeMeasurements;  //scalar measurement, so easy
  Matrix<float, NUM_STATES, 1> L = _cov * H.transpose() * (1 / innovationCov);

  //can now do a Mahalanobis outlier detection:
  float const measDistSqr = (range - expectedRange) * (range - expectedRange)
      / innovationCov;  //scalar is easy
  if (measDistSqr
      > _outlierDetectionStatisticalDist * _outlierDetectionStatisticalDist) {
    //Reject this as an outlier
    _numMeasRejected++;
    _numMeasRejectedSequentially++;
    if (_numMeasRejectedSequentially >= _maxNumMeasRejectedSequentially) {
      //TODO: Is this the right action? Alternative is to force-accept the measurment. -- blame Saman.
      Reset();
    }
    return;
  }
  _numMeasRejectedSequentially = 0;

  //state update:
  Matrix<float, NUM_STATES, 1> dx = L * (range - expectedRange);
  _pos = _pos + Vec3f(dx(I_POS + 0, 0), dx(I_POS + 1, 0), dx(I_POS + 2, 0));
  _vel = _vel + Vec3f(dx(I_VEL + 0, 0), dx(I_VEL + 1, 0), dx(I_VEL + 2, 0));
  _lastMeasUpdateAttCorrection = Vec3f(dx(I_ATT + 0, 0), dx(I_ATT + 1, 0),
                                       dx(I_ATT + 2, 0));
  _att = _att * Rotationf::FromRotationVector(_lastMeasUpdateAttCorrection);

  //covariance update:
  _cov = (IdentityMatrix<float, NUM_STATES>() - L * H) * _cov;

  MakeCovarianceSymmetric();

  _timerLastGoodMeasUpdate.Reset();
}

void KalmanFilter6DOF::MakeCovarianceSymmetric() {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = i + 1; j < NUM_STATES; j++) {
      _cov(i, j) = _cov(j, i);
    }
  }
}
