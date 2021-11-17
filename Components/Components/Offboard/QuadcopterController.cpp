#include "QuadcopterController.hpp"

using namespace Offboard;

QuadcopterController::QuadcopterController() {
  _minVerticalProperAcceleration = 0.5 * 9.81;  //we can fall at ~ 0.5*gravity
  _maxProperAcc = 20;
  _minProperAcc = -1;
}

void QuadcopterController::Run(Vec3d const curPos, Vec3d const curVel,
                               Rotationd const curAtt, Vec3d const desPos,
                               Vec3d const desVel, Vec3d const desAcc,
                               double const desiredYawAngle,
                               Vec3d &outCmdAngVel,
                               double &outCmdThrust) const {
  Vec3f const e3(0, 0, 1);

  //Run the controller
  Vec3f const cmdAcc = _posCtrl.GetDesAcceleration(Vec3f(curPos), Vec3f(curVel),
                                                   Vec3f(desPos), Vec3f(desVel),
                                                   Vec3f(desAcc));

  Vec3f cmdProperAcc = cmdAcc + Vec3f(0, 0, 9.81f);

  //saturate thrust
  if (cmdProperAcc.GetNorm2() > _maxProperAcc) {
    cmdProperAcc *= _maxProperAcc / cmdProperAcc.GetNorm2();
  }

  //apply the max tilt angle:
  if (cmdProperAcc.z < _minVerticalProperAcceleration) {
    cmdProperAcc.z = _minVerticalProperAcceleration;
  }

  //Compute total thrust, thrust direction:
  float const normCmdProperAcc = cmdProperAcc.GetNorm2();
  Vec3f const cmdThrustDir = cmdProperAcc / normCmdProperAcc;

  //Make sure we produce the required z-acceleration (up to some maximum)
  outCmdThrust = normCmdProperAcc
      * (Rotationf(curAtt) * Vec3f(0, 0, 1)).Dot(cmdThrustDir);

  if (outCmdThrust < _minProperAcc) {
    outCmdThrust = _minProperAcc;
  }

  //Construct the attitude that would match this desired thrust direction
  Rotationf cmdAtt;

  const float cosAngle = cmdThrustDir.Dot(e3);
  float angle;
  if (cosAngle >= (1 - 1e-12f)) {
    angle = 0;
  } else if (cosAngle <= -(1 - 1e-12f)) {
    angle = float(M_PI);
  } else {
    angle = acosf(cosAngle);  //magnitude of desired rotation
  }
  Vec3f rotAx = e3.Cross(cmdThrustDir);
  const float n = rotAx.GetNorm2();
  if (n < 1e-6f) {
    cmdAtt = Rotationf::Identity();
  } else {
    cmdAtt = Rotationf::FromRotationVector(rotAx * (angle / n));
  }

  //yaw angle
  Rotationf cmdAttYawed = cmdAtt
      * Rotationf::FromRotationVector(Vec3f(0, 0, desiredYawAngle));

  outCmdAngVel = Vec3d(
      _attCtr.GetDesiredAngularVelocity(cmdAttYawed, Rotationf(curAtt)));
}

void QuadcopterController::RunTracking(Vec3d const curPos, Vec3d const curVel,
                                       Rotationd const curAtt,
                                       Vec3d const refPos, Vec3d const refVel,
                                       Vec3d const refAcc,
                                       double const desiredYawAngle,
                                       double const refThrust,
                                       Vec3d const refAngVel,
                                       Vec3d &outCmdAngVel,
                                       double &outCmdThrust,
                                       Rotationf &outCmdAtt) const {

  // Get the normalized thrust error
  // We set the reference acceleration to zero because the position controller only uses it as a feedforward term
  // e.g. we don't calculate an acceleration error (although we probably could if we used IMU data, but right now we don't)

  Vec3f const accErr = _posCtrl.GetDesAcceleration(Vec3f(curPos), Vec3f(curVel),
                                                   Vec3f(refPos), Vec3f(refVel),
                                                   Vec3f(0.0, 0.0, 0.0));

  outCmdThrust = refThrust + accErr.Dot(Rotationf(curAtt) * Vec3f(0, 0, 1));

  //Compute total thrust, thrust direction:
  float const normRefProperAcc =
      (refAcc + accErr + Vec3f(0, 0, 9.81f)).GetNorm2();
  Vec3f const refThrustDir = (refAcc + accErr + Vec3f(0, 0, 9.81f))
      / normRefProperAcc;

  //Construct the attitude that would match this desired thrust direction
  Rotationf refAtt;

  Vec3f const e3(0, 0, 1);
  const float cosAngle = refThrustDir.Dot(e3);
  float angle;
  if (cosAngle >= (1 - 1e-12f)) {
    angle = 0;
  } else if (cosAngle <= -(1 - 1e-12f)) {
    angle = float(M_PI);
  } else {
    angle = acosf(cosAngle);  //magnitude of desired rotation
  }
  Vec3f rotAx = e3.Cross(refThrustDir);
  const float n = rotAx.GetNorm2();
  if (n < 1e-6f) {
    refAtt = Rotationf::Identity();
  } else {
    refAtt = Rotationf::FromRotationVector(rotAx * (angle / n));
  }

  //yaw angle
  Rotationf refAttYawed = refAtt
      * Rotationf::FromRotationVector(Vec3f(0, 0, desiredYawAngle));
  outCmdAtt = refAttYawed;
  Vec3f const angVelErr = Vec3d(
      _attCtr.GetDesiredAngularVelocity(refAttYawed, Rotationf(curAtt)));
  outCmdAngVel = refAngVel + angVelErr;
}

