#pragma once

#include "Common/Math/Vec3.hpp"
#include "Common/Math/Matrix.hpp"

namespace Onboard {

//The controller that tracks angular velocities, and outputs desired torques
class QuadcopterAngularVelocityController {
 public:
  QuadcopterAngularVelocityController()
      : _timeConst_xy(0),
        _timeConst_z(0),
        _inertiaMatrix(ZeroMatrix<float, 3, 3>()) {
    //Note: initializing time constants to 0 forces the user to call SetParameters before calling GetDesiredTorques
  }

  void SetParameters(float timeConst_xy, float timeConst_z,
                     Matrix<float, 3, 3> inertiaMatrix) {
    _timeConst_xy = timeConst_xy;
    _timeConst_z = timeConst_z;
    _inertiaMatrix = inertiaMatrix;
  }

  Vec3f GetDesiredTorques(const Vec3f desiredAngVel, const Vec3f estAngVel) {

    Vec3f angVelError = desiredAngVel - estAngVel;
    Vec3f desAngAccel(angVelError.x/_timeConst_xy,
                      angVelError.y/_timeConst_xy,
                      angVelError.z/_timeConst_z);

    Vec3f nonlinCorrection = estAngVel.Cross(_inertiaMatrix*estAngVel);

    Vec3f desTorque = _inertiaMatrix*desAngAccel + nonlinCorrection;

    return desTorque;
  }

 private:

  float _timeConst_xy;  //time constant for roll & pitch rate [s]
  float _timeConst_z;	 //time constant for yaw rate [s]

  Matrix<float, 3, 3> _inertiaMatrix;

};

}

//namespace Onboard
