#pragma once

#include "Common/Math/Rotation.hpp"
#include "Common/Math/Vec3.hpp"

namespace Onboard {

//The controller that tracks attitude, and outputs desired angular velocity
class QuadcopterAttitudeController {
 public:
  QuadcopterAttitudeController()
      : _timeConst_xy(0),
        _timeConst_z(0) {
    //nothing here
  }
  void SetParameters(float timeConst_xy, float timeConst_z) {
    _timeConst_xy = timeConst_xy;
    _timeConst_z = timeConst_z;
    if(_timeConst_z < _timeConst_xy){
      // we explicitly disallow this.
      // Cannot have more aggressive yaw control than tilt control
      _timeConst_z = _timeConst_xy;
      assert(0);
    }
  }

  float GetTimeConst_xy() const {
    return _timeConst_xy;
  }

  float GetTimeConst_z() const {
    return _timeConst_z;
  }

  Vec3f GetDesiredAngularVelocity(const Rotationf desAttitude,
                                  const Rotationf estAttitude) const {
    Rotationf errAtt = (desAttitude.Inverse() * estAttitude);
    const Vec3f desRotVec = errAtt.ToRotationVector();

    Vec3f desRedAttRotAx = Vec3f(errAtt.Inverse() * Vec3f(0, 0, 1)).Cross(
        Vec3f(0, 0, 1));
    float desRedAttRotAn_cos = Vec3f(errAtt.Inverse() * Vec3f(0, 0, 1)).Dot(
        Vec3f(0, 0, 1));
    float desRedAttRotAn;
    if (desRedAttRotAn_cos >= 1.0f) {
      desRedAttRotAn = 0;
    } else if (desRedAttRotAn_cos <= -1.0f) {
      desRedAttRotAn = float(M_PI);
    } else {
      desRedAttRotAn = acosf(desRedAttRotAn_cos);
    }

    //normalize
    float n = desRedAttRotAx.GetNorm2();
    if (n < 1e-12f) {
      desRedAttRotAx = Vec3f(0, 0, 0);
    } else {
      desRedAttRotAx = desRedAttRotAx / n;
    }

    float k3 = (1.0f / _timeConst_z);
    float k12 = (1.0f / _timeConst_xy);

    Vec3f desAngVel = -k3 * desRotVec
        - (k12 - k3) * desRedAttRotAn * desRedAttRotAx;

    return desAngVel;
  }

 private:

  float _timeConst_xy;  //time constant for roll & pitch rate [s]
  float _timeConst_z;	 //time constant for yaw rate [s]

};

}
//namespace Onboard
