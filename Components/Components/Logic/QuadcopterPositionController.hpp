#pragma once

#include "Common/Math/Vec3.hpp"

namespace Onboard {

//The controller that tracks velocity & position, outputs desired acceleration vector.
class QuadcopterPositionController {
 public:
  QuadcopterPositionController()
      : _natFreq(0),
        _dampingRatio(0) {
    //nothing here
  }

  void SetParameters(float natFreq, float dampingRatio) {
    _natFreq = natFreq;
    _dampingRatio = dampingRatio;
  }

  //full position feedback, optional feed-forwards
  inline Vec3f GetDesAcceleration(Vec3f estPos, Vec3f estVel, Vec3f desPos,
                                  Vec3f desVel = Vec3f(0, 0, 0), Vec3f desAcc =
                                      Vec3f(0, 0, 0)) const {
    return (desPos - estPos) * _natFreq * _natFreq
        + (desVel - estVel) * 2 * _natFreq * _dampingRatio + desAcc;
  }

  float GetNaturalFrequency() const {
    return _natFreq;
  }

  float GetDamping() const {
    return _dampingRatio;
  }

  /*
   //Only velocity control
   inline Vec3f GetDesAccelerationVelCtrl(Vec3f estVel, Vec3f desVel = Vec3f(0,0,0), Vec3f desAcc = Vec3f(0,0,0)) const
   {
   return GetDesAcceleration(Vec3f(0,0,0), estVel, Vec3f(0,0,0), desVel, desAcc);
   }
   */

 private:
  float _natFreq;  //closed-loop natural frequency [rad/s]
  float _dampingRatio;  //closed-loop damping ratio [1]
};

}

//namespace Onboard
