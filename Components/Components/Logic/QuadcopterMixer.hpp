#pragma once

namespace Onboard {

//Computes the forces necessary to achieve a given total force & torques.
/*Forces are assumed to be assigned as follows:
 *       x
 *       ^
 *  f[3] |  f[0]
 *       |
 * y<----+-----
 *       |
 *  f[2] |  f[1]
 */
// Note: The motors are assumed to NOT lie on the axes, and specifically
//  motor 0 is at (x,y) = l*(+sqrt(2), -sqrt(2))
//  motor 1 is at (x,y) = l*(-sqrt(2), -sqrt(2))
//  motor 2 is at (x,y) = l*(-sqrt(2), +sqrt(2))
//  motor 3 is at (x,y) = l*(+sqrt(2), +sqrt(2))
class QuadcopterMixer {
 public:
  //prop0SpinDir is +1 if the prop FR spins upwards
  QuadcopterMixer()
      : _d(0),
        _kt(0),
        _kf(0) {
    _minThrustPerPropeller = 0.0f;
    _maxThrustPerPropeller = 1000000.0f;  //just a very large number
    _maxCmdTotalThrust = 4 * _maxThrustPerPropeller;

    for (int i = 0; i < 4; i++) {
      _corrFac[i] = 1.0f;
    }
  }

  void SetParameters(float armLength, float propellerThrustFromSpeedSqr,
                     float propellerTorqueFromThrust, int prop0SpinDir,
                     float maxThrustPerPropeller, float minThrustPerPropeller,
                     float maxCmdTotalThrust = -1.0f) {
    _d = armLength / sqrtf(2.0f);
    _kt = prop0SpinDir * propellerTorqueFromThrust;
    _kf = propellerThrustFromSpeedSqr;
    _maxThrustPerPropeller = maxThrustPerPropeller;
    _minThrustPerPropeller = minThrustPerPropeller;
    if (maxCmdTotalThrust < 0) {
      //default value:
      _maxCmdTotalThrust = 4 * maxThrustPerPropeller * 0.8f;  //leave at least 20% for attitude control
    } else {
      _maxCmdTotalThrust = maxCmdTotalThrust;
    }
  }

  void SetPropellerCorrectionFactors(float factors[4]) {
    for (int i = 0; i < 4; i++) {
      _corrFac[i] = factors[i];
    }
  }

  float GetPropellerCorrectionFactors(unsigned const i) const {
    return _corrFac[i];
  }

  void GetMotorForces(const float totF, const Vec3f t, float outF[4]) {
    //outF is the output (similar to pass-by-reference)

    //saturate the thrust, if needed
    float desF = totF > _maxCmdTotalThrust ? _maxCmdTotalThrust : totF;

    outF[0] = (-t.x / _d - t.y / _d - t.z / _kt + desF) / 4.0f;
    outF[1] = (-t.x / _d + t.y / _d + t.z / _kt + desF) / 4.0f;
    outF[2] = (+t.x / _d + t.y / _d - t.z / _kt + desF) / 4.0f;
    outF[3] = (+t.x / _d - t.y / _d + t.z / _kt + desF) / 4.0f;

    //cap forces:
    for (int i = 0; i < 4; i++) {
      if (outF[i] < _minThrustPerPropeller) {
        outF[i] = _minThrustPerPropeller;
      } else if (outF[i] > _maxThrustPerPropeller) {
        outF[i] = _maxThrustPerPropeller;
      }

    }

    //todo: smarter saturation?
    return;
  }

  void PropellerSpeedsFromThrust(float const thrusts[4],
                                 float outSpeeds[4]) const {
    for (int i = 0; i < 4; i++) {
      float const thrust = thrusts[i];
      if (thrust <= 0) {
        outSpeeds[i] = 0;
        continue;
      }
      outSpeeds[i] = sqrtf(thrust / (_corrFac[i] * _kf));
    }
    return;
  }

  // used for propeller calibration
  float GetUncorrectedForce(float const inSpeed) const {
    return _kf * inSpeed * inSpeed;
  }

 private:
  float _d;  //distance from COM to propeller center, in each component (=armlength/sqrt(2))
  float _kt;  //prop torque constant, [N.m/N]
  float _kf;  //prop force constant, [N/(rad/s)**2]
  float _maxCmdTotalThrust;  // a saturation on the thrust we produce, to ensure we have something left for attitude control.
  float _minThrustPerPropeller, _maxThrustPerPropeller;  //limits per-propeller
  float _corrFac[4];  //propeller correction factors, per motor.

};

}

//namespace Onboard
