/* A controller for quadcopters.
 */

#pragma once
#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"

#include "Components/Logic/QuadcopterPositionController.hpp"
#include "Components/Logic/QuadcopterAttitudeController.hpp"

namespace Offboard {

/**! A high-level controller for a quadcopter. Is mostly a wrapper for the onboard code.
 *
 * The controller is *static*, that is it has no memory (integrators, etc.)
 */
class QuadcopterController {

 public:
  QuadcopterController();

  void SetMinMaxAcceleration(const double minVerticalProperAcc,
                             const double maxProperAcc,
                             const double minProperAcc = -1) {
    _minVerticalProperAcceleration = minVerticalProperAcc;
    _maxProperAcc = maxProperAcc;
    if (minProperAcc < 0) {
      if (_minVerticalProperAcceleration > 0) {
        _minProperAcc = _minVerticalProperAcceleration;
      }
    } else {
      _minProperAcc = minProperAcc;
    }
  }

  void SetParameters(const double posControl_natFreq,
                     const double posControl_damping,
                     const double attControl_timeConst_xy,
                     const double attControl_timeConst_z) {
    _posCtrl.SetParameters(posControl_natFreq, posControl_damping);
    _attCtr.SetParameters(attControl_timeConst_xy, attControl_timeConst_z);
  }

  /**! Run full feedback controller.
   * Can set the desired velocity, attitude and angular velocity to zero if feedforwards aren't available.
   */
  void Run(Vec3d const curPos, Vec3d const curVel, Rotationd const curAtt,
           Vec3d const desPos, Vec3d const desVel, Vec3d const desAcc,
           double const desiredYawAngle, Vec3d &outCmdAngVel,
           double &outCmdThrust) const;

  void RunTracking(Vec3d const curPos, Vec3d const curVel,
                   Rotationd const curAtt, Vec3d const refPos,
                   Vec3d const refVel, Vec3d const refAcc,
                   double const desiredYawAngle, double const refThrust,
                   Vec3d const refAngVel, Vec3d &outCmdAngVel,
                   double &outCmdThrust, Rotationf &outCmdAtt) const;

  double GetPositionControlNaturalFrequency() const {
    return double(_posCtrl.GetNaturalFrequency());
  }

  double GetPositionControlDamping() const {
    return double(_posCtrl.GetDamping());
  }

  Onboard::QuadcopterPositionController _posCtrl;
  Onboard::QuadcopterAttitudeController _attCtr;
 private:

  double _minVerticalProperAcceleration;  //related to the biggest angle that we allow to command [m/s**2]
  double _minProperAcc, _maxProperAcc;  //minimum/maximum proper acceleration we may command, [m/s**2]
};

}
