#pragma once

#include <mutex>

#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"
#include "Common/Time/Timer.hpp"
#include "Components/Offboard/MocapStateEstimator.hpp"

namespace Offboard {

/* Some simple safety rules that you can use to automatically detect "bad" situations.
 */
class SafetyNet {
 public:

  struct SafetyState {
    SafetyState() {
      vehicleNotSeen = true;
      unsafePosition = false;
      upsideDownAndLow = false;
      userUnsafe = false;
    }

    bool IsSafe() const {
      if (vehicleNotSeen) {
        return false;
      }

      if (unsafePosition) {
        return false;
      }

      if (upsideDownAndLow) {
        return false;
      }

      if (userUnsafe) {
        return false;
      }

      return true;
    }

    bool vehicleNotSeen;
    bool unsafePosition;
    bool upsideDownAndLow;
    bool userUnsafe;

  };

  SafetyNet()
      : _safetyState(),
        _minCorner(-2.4, -3.1, -0.5), // Lab space, approximately
        _maxCorner(+1.8, +3.1, 4.5), // Lab space, approximately
        _minNormalHeight(1.0),  // we're only allowed below this height while pointing upwards
        _vehicleNotSeenTimeout(0.5) {
    // min/max corner X-Y values form a box approximately 10cm inside the side walls and vertical net
    // -X is the back wall and +X is the vertical net
    // +Z is chosen to be roughly the maximum height the mocap can see. The ceiling is slightly higher
  }

  void SetSafeCorners(Vec3d minCorner, Vec3d maxCorner,
                      double minNormalHeight) {
    _minCorner = minCorner;
    _maxCorner = maxCorner;
    _minNormalHeight = minNormalHeight;
    for (int i = 0; i < 3; i++) {
      assert(_minCorner[i] < _maxCorner[i]);
    }
  }

  void UpdateWithEstimator(MocapStateEstimator::MocapEstimatedState const est,
                           double const timeSinceLastGoodMeas) {
    if (timeSinceLastGoodMeas > _vehicleNotSeenTimeout) {
      _safetyState.vehicleNotSeen = true;
    } else {
      _safetyState.vehicleNotSeen = false;
    }

    _safetyState.unsafePosition = false;
    for (int i = 0; i < 3; i++) {
      if (est.pos[i] < _minCorner[i]) {
        _safetyState.unsafePosition = true;
      }
      if (est.pos[i] > _maxCorner[i]) {
        _safetyState.unsafePosition = true;
      }
    }

    _safetyState.upsideDownAndLow = false;
    if (est.pos.z < _minNormalHeight) {
      if ((est.att * Vec3d(0, 0, 1)).z < 0) {
        //pointing downwards
        _safetyState.upsideDownAndLow = true;
      }
    }
  }

  bool GetIsSafe() const {
    return _safetyState.IsSafe();
  }

  void SetUnsafe() {
    _safetyState.userUnsafe = true;
  }

  SafetyState GetSafetyState() const {
    return _safetyState;
  }

  std::string GetStatusString() const {
    if (_safetyState.IsSafe()) {
      return std::string("all OK");
    }

    std::stringstream ss;
    ss << "Not safe: ";
    if (_safetyState.unsafePosition) {
      ss << "(unsafe position) ";
    }
    if (_safetyState.vehicleNotSeen) {
      ss << "(not seen) ";
    }
    if (_safetyState.userUnsafe) {
      ss << "(user triggered) ";
    }
    if (_safetyState.upsideDownAndLow) {
      ss << "(upside down and low) ";
    }
    return ss.str();
  }

 private:
  SafetyState _safetyState;

  Vec3d _minCorner, _maxCorner;  //The limits of the safe flight space
  double _minNormalHeight;  //we're only allowed below this height while pointing upwards
  double _vehicleNotSeenTimeout;  //how long the vehicle can be invisible before an issue

};

}
