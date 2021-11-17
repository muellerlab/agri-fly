/* Hardware Abstraction Timer
 * A clock you can use in the onboard code to get the time.
 * mwm
 *
 * A simple clock. In simulation, you must slave it to the simulation timer.
 * on the px4, should have microsecond precision, and wraps only after approx.
 * 2^64 microseconds (-> a VERY long time).
 *
 */

#pragma once

#include "BaseTimer.hpp"

#ifndef __EXPORT
#define __EXPORT
#endif

class __EXPORT Timer {
 public:
  Timer(BaseTimer* const masterTimer)
      : _masterTimer(masterTimer) {
    Reset();
  }

  template<typename Real>
  void AdjustTimeBySeconds(Real additionalSeconds) {
    if (additionalSeconds > 0) {
      _lastResetTime_usec -= uint64_t(additionalSeconds * Real(1e6));
    } else {
      _lastResetTime_usec += uint64_t(additionalSeconds * Real(-1e6));
    }
  }

  template<typename Real>
  Real GetSeconds(void) const {
    return (Real) (GetMicroSeconds() * Real(1e-6));  //from microseconds, to seconds
  }

  double GetSeconds_d(void) const {
    return GetSeconds<double>();
  }

  float GetSeconds_f(void) const {
    return GetSeconds<float>();
  }

  uint64_t GetMicroSeconds(void) const {
    return _masterTimer->GetMicroSeconds() - _lastResetTime_usec;
  }

  void Reset(void) {
    _lastResetTime_usec = _masterTimer->GetMicroSeconds();
  }

  BaseTimer* GetMasterTimer() const {
    return _masterTimer;
  }

 private:
  BaseTimer* const _masterTimer;  //const, because we don't want to accidentally change someone else's time, who may share our master timer.
  uint64_t _lastResetTime_usec;
};
