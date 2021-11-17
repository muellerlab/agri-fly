/* Hardware Abstraction Timer
 * A clock you must manually advance, but is interface-compatible with BaseTimer class.
 *
 */

#pragma once

#include "BaseTimer.hpp"

#ifndef __EXPORT
#define __EXPORT
#endif

//A timer that is moved manually
class __EXPORT ManualTimer : public BaseTimer {
 public:
  ManualTimer()
      : _currentTime(0) {

  }

  virtual ~ManualTimer() {
  }

  void ResetMicroseconds(uint64_t t_us) {
    _currentTime = t_us;
  }

  void AdvanceMicroSeconds(uint64_t dt_us) {
    _currentTime += dt_us;
  }

  template<typename Real>
  Real GetSeconds(void) const {
    return (Real) (GetMicroSeconds() * Real(1e-6));  //from microseconds, to seconds
  }

  virtual uint64_t GetMicroSeconds(void) const {
    return _currentTime;
  }

 private:

  uint64_t _currentTime;
};
