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

#include <stdint.h>

#include "BaseTimer.hpp"

#ifdef _MICROCONTROLLER
#include <drivers/drv_hrt.h>

#else
#include <chrono>
#include <ctime>
#include <ratio>
#endif

#ifndef __EXPORT
#define __EXPORT
#endif

class __EXPORT HardwareTimer : public BaseTimer {
 public:
  HardwareTimer()
#ifdef _MICROCONTROLLER
      {
#else
      : _creationTime(std::chrono::steady_clock::now()) {
#endif
  }

  virtual ~HardwareTimer() {
  }

  virtual uint64_t GetMicroSeconds(void) const {
#ifdef _MICROCONTROLLER
    return hrt_absolute_time();
#else
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - _creationTime).count();
#endif
  }

 private:

#ifdef _MICROCONTROLLER
  //nothing
#else
  const std::chrono::time_point<std::chrono::steady_clock> _creationTime;
#endif

};
