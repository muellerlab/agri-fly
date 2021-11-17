/* A base timer class, to be extended.
 *
 */

#pragma once

#include <stdint.h>

#ifndef __EXPORT
#define __EXPORT
#endif

class __EXPORT BaseTimer {
 public:
  BaseTimer() {
  }

  virtual ~BaseTimer() {
  }

  virtual uint64_t GetMicroSeconds(void) const = 0;

};
