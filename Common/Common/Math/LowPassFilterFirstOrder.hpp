/*
 * LowPassFilterorder.hpp
 *
 */

#pragma once
#include <math.h>

template<typename TYPE_RATE, typename TYPE_SAMPLE>
class LowPassFilterFirstOrder {

 public:
  LowPassFilterFirstOrder()
      : _coefficient(0.0f) {
    // empty here
  }

  void Initialise(TYPE_RATE samplingPeriod,
                  TYPE_RATE cutoffFrequency__rad_per_s, TYPE_SAMPLE initValue) {
		/* Inputs:
		 * samplingPeriod [s] -- the period with which the filter is called (e.g. 
		 *  if we're using in a loop running at 500 Hz, then this is 
		 *  samplingPeriod = 1/500.0=0.002
		 * cutoffFrequency__rad_per_s -- the cut-off frequency, alternatively the 
		 *  inverse of the filter time constant. Note that frequency is in rad/s
		 *  not in Hz. For a time constant of 10s, we have 
		 *  cutoffFrequency__rad_per_s = 1/10 = 0.1rad/s
		 * initValue -- where the filter is initialized. 
		 */ 
    _previousValue = initValue;
    _coefficient = exp(-samplingPeriod * cutoffFrequency__rad_per_s);  //note: this will be very slow on the microcontroller, due to double-precision exp call. However, since we're likely only calling this once, it's not so bad.
  }

  TYPE_SAMPLE Apply(TYPE_SAMPLE input) {

    if (_coefficient <= 0.0f) {
      // no filtering
      _previousValue = input;
      return input;
    }

    TYPE_SAMPLE output = _coefficient * _previousValue
        + (1 - _coefficient) * input;

    _previousValue = output;

    return output;
  }

  TYPE_SAMPLE GetValue() const {
    return _previousValue;
  }

 private:

  TYPE_RATE _coefficient;
  TYPE_SAMPLE _previousValue;

};
