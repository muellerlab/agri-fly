/*
 * LowPassFilterSecondOrder
 *
 *      Author: mwm
 */

#pragma once
#include <math.h>

template<typename TYPE_RATE, typename TYPE_SAMPLE>
class LowPassFilterSecondOrder {

 public:
  LowPassFilterSecondOrder() {
    _a1 = 0;
    _a2 = 0;
    _b0 = 1;
    _b1 = 0;
    _b2 = 0;
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

    //short-hand
    TYPE_RATE const dt = samplingPeriod;
    TYPE_RATE const wc = cutoffFrequency__rad_per_s;
    TYPE_RATE const sqrt2 = TYPE_RATE(sqrt(2.0));
    _a1 = (dt * dt * wc * wc - 2 * sqrt2 * dt * wc + 4)
        / (dt * dt * wc * wc + 2 * sqrt2 * dt * wc + 4);
    _a2 = 2 * (dt * dt * wc * wc - 4)
        / (dt * dt * wc * wc + 2 * sqrt2 * dt * wc + 4);
    _b0 = dt * dt * wc * wc / (dt * dt * wc * wc + 2 * sqrt2 * dt * wc + 4);
    _b1 = dt * dt * wc * wc / (dt * dt * wc * wc + 2 * sqrt2 * dt * wc + 4);
    _b2 = 2 * dt * dt * wc * wc / (dt * dt * wc * wc + 2 * sqrt2 * dt * wc + 4);

    _xm0 = _xm1 = _ym0 = _ym1 = initValue;

  }

  TYPE_SAMPLE Apply(TYPE_SAMPLE input) {

    TYPE_SAMPLE output = _b2 * input;
    output += +_b0 * _xm0 + _b1 * _xm1;
    output += -_a1 * _ym0 - _a2 * _ym1;
    //time shifts:
    _xm0 = _xm1;
    _xm1 = input;

    _ym0 = _ym1;
    _ym1 = output;

    return output;
  }

  TYPE_SAMPLE GetValue() const {
    return _ym1;
  }

 private:
  TYPE_RATE _a1, _a2;  //output coefficients
  TYPE_RATE _b0, _b1, _b2;  //input coefficients
  TYPE_SAMPLE _xm0, _xm1;  //prior inputs
  TYPE_SAMPLE _ym0, _ym1;  //prior outputs

};

