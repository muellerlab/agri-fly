/***************************************************************************
 *   Copyright (C) 2016 by Саша Миленковић                                 *
 *   sasa.milenkovic.xyz@gmail.com                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *   ( http://www.gnu.org/licenses/gpl-3.0.en.html )                       *
 *                     *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*
 * Modified by Gaurav Jalan @HiPeRLab Berkeley
 * made compatible with "Real" data type
 * Changed function arguments
 * Added new Quartic namespace
 */

/*
 * Modified by Nathan Bucki
 * Moved everything into header file
 * Changed conventions to match lab code
 * Changed namespace from "Quartic" to "RootFinder" because 3rd order polynomials are included
 */

#pragma once
#include <math.h>

namespace RootFinder {

const float PI = 3.141592653589793238463;

const float M_2PI = 2 * PI;

const float eps = 1e-12;

float tsqrt(float in);
double tsqrt(double in);
float tcos(float in);
double tcos(double in);
float tacos(float in);
double tacos(double in);
float tabs(float in);
double tabs(double in);

template<typename Real>
// x - array of size 3
// In case 3 real roots: => x[0], x[1], x[2], return 3
//         2 real roots: x[0], x[1],          return 2
//         1 real root : x[0], x[1] ± i*x[2], return 1
unsigned int solve_cubic(Real a, Real b, Real c, Real* x) {
  Real a2 = a * a;
  Real q = (a2 - 3 * b) / 9;
  Real r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
  Real r2 = r * r;
  Real q3 = q * q * q;
  Real A, B;
  if (r2 < q3) {
    Real t = r / tsqrt(q3);
    if (t < -1)
      t = -1;
    if (t > 1)
      t = 1;
    t = tacos(t);
    a /= 3;
    q = -2 * tsqrt(q);
    x[0] = q * tcos(t / 3) - a;
    x[1] = q * tcos((t + Real(M_2PI)) / Real(3)) - a;
    x[2] = q * tcos((t - Real(M_2PI)) / Real(3)) - a;
    return 3;
  } else {
    A = -pow(tabs(r) + tsqrt(r2 - q3), 1. / 3);
    if (r < 0)
      A = -A;
    B = (tabs(A) < Real(eps) ? 0 : q / A);

    a /= 3;
    x[0] = (A + B) - a;
    x[1] = Real(-0.5) * (A + B) - a;
    x[2] = Real(0.5) * tsqrt(Real(3.)) * (A - B);
    if (tabs(x[2]) < Real(eps)) {
      x[2] = x[1];
      return 2;
    }

    return 1;
  }
}

//---------------------------------------------------------------------------
// solve quartic equation x^4 + a*x^3 + b*x^2 + c*x + d
// Attention - this function returns dynamically allocated array. It has to be released afterwards.

template<typename Real>

unsigned int solve_quartic(const Real& a, const Real& b, const Real& c,
                           const Real& d, Real* root) {
  Real a3 = -b;
  Real b3 = a * c - Real(4.) * d;
  Real c3 = -a * a * d - c * c + Real(4.) * b * d;

  //initialise counters for real and imaginary roots
  int rCnt = 0;
  // cubic resolvent
  // y^3 − b*y^2 + (ac−4d)*y − a^2*d−c^2+4*b*d = 0

  Real x3[3];
  unsigned int iZeroes = solve_cubic(a3, b3, c3, x3);

  Real q1, q2, p1, p2, D, sqD, y;

  y = x3[0];
  // The essence - choosing Y with maximal absolute value.
  if (iZeroes != 1) {
    if (tabs(x3[1]) > tabs(y))
      y = x3[1];
    if (tabs(x3[2]) > tabs(y))
      y = x3[2];
  }

  // h1+h2 = y && h1*h2 = d  <=>  h^2 -y*h + d = 0    (h === q)

  D = y * y - 4 * d;
  if (tabs(D) < Real(eps))  //in other words - D==0
      {
    q1 = q2 = y * Real(0.5);
    // g1+g2 = a && g1+g2 = b-y   <=>   g^2 - a*g + b-y = 0    (p === g)
    D = a * a - Real(4) * (b - y);
    if (tabs(D) < Real(eps))  //in other words - D==0
      p1 = p2 = a * Real(0.5);

    else {
      sqD = tsqrt(D);
      p1 = (a + sqD) * Real(0.5);
      p2 = (a - sqD) * Real(0.5);
    }
  } else {
    sqD = tsqrt(D);
    q1 = (y + sqD) * Real(0.5);
    q2 = (y - sqD) * Real(0.5);
    // g1+g2 = a && g1*h2 + g2*h1 = c       ( && g === p )  Krammer
    p1 = (a * q1 - c) / (q1 - q2);
    p2 = (c - a * q2) / (q1 - q2);
  }

  // solving quadratic eq. - x^2 + p1*x + q1 = 0
  D = p1 * p1 - 4 * q1;
  if (!(D < Real(0.0))) {
    // real roots filled from left
    sqD = tsqrt(D);
    root[rCnt] = (-p1 + sqD) * Real(0.5);
    ++rCnt;
    root[rCnt] = (-p1 - sqD) * Real(0.5);
    ++rCnt;
  }

  // solving quadratic eq. - x^2 + p2*x + q2 = 0
  D = p2 * p2 - 4 * q2;
  if (!(D < Real(0.0))) {
    sqD = tsqrt(D);
    root[rCnt] = (-p2 + sqD) * Real(0.5);
    ++rCnt;
    root[rCnt] = (-p2 - sqD) * Real(0.5);
    ++rCnt;
  }

  return rCnt;
}

inline float tsqrt(float in) {
  return sqrtf(in);
}

inline double tsqrt(double in) {
  return sqrt(in);
}

inline float tcos(float in) {
  return cosf(in);
}

inline double tcos(double in) {
  return cos(in);
}

inline float tacos(float in) {
  return acosf(in);
}

inline double tacos(double in) {
  return acos(in);
}

inline float tabs(float in) {
  return fabsf(in);
}

inline double tabs(double in) {
  return fabs(in);
}

}

