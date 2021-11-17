/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Nathan:
 * Reformatted
 * Fixed problems with GetOmega (TEST THIS)
 */

#include <algorithm>
#include <limits>
#include <cerrno>
#include "Components/TrajectoryGenerator/RapidTrajectoryGenerator.hpp"
#include "Common/Math/RootFinder.hpp"

using namespace CommonMath;
using namespace RapidQuadrocopterTrajectoryGenerator;

RapidTrajectoryGenerator::RapidTrajectoryGenerator(const Vec3d x0,
                                                   const Vec3d v0,
                                                   const Vec3d a0,
                                                   const Vec3d gravity) {
  //initialise each axis:
  Reset();
  for (int i = 0; i < 3; i++)
    _axis[i].SetInitialState(x0[i], v0[i], a0[i]);
  _grav = gravity;
}

void RapidTrajectoryGenerator::SetGoalPosition(const Vec3d in) {
  for (unsigned i = 0; i < 3; i++)
    SetGoalPositionInAxis(i, in[i]);
}

void RapidTrajectoryGenerator::SetGoalVelocity(const Vec3d in) {
  for (int i = 0; i < 3; i++)
    SetGoalVelocityInAxis(i, in[i]);
}

void RapidTrajectoryGenerator::SetGoalAcceleration(const Vec3d in) {
  for (int i = 0; i < 3; i++)
    SetGoalAccelerationInAxis(i, in[i]);
}

void RapidTrajectoryGenerator::Reset(void) {
  for (int i = 0; i < 3; i++) {
    _axis[i].Reset();
  }
  _tf = 0;
}

//Generate the trajectory:
void RapidTrajectoryGenerator::Generate(const double timeToFinish) {
  _tf = timeToFinish;
  for (int i = 0; i < 3; i++) {
    _axis[i].GenerateTrajectory(_tf);
  }
}

RapidTrajectoryGenerator::InputFeasibilityResult RapidTrajectoryGenerator::CheckInputFeasibilitySection(
    double fminAllowed, double fmaxAllowed, double wmaxAllowed, double t1,
    double t2, double minTimeSection) {
  if (t2 - t1 < minTimeSection)
    return InputIndeterminable;
  //test the acceleration at the two limits:
  if (std::max(GetThrust(t1), GetThrust(t2)) > fmaxAllowed)
    return InputInfeasibleThrustHigh;
  if (std::min(GetThrust(t1), GetThrust(t2)) < fminAllowed)
    return InputInfeasibleThrustLow;

  double fminSqr = 0;
  double fmaxSqr = 0;
  double jmaxSqr = 0;

  //Test the limits of the box we're putting around the trajectory:
  for (int i = 0; i < 3; i++) {
    double amin, amax;
    _axis[i].GetMinMaxAcc(amin, amax, t1, t2);

    //distance from zero thrust point in this axis
    double v1 = amin - _grav[i];  //left
    double v2 = amax - _grav[i];  //right

    //definitely infeasible:
    if (std::max(pow(v1, 2), pow(v2, 2)) > pow(fmaxAllowed, 2))
      return InputInfeasibleThrustHigh;

    if (v1 * v2 < 0) {
      //sign of acceleration changes, so we've gone through zero
      fminSqr += 0;
    } else {
      fminSqr += pow(std::min(fabs(v1), fabs(v2)), 2);
    }

    fmaxSqr += pow(std::max(fabs(v1), fabs(v2)), 2);

    jmaxSqr += _axis[i].GetMaxJerkSquared(t1, t2);
  }

  double fmin = sqrt(fminSqr);
  double fmax = sqrt(fmaxSqr);
  double wBound;
  if (fminSqr > 1e-6)
    wBound = sqrt(jmaxSqr / fminSqr);  //the 1e-6 is a divide-by-zero protection
  else
    wBound = std::numeric_limits<double>::max();

  //definitely infeasible:
  if (fmax < fminAllowed)
    return InputInfeasibleThrustLow;
  if (fmin > fmaxAllowed)
    return InputInfeasibleThrustHigh;

  //possibly infeasible:
  if (fmin < fminAllowed || fmax > fmaxAllowed || wBound > wmaxAllowed) {  //indeterminate: must check more closely:
    double tHalf = (t1 + t2) / 2;
    InputFeasibilityResult r1 = CheckInputFeasibilitySection(fminAllowed,
                                                             fmaxAllowed,
                                                             wmaxAllowed, t1,
                                                             tHalf,
                                                             minTimeSection);

    if (r1 == InputFeasible) {
      //continue with second half
      return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed,
                                          tHalf, t2, minTimeSection);
    }

    //first section is already infeasible, or indeterminate:
    return r1;
  }

  //definitely feasible:
  return InputFeasible;
}

RapidTrajectoryGenerator::InputFeasibilityResult RapidTrajectoryGenerator::CheckInputFeasibility(
    double fminAllowed, double fmaxAllowed, double wmaxAllowed,
    double minTimeSection) {
  //required thrust limits along trajectory
  double t1 = 0;
  double t2 = _tf;

  return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, t1,
                                      t2, minTimeSection);
}

RapidTrajectoryGenerator::StateFeasibilityResult RapidTrajectoryGenerator::CheckVelocityFeasibility(
    double vmax) {
  // perform per-axis test
  for (int dim = 0; dim < 3; dim++) {
    double c[4] = { 0 };

    c[0] = _axis[dim].GetParamAlpha() / 6.0;     //t**3
    c[1] = _axis[dim].GetParamBeta() / 2.0;      //t**2
    c[2] = _axis[dim].GetParamGamma() / 1.0;     //t**1
    c[3] = _axis[dim].GetInitialAcceleration();  //1

    // last two for 0 and _tf
    double roots[3 + 2];
    size_t rootCount = 0;

    if (fabs(c[0]) > 1e-6) {
      rootCount = RootFinder::solve_cubic<double>(c[1] / c[0], c[2] / c[0],
                                                  c[3] / c[0], roots);
    } else {
      // TODO TECHNICALLY WE NEED TO IMPLEMENT P2 AND P1
      // rootCount = Quartic::solveP2(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);
      return StateInfeasible;
    }

    // want 0 and _tf to be tested
    roots[rootCount] = 0;
    roots[rootCount + 1] = _tf;

    for (unsigned i = 0; i < (rootCount + 2); i++) {
      //don't evaluate points outside the domain
      if (roots[i] < 0)
        continue;
      if (roots[i] > _tf)
        continue;

      Vec3d velAtRoot = GetVelocity(roots[i]);
      if (std::abs(velAtRoot.x) >= vmax || std::abs(velAtRoot.y) >= vmax
          || std::abs(velAtRoot.z) >= vmax) {
        //exceeded allowed velocity
        return StateInfeasible;
      }
    }
  }

  return StateFeasible;
}

RapidTrajectoryGenerator::StateFeasibilityResult RapidTrajectoryGenerator::CheckPositionFeasibility(
    Vec3d boundaryPoint, Vec3d boundaryNormal) {
  //Ensure that the normal is a unit vector:
  boundaryNormal = boundaryNormal.GetUnitVector();

  //first, we will build the polynomial describing the velocity of the a
  //quadrocopter in the direction of the normal. Then we will solve for
  //the zeros of this, which give us the times when the position is at a
  //critical point. Then we evaluate the position at these points, and at
  //the trajectory beginning and end, to see whether we are feasible.

  //need to check that the trajectory stays inside the safe box throughout the flight:

  //the coefficients of the quartic equation: x(t) = c[0]t**4 + c[1]*t**3 + c[2]*t**2 + c[3]*t + c[4]
  double c[5] = { 0, 0, 0, 0, 0 };

  for (int dim = 0; dim < 3; dim++) {
    c[0] += boundaryNormal[dim] * _axis[dim].GetParamAlpha() / 24.0;  //t**4
    c[1] += boundaryNormal[dim] * _axis[dim].GetParamBeta() / 6.0;  //t**3
    c[2] += boundaryNormal[dim] * _axis[dim].GetParamGamma() / 2.0;  //t**2
    c[3] += boundaryNormal[dim] * _axis[dim].GetInitialAcceleration();  //t
    c[4] += boundaryNormal[dim] * _axis[dim].GetInitialVelocity();  //1
  }

  //Solve the roots (we prepend the times 0 and tf):
  double roots[6];
  roots[0] = 0;
  roots[1] = _tf;

  size_t rootCount;
  if (fabs(c[0]) > 1e-6) {
    rootCount = RootFinder::solve_quartic<double>(c[1] / c[0], c[2] / c[0],
                                                  c[3] / c[0], c[4] / c[0],
                                                  roots);
  } else {
    rootCount = RootFinder::solve_cubic<double>(c[2] / c[1], c[3] / c[1],
                                                c[4] / c[1], roots);
  }

  for (unsigned i = 0; i < (rootCount + 2); i++) {
    //don't evaluate points outside the domain
    if (roots[i] < 0)
      continue;
    if (roots[i] > _tf)
      continue;

    if ((GetPosition(roots[i]) - boundaryPoint).Dot(boundaryNormal) <= 0) {
      //touching, or on the wrong side of, the boundary!
      return StateInfeasible;
    }
  }
  return StateFeasible;
}

Vec3d RapidTrajectoryGenerator::GetOmega(double t, double timeStep) const {
  //Calculates the body rates necessary at time t, to rotate the normal vector.
  //The result is coordinated in the world frame, i.e. would have to be rotated into a
  //body frame.
  const Vec3d n0 = GetNormalVector(t);
  const Vec3d n1 = GetNormalVector(t + timeStep);

  const Vec3d crossProd = n0.Cross(n1);  //direction of omega, in inertial axes
  if (crossProd.GetNorm2() <= 1e-6) {
    // Cross product failed
    return Vec3d(0, 0, 0);
  } else {
    errno = 0;
    Vec3d n = crossProd.GetUnitVector();
    double angle = acos(n0.Dot(n1)) / timeStep;
    if (errno) {
      // either unit vector not defined or numeric issues cause dot product to fail
      return Vec3d(0, 0, 0);
    } else {
      return angle * n;
    }
  }
}
