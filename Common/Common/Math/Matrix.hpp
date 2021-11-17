/* mwm
 * 
 */

#pragma once

#ifdef _MICROCONTROLLER
#include "../matrix/matrix/math.hpp"
#else
#include <Eigen/Dense>
#endif

#ifdef _MICROCONTROLLER
//we use the PX4 class
template<typename Real, int M, int N>
using Matrix = matrix::Matrix<Real, M, N>;

template<typename Real, int M>
using SquareMatrix = matrix::SquareMatrix<Real, M>;

template<typename Real, int M, int N>
Matrix<Real, M,N> ZeroMatrix() {
  Matrix<Real, M, N> m;
  m.zero();
  return m;
}

template<typename Real, int M>
SquareMatrix<Real, M> IdentityMatrix() {
  SquareMatrix<Real, M> m;
  m.identity();
  return m;
}

#else
template<typename Real, int M, int N>
using Matrix = Eigen::Matrix<Real, M, N>;
template<typename Real, int M>
using SquareMatrix = Eigen::Matrix<Real, M, M>;

template<typename Real, int M, int N>
Matrix<Real, M, N> ZeroMatrix() {
  return Matrix<Real, M, N>::Zero();
}

template<typename Real, int M>
SquareMatrix<Real, M> IdentityMatrix() {
  return SquareMatrix<Real, M>::Identity();

}
#endif
