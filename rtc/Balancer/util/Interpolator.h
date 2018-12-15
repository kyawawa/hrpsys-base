// -*- mode: C++; coding: utf-8-unix; -*-

#ifndef __INTERPOLATOR_H__
#define __INTERPOLATOR_H__

#include <array>
#include <Eigen/Core>

using minJerkCoeff = std::array<double, 6>; // a0 - a5
using minJerkCoeffTime = std::array<double, 7>; // a0 - a5, T

minJerkCoeff calcMinJerkCoeff(const Eigen::Vector3d& start, const Eigen::Vector3d& finish, const double T);

/*
  Calculate coefficient of minimum jerk interpolstion with Time
  when Initial Jerk = 0
*/
minJerkCoeffTime calcMinJerkCoeffWithTimeInitJerkZero(const Eigen::Vector3d& start, const Eigen::Vector3d& finish);

// double calcFifthOrderSpline(double t, const minJerkCoeff& coeff);

template<size_t n>
double calcNthOrderSpline(const std::array<double, n>& coeff, const double t)
{
    double value = 0;
    for (size_t i = 0; i < n + 1; ++i) {
        value += coeff[i] * pow(t, i);
    }
    return value;
}

#endif // __INTERPOLATOR_H__
