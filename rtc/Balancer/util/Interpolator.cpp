// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  Interpolator.cpp
 * @brief Library for interpolation
 * @author Tatsuya Ishikawa
 */

#include "Interpolator.h"
#include <cmath>

minJerkCoeff calcMinJerkCoeff(const Eigen::Vector3d& start, const Eigen::Vector3d& finish, const double T)
{
    const double a0 = start[0];
    const double a1 = start[1];
    const double a2 = start[2] / 2.0;
    const double a3 = 1 / (2 * T*T*T)     * ((-6 * a2 + finish[2]) * T*T +
                                             (-12 * a1 - 8 * finish[1]) * T +
                                             20 * (-a0 + finish[0]));
    const double a4 = -1 / (T*T*T*T)      * ((-3 * a2 + finish[2]) * T*T +
                                             (-8 * a1 - 7 * finish[1]) * T +
                                             15 * (-a0 + finish[0]));
    const double a5 = 1 / (2 * T*T*T*T*T) * ((-2 * a2 + finish[2]) * T*T +
                                             (-6 * a1 - 6 * finish[1]) * T +
                                             12 * (-a0 + finish[0]));

    return minJerkCoeff{{a0, a1, a2, a3, a4, a5}};
}

minJerkCoeffTime calcMinJerkCoeffWithTimeInitJerkZero(const Eigen::Vector3d& start, const Eigen::Vector3d& finish)
{
    const double a0 = start[0];
    const double a1 = start[1];
    const double a2 = start[2] / 2.0;
    // Should select -2 or +2 ?
    const double T  = -1 / (6 * a2 - finish[2]) *
        (6 * a1 + 4 * finish[1] - 2 * sqrt(-30 * a0 * a2 + 5 * a0 * finish[2] +
                                           9 * a1*a1 + 12 * a1 * finish[1] +
                                           30 * a2 * finish[0] - 5  * finish[2] * finish[0]
                                           + 4 * finish[1] * finish[1]));
    const double a3 = 1 / (2 * T*T*T)     * ((-6 * a2 + finish[2]) * T*T +
                                             (-12 * a1 - 8 * finish[1]) * T +
                                             20 * (-a0 + finish[0]));
    const double a4 = -1 / (T*T*T*T)      * ((-3 * a2 + finish[2]) * T*T +
                                             (-8 * a1 - 7 * finish[1]) * T +
                                             15 * (-a0 + finish[0]));
    const double a5 = 1 / (2 * T*T*T*T*T) * ((-2 * a2 + finish[2]) * T*T +
                                             (-6 * a1 - 6 * finish[1]) * T +
                                             12 * (-a0 + finish[0]));

    return minJerkCoeffTime{{a0, a1, a2, a3, a4, a5, T}};
}

template<size_t n>
double calcNthOrderSpline(const std::array<double, n>& coeff, const double t)
{
    double value = 0;
    for (size_t i = 0; i < n + 1; ++i) {
        value += coeff[i] * pow(t, i);
    }
    return value;
}
