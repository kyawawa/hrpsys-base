// -*- mode: C++ -*-

/**
 * @file  StateEstimator.h
 * @brief
 * @date  $Date$
 */

#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <cmath>
#include <hrplib/EigenTypes.h>
#include "LinkConstraint.h"

namespace hrp {

constexpr double DEFAULT_GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

struct stateValues
{
    // hrp::BodyPtr body;
    hrp::dvector q;
    hrp::Vector3 cog;
    hrp::Vector3 cp;
};

hrp::Vector3 calcActZMP(const std::vector<LinkConstraint>& constraints,
                        const double zmp_z);

inline hrp::Vector3 calcCP(const hrp::Vector3& cog, const hrp::Vector3& cog_vel, const double zmp_z,
                           const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION)
{
    return cog + cog_vel / std::sqrt(g_acc / (cog[2] - zmp_z));
}

bool calcIsOnGround();

}

#endif // STATEESTIMATOR_H
