// -*- mode: C++ -*-

/**
 * @file  StateEstimator.h
 * @brief
 * @date  $Date$
 */

#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <cmath>
#include <Eigen/Geometry>
#include <hrpUtil/EigenTypes.h>
#include <hrpModel/Body.h>
#include "LinkConstraint.h"

namespace hrp {

struct stateValues
{
    // hrp::BodyPtr body;
    hrp::dvector q;
    hrp::Vector3 cog;
    hrp::Vector3 cp; // Capture Point
    hrp::Vector3 zmp;
    Eigen::Isometry3d foot_origin_coord;

    hrp::Vector3 transformFootOriginCoord(const hrp::Vector3& target) {
        return foot_origin_coord.linear() * (target - foot_origin_coord.translation());
    }
};

hrp::Vector3 calcActZMP(const hrp::BodyPtr& act_robot,
                        const std::vector<LinkConstraint>& constraints,
                        const double zmp_z);

hrp::Vector3 calcCOPFromRobotState(const hrp::BodyPtr& act_robot,
                                   const std::vector<LinkConstraint>& constraints,
                                   const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT);

hrp::Matrix33 calcCOPRotationFromRobotState(const hrp::BodyPtr& act_robot,
                                            const std::vector<LinkConstraint>& constraints,
                                            const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT);

inline hrp::Vector3 calcCP(const hrp::Vector3& cog, const hrp::Vector3& cog_vel, const double zmp_z,
                           const double g_acc = 9.80665)
{
    return cog + cog_vel / std::sqrt(g_acc / (cog[2] - zmp_z));
}

bool calcIsOnGround();

}

#endif // STATEESTIMATOR_H
