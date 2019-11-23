// -*- mode: C++ -*-

/**
 * @file  StateEstimator.cpp
 * @brief
 * @date  $Date$
 */

#include "StateEstimator.h"

namespace hrp {

hrp::Vector3 calcActZMP(const std::vector<LinkConstraint>& constraints,
                        const std::vector<hrp::dvector6>& wrenches,
                        const double zmp_z)
{
    // TODO: rename
    double tmpzmpx = 0;
    double tmpzmpy = 0;
    double tmpfz = 0;
    // double tmpfz2 = 0.0;

    for (const LinkConstraint& constraint : constraints) {
        // TODO: is_zmp_calc_enableの代わり
        // if (!is_zmp_calc_enable[i]) continue;
        const hrp::ForceSensor* const sensor = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
        const hrp::Matrix33 sensor_R = sensor->link->R * sensor->localR;
        // rats::rotm3times(sensor_R, sensor->link->R, sensor->localR);
        const hrp::Vector3 nf = sensor_R * wrenches[i].head<3>();
        const hrp::Vector3 nm = sensor_R * wrenches[i].tail<3>();
        const hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
        tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
        tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
        tmpfz += nf(2);

        // calc ee-local COP
        // const hrp::Link* target = m_robot->link(stikp[i].target_name);
        // const hrp::Matrix33 eeR = target->R * stikp[i].localR;
        // const hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * stikp[i].localp)); // ee-local force sensor pos
        // nf = eeR.transpose() * nf;
        // nm = eeR.transpose() * nm;
        // // ee-local total moment and total force at ee position
        // const double tmp_cop_mx = nf(2) * ee_fsp(1) - nf(1) * ee_fsp(2) + nm(0);
        // const double tmp_cop_my = nf(2) * ee_fsp(0) - nf(0) * ee_fsp(2) - nm(1);
        // const double tmp_cop_fz = nf(2);
        // contact_cop_info[i][0] = tmp_cop_mx;
        // contact_cop_info[i][1] = tmp_cop_my;
        // contact_cop_info[i][2] = tmp_cop_fz;
        // prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
        // tmpfz2 += prev_act_force_z[i];
    }

    return hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);

    // if (tmpfz2 < contact_decision_threshold) {
    //     ret_zmp = act_zmp;
    //     return false; // in the air
    // } else {
    //     ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
    //     return true; // on ground
    // }
}

}
