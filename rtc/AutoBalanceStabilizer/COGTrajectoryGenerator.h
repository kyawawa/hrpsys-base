// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  COGTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef COGTRAJECTORYGENERATOR_H
#define COGTRAJECTORYGENERATOR_H

#include <vector>
#include <memory>
#include <hrpUtil/EigenTypes.h>
#include "PreviewController.h"

namespace hrp {

class COGTrajectoryGenerator
{
  public:
    enum CogCalculationType : size_t { PREVIEW_CONTROL, FOOT_GUIDED };

  private:
    static constexpr double DEFAULT_GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

    hrp::Vector3 cog     = hrp::Vector3::Zero();
    hrp::Vector3 cog_vel = hrp::Vector3::Zero();
    hrp::Vector3 cog_acc = hrp::Vector3::Zero();
    double omega = std::sqrt(DEFAULT_GRAVITATIONAL_ACCELERATION / 1.0);

    CogCalculationType calculation_type = PREVIEW_CONTROL;
    std::unique_ptr<ExtendedPreviewController> preview_controller;

    // Foot guided run variables
  public:
    COGTrajectoryGenerator(const hrp::Vector3& init_cog,
                           const CogCalculationType type = PREVIEW_CONTROL) :
        cog(init_cog), calculation_type(type), omega(std::sqrt(DEFAULT_GRAVITATIONAL_ACCELERATION / init_cog[2]))
    {}

    COGTrajectoryGenerator(const hrp::Vector3& init_cog,
                           const hrp::Vector3& init_cog_vel,
                           const hrp::Vector3& init_cog_acc,
                           const CogCalculationType type = PREVIEW_CONTROL) :
        cog(init_cog), cog_vel(init_cog_vel), cog_acc(init_cog_acc), calculation_type(type), omega(std::sqrt(DEFAULT_GRAVITATIONAL_ACCELERATION / init_cog[2]))
    {}

    const hrp::Vector3& getCog()    const { return cog; }
    const hrp::Vector3& getCogVel() const { return cog_vel; }
    const hrp::Vector3& getCogAcc() const { return cog_acc; }
    hrp::Vector3 calcCP() const { return cog + cog_vel / omega; }

    void setCogCalculationType(const CogCalculationType type) { calculation_type = type; }
    void setOmega(const double cog_z, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION)
    {
        omega = std::sqrt(g_acc / cog_z);
    }
    void initPreviewController(const double dt, const hrp::Vector3& cur_ref_zmp);

    void calcCogFromZMP(const std::deque<hrp::Vector3>& refzmp_list, const double dt);

    /**
     * @fn
     * @return reference zmp == (0, 0, 0)^T
     */
    hrp::Vector3 calcCogForFlightPhase(const double dt, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);

    /**
     * @fn
     * @return reference zmp
     */
    hrp::Vector3 calcCogForRun(const hrp::Vector3& support_point,
                               const hrp::Vector3& landing_point,
                               const hrp::Vector3& start_zmp_offset,
                               const hrp::Vector3& end_zmp_offset,
                               const hrp::Vector3& target_cp_offset,
                               const double take_off_z,
                               const double jump_height,
                               const size_t start_count,
                               const size_t supporting_count,
                               const size_t landing_count,
                               const size_t cur_count,
                               const double dt,
                               const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    /**
     * @fn
     * @return reference zmp
     */
    hrp::Vector3 calcCogForRunFromLandingPoints(const hrp::Vector3& support_point,
                                                const hrp::Vector3& landing_points,
                                                const hrp::Vector3& start_zmp_offset,
                                                const hrp::Vector3& end_zmp_offset,
                                                const hrp::Vector3& target_cp_offset,
                                                const double jump_height,
                                                const size_t start_count,
                                                const size_t supporting_count,
                                                const size_t landing_count,
                                                const size_t cur_count,
                                                const double dt,
                                                const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);

    void calcCogZForJump(const size_t count_to_jump,
                         const double jump_height,
                         const double take_off_z,
                         const double dt,
                         const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
};

}

#endif // COGTRAJECTORYGENERATOR_H
