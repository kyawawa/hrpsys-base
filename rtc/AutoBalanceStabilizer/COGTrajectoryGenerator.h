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
    enum LocomotionMode : size_t { WALK, RUN };
    enum CogCalculationType : size_t { PREVIEW_CONTROL, FOOT_GUIDED };

  private:
    hrp::Vector3 cog     = hrp::Vector3::Zero();
    hrp::Vector3 cog_vel = hrp::Vector3::Zero();
    hrp::Vector3 cog_acc = hrp::Vector3::Zero();

    CogCalculationType calculation_type = PREVIEW_CONTROL;
    std::unique_ptr<ExtendedPreviewController> preview_controller;

  public:
    COGTrajectoryGenerator(const hrp::Vector3& init_cog,
                           const CogCalculationType type = PREVIEW_CONTROL) :
        cog(init_cog), calculation_type(type)
    {}

    COGTrajectoryGenerator(const hrp::Vector3& init_cog,
                           const hrp::Vector3& init_cog_vel,
                           const hrp::Vector3& init_cog_acc,
                           const CogCalculationType type = PREVIEW_CONTROL) :
        cog(init_cog), cog_vel(init_cog_vel), cog_acc(init_cog_acc), calculation_type(type)
    {}

    const hrp::Vector3& getCog()    const { return cog; }
    const hrp::Vector3& getCogVel() const { return cog_vel; }
    const hrp::Vector3& getCogAcc() const { return cog_acc; }

    void setCogCalculationType(CogCalculationType type) { calculation_type = type; }
    void initPreviewController(const double dt, const hrp::Vector3& cur_ref_zmp);
    void calcCogFromLandingPoints(const hrp::Vector3& support_point,
                                  const hrp::Vector3& landing_points,
                                  const double dt,
                                  const double start_time,
                                  const double supporting_time,
                                  const double landing_times,
                                  const double cur_time);
    void calcCogFromZMP(const std::deque<hrp::Vector3>& refzmp_list);
};

}

#endif // COGTRAJECTORYGENERATOR_H
