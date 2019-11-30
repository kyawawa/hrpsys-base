// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  LimbTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef LIMBTRAJECTORYGENERATOR_H
#define LIMBTRAJECTORYGENERATOR_H

#include <vector>
#include <hrpUtil/EigenTypes.h>

namespace hrp {

class LimbTrajectoryGenerator
{
  public:
    enum TrajectoryType : size_t {LINEAR, DELAY_HOFFARBIB, CYCLOIDDELAY, RECTANGLE};
    struct ViaPoint
    {
        hrp::Vector3 point;
        double diff_rot_angle; // TODO: Rotation Matrixではなくて問題ないか
        size_t count;
    };

  private:
    TrajectoryType traj_type = RECTANGLE;
    std::vector<ViaPoint> via_points;

    hrp::Matrix33 start_rot = hrp::Matrix33::Identity();
    Eigen::AngleAxisd diff_rot = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

    // Params for delay Hoff & Arbib
    double delay_time_offset = 0.2; // [s]

    std::tuple<size_t, double, hrp::Vector3, double> calcTarget(const size_t count, const double dt);
    void calcLinearTrajectory(const size_t count, const double dt,
                              Eigen::Ref<Eigen::Vector3d> pos, Eigen::Ref<Eigen::Vector3d> vel,
                              Eigen::Ref<Eigen::Vector3d> acc,
                              Eigen::Ref<Eigen::Matrix3d> rot, double& rot_vel,
                              double& rot_acc);
    void calcDelayHoffArbibTrajectory(const size_t count, const double dt,
                                      Eigen::Ref<Eigen::Vector3d> pos, Eigen::Ref<Eigen::Vector3d> vel,
                                      Eigen::Ref<Eigen::Vector3d> acc,
                                      Eigen::Ref<Eigen::Matrix3d> rot, double& rot_vel,
                                      double& rot_acc);
    // Preset functions to generate via points for delay Hoff & Arbib trajectory
    void calcCycloidDelayViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                   const size_t start_count, const size_t goal_count,
                                   const double height);
    void calcRectangleViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                const size_t start_count, const size_t goal_count,
                                const double height);
    void modifyRectangleViaPoints(const hrp::Vector3& goal,
                                  const size_t current_count,
                                  const size_t new_goal_count,
                                  const double dt);
  public:
    bool isViaPointsEmpty() const { return via_points.empty(); }
    const std::vector<ViaPoint>& getViaPoints() const { return via_points; }
    void clearViaPoints() { via_points.clear(); }

    void calcViaPoints(const TrajectoryType traj_type,
                       const Eigen::Isometry3d& start, const Eigen::Isometry3d& goal,
                       const size_t start_count, const size_t goal_count,
                       const double height);

    void setViaPoints(const TrajectoryType _traj_type,
                      const Eigen::Isometry3d& start,
                      const Eigen::Isometry3d& goal,
                      const std::vector<ViaPoint>& _via_points);

    void calcRotationViaPoints(const TrajectoryType _traj_type,
                               const Eigen::Isometry3d& start,
                               const Eigen::Vector3d& rot_axis,
                               const double rot_angle,
                               const size_t start_count,
                               const size_t goal_count);

    void modifyViaPoints(const Eigen::Isometry3d& current_coord,
                         const Eigen::Isometry3d& new_goal,
                         const size_t current_count,
                         const size_t new_goal_count,
                         const double dt);
    /**
     * @fn
     * Calculate limb trajectory using via points and a current count
     */
    void calcTrajectory(const size_t count, const double dt,
                        Eigen::Ref<Eigen::Vector3d> pos, Eigen::Ref<Eigen::Vector3d> vel,
                        Eigen::Ref<Eigen::Vector3d> acc,
                        Eigen::Ref<Eigen::Matrix3d> rot, double& rot_vel,
                        double& rot_acc);
    void setDelayTimeOffset(const double offset) { delay_time_offset = offset; }
    double getDelayTimeOffset() const { return delay_time_offset; }
};

}

#endif // LIMBTRAJECTORYGENERATOR_H
