// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  GaitGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <boost/make_shared.hpp>
#include <tuple>
#include "GaitGenerator.h"
#include "Utility.h"
#include "EigenUtil.h"

namespace hrp {

GaitGenerator::GaitGenerator(const hrp::BodyPtr& _robot,
                             std::mutex& _mutex,
                             const double _dt,
                             std::vector<LinkConstraint>&& init_constraints,
                             const double preview_time) :
    m_mutex(_mutex),
    root_id(_robot->rootLink()->index),
    preview_window(static_cast<size_t>(std::round(preview_time / _dt))),
    default_single_support_count(static_cast<size_t>(1.0 / _dt)),
    default_double_support_count(static_cast<size_t>(0.25 / _dt)),
    default_toe_support_count(static_cast<size_t>(0.5 / _dt)),
    default_heel_support_count(static_cast<size_t>(0.15 / _dt))
{
    const size_t num_limbs = init_constraints.size();

    // Set initial position as target (need to be done calcForwardKinematics)
    // TODO: ここで良いか
    for (auto& constraint : init_constraints) {
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        constraint.targetPos() = constraint.calcActualTargetPosFromLinkState(link->p, link->R);
        // std::cerr << "link   " << link->name << " pos: " << link->p.transpose() << std::endl;
        // std::cerr << "target " << link->name << " pos: " << constraint.targetPos().transpose() << std::endl;
        // std::cerr << "local  " << link->name << " pos: " << constraint.localPos().transpose() << std::endl;
        constraint.targetRot() = constraint.calcActualTargetRotFromLinkState(link->R);
    }

    constraints_list.resize(1);
    constraints_list[0].constraints = std::move(init_constraints); // TODO: これでmoveできるかテストする
    constraints_list[0].start_count = 0;

    root_coord.translation() = _robot->rootLink()->p;
    root_coord.linear() = _robot->rootLink()->R;

    prev_ref_cog = _robot->calcCM();
    zmp_gen = std::make_unique<RefZMPGenerator>(_dt, preview_window, constraints_list[0]);
    cog_gen = std::make_unique<COGTrajectoryGenerator>(prev_ref_cog);

    cog_gen->initPreviewController(_dt, zmp_gen->getCurrentRefZMP());
}

// TODO: cur_count か メンバ変数のloopかどちらかにする
void GaitGenerator::resetGaitGenerator(const hrp::BodyPtr& _robot,
                                       const size_t cur_count,
                                       const double _dt)
{
    // TODO: ここで中心を0にするようにtargetCoordsを移動するべき？
    //       start autobalancerをしたところ基準で考えたい気持ち
    //       いずれにせよodom resetみたいなのは必要

    constraints_list[0].constraints = constraints_list.back().constraints;
    constraints_list[0].start_count = cur_count;
    constraints_list.resize(1);
    cur_const_idx = 0;

    root_coord.translation() = _robot->rootLink()->p;
    root_coord.linear() = _robot->rootLink()->R;

    prev_ref_cog = _robot->calcCM();

    zmp_gen.reset(new RefZMPGenerator(_dt, preview_window, constraints_list[0]));
    resetCOGTrajectoryGenerator(prev_ref_cog, _dt);
}

void GaitGenerator::forwardTimeStep(const size_t cur_count)
{
    zmp_gen->popAndPushRefZMP(constraints_list, cur_count);
    cur_const_idx = getConstraintIndexFromCount(constraints_list, cur_count);
    ConstraintsWithCount& cur_cwc = constraints_list[cur_const_idx];
    if (cur_cwc.start_count != cur_count || cur_const_idx == 0) return;

    // if (cur_cwc.start_count == cur_count && cur_const_idx > 0) {
    // TODO: 最初のStateを変更しておかないと, ToeHeelの切り替え
    cur_cwc.copyLimbState(constraints_list[cur_const_idx - 1]);

    for (size_t idx = 0; idx < cur_cwc.constraints.size(); ++idx) {
        LinkConstraint& cur_const = cur_cwc.constraints[idx];
        if (cur_const.getConstraintType() == LinkConstraint::FIX || cur_const.getConstraintType() == LinkConstraint::FREE) continue;

        size_t goal_idx = cur_const_idx + 1;
        while (goal_idx < constraints_list.size()) {
            if (constraints_list[goal_idx].constraints[idx].getConstraintType() != LinkConstraint::FLOAT) break;
            ++goal_idx;
        }
        if (goal_idx == constraints_list.size()) continue;

        const ConstraintsWithCount goal_cwc = constraints_list[goal_idx];
        const Eigen::Isometry3d target_coord = goal_cwc.constraints[idx].targetCoord() * goal_cwc.constraints[idx].localCoord().inverse() * cur_const.localCoord(); // TODO: 確認

        if (cur_const.getConstraintType() == LinkConstraint::FLOAT) {
            cur_const.calcLimbViaPoints(default_traj_type,
                                        target_coord,
                                        cur_cwc.start_count,
                                        goal_cwc.start_count,
                                        default_step_height);
        } else if (cur_const.getConstraintType() == LinkConstraint::SLIDE) {
            std::cerr << "[GaitGenerator] SLIDE is not implemented yet." << std::endl;
        } else if (cur_const.getConstraintType() == LinkConstraint::ROTATE) {
            // TODO: calcLimbViaPointsに統合?
            cur_const.calcLimbRotationViaPoints(LimbTrajectoryGenerator::LINEAR,
                                                Eigen::Vector3d::UnitY(),
                                                toe_kick_angle,
                                                cur_cwc.start_count,
                                                goal_cwc.start_count);
        }
    }
    // }
}

void GaitGenerator::calcCogAndLimbTrajectory(const size_t cur_count, const double dt)
{
    cog_gen->calcCogFromZMP(zmp_gen->getRefZMPList());
    constraints_list[cur_const_idx].calcLimbTrajectory(cur_count, dt);

    root_coord.translation() += cog_gen->getCog() - prev_ref_cog;
    // TODO: 手が追加された時やその他の時にも対応できるように
    root_coord.linear() = constraints_list[cur_const_idx].calcCOPRotationFromConstraints(LinkConstraint::FREE);
    prev_ref_cog = cog_gen->getCog();

    return;
}

void GaitGenerator::modifyConstraintsTarget(const size_t cur_count,
                                            const size_t cwc_idx_from_current,
                                            const size_t modif_const_idx,
                                            const Eigen::Isometry3d& modif_mat,
                                            const int modif_count,
                                            const double dt)
{
    size_t modif_cwc_idx = cur_const_idx + cwc_idx_from_current;

    // TODO: 例えば４足歩行だと，
    //       1. 今の遊脚のターゲットをかえる
    //       2. 次の遊脚のターゲットをかえ，他はそのまま...というふうになり，全てを変更したらそれ以降は変更してても良い
    // [0] float fix float fix
    //           変更
    // [1] fix float fix float
    //               変更

    const size_t cwc_size = constraints_list.size();
    while (modif_cwc_idx < cwc_size &&
           constraints_list[modif_cwc_idx].constraints[modif_const_idx].getConstraintType() > LinkConstraint::ROTATE) {
        ++modif_cwc_idx;
    }
    if (modif_cwc_idx == cwc_size) return;

    for (size_t cwc_idx = modif_cwc_idx; cwc_idx < cwc_size; ++cwc_idx) {
        constraints_list[cwc_idx].constraints[modif_const_idx].targetCoord() = modif_mat * constraints_list[cwc_idx].constraints[modif_const_idx].targetCoord();
        constraints_list[cwc_idx].start_count += modif_count;
    }

    // TODO: cur_const_idxがFLOATなら良いが，直接着地のidxを指定しているとこれではだめ
    if (constraints_list[cur_const_idx].constraints[modif_const_idx].getConstraintType() > LinkConstraint::ROTATE) {
        // TODO: regenerate
        const LinkConstraint& landing_constraint = constraints_list[modif_cwc_idx].constraints[modif_const_idx];
        const Eigen::Isometry3d target_coord = landing_constraint.targetCoord() * landing_constraint.localCoord().inverse() * constraints_list[cur_const_idx].constraints[modif_const_idx].localCoord(); // TODO: forwardTimeStepにも同じのがある．関数化

        constraints_list[cur_const_idx].constraints[modif_const_idx]
            .modifyLimbViaPoints(target_coord,
                                 cur_count,
                                 constraints_list[modif_cwc_idx].start_count, // TODO: goal count及びその後のずらし,
                                 dt);
    }

    const size_t const_size = constraints_list[modif_cwc_idx].constraints.size();
    for (size_t i = 0; i < const_size; ++i) {
        if (i == modif_const_idx) continue;

        size_t cur_modif_cwc_idx = modif_cwc_idx;
        while (cur_modif_cwc_idx < cwc_size &&
               constraints_list[cur_modif_cwc_idx].constraints[i].getConstraintType() <= LinkConstraint::ROTATE) {
            ++cur_modif_cwc_idx;
        }
        while (cur_modif_cwc_idx < cwc_size &&
               constraints_list[cur_modif_cwc_idx].constraints[i].getConstraintType() > LinkConstraint::ROTATE) {
            ++cur_modif_cwc_idx;
        }

        for (size_t j = cur_modif_cwc_idx; j < cwc_size; ++j) {
            constraints_list[j].constraints[i].targetCoord() = modif_mat * constraints_list[j].constraints[i].targetCoord();
        }
    }

    setRefZMPList(cur_count);
    // setRefZMPList(cur_count, constraints_list[cur_const_idx + cwc_idx_from_current].start_count - cur_count); // TODO
}

// This function is almost the same as ConstraintsWithCount.calcCOPFromConstraints()
hrp::Vector3 GaitGenerator::calcReferenceCOPFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const
{
    hrp::Vector3 cop_pos = hrp::Vector3::Zero();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : cur_consts) {
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT) continue;
        const double weight = constraint.getCOPWeight();
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        cop_pos += constraint.calcActualTargetPosFromLinkState(link->p, link->R) * weight;
        sum_weight += weight;
    }
    if (sum_weight > 0) cop_pos /= sum_weight;

    return cop_pos;
}

// This function is almost the same as ConstraintsWithCount.calcCOPRotationFromConstraints()
hrp::Matrix33 GaitGenerator::calcReferenceCOPRotFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const
{
    Eigen::Quaternion<double> cop_quat = Eigen::Quaternion<double>::Identity();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : cur_consts) {
        const double weight = constraint.getCOPWeight();
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT || weight == 0 /* to avoid zero division */) continue;
        sum_weight += weight;
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        const Eigen::Quaternion<double> contact_quat(constraint.calcActualTargetRotFromLinkState(link->R));
        cop_quat = cop_quat.slerp(weight / sum_weight, contact_quat);
    }

    return cop_quat.toRotationMatrix();
}

void GaitGenerator::adjustCOPCoordToTarget(const hrp::BodyPtr& _robot, const size_t count)
{
    const ConstraintsWithCount& cur_consts = getCurrentConstraints(count);
    const hrp::Vector3 cop_pos = calcReferenceCOPFromModel(_robot, cur_consts.constraints);
    const hrp::Matrix33 cop_rot = calcReferenceCOPRotFromModel(_robot, cur_consts.constraints);

    const hrp::Vector3 target_cop_pos = cur_consts.calcCOPFromConstraints();
    const hrp::Matrix33 target_cop_rot = cur_consts.calcCOPRotationFromConstraints();

    rootPos() = (target_cop_pos - cop_pos) + _robot->rootLink()->p;
    // rootRot() = target_cop_rot * cop_rot.transpose() * _robot->rootLink()->R;
    // TODO: rollとpitchの修正を0にする．これで良いのか
    hrp::Matrix33 diff_rot = target_cop_rot * cop_rot.transpose();
    hrp::Vector3 diff_rpy = hrp::rpyFromRot(diff_rot);
    diff_rpy[0] = diff_rpy[1] = 0.0;
    diff_rot = hrp::rotFromRpy(diff_rpy);
    rootRot() = diff_rot * _robot->rootLink()->R;
}

std::vector<ConstraintsWithCount>
GaitGenerator::calcFootStepConstraints(const ConstraintsWithCount& last_constraints,
                                       const std::vector<size_t>& swing_indices,
                                       const std::vector<Eigen::Isometry3d>& targets,
                                       const size_t swing_start_count,
                                       const size_t one_step_count,
                                       const bool use_toe_heel,
                                       const std::vector<size_t>& toe_support_indices,
                                       const size_t toe_support_count,
                                       const size_t heel_support_count)
{
    // TODO: Toe-heel
    //       float -> support toe -> landing heel -> landing plane -> float ...
    //       landingのheelがsupportのtoeより後ろの場合，toe-heelの意味ない? <- 平地はそうっぽいが，階段とか
    //       最初と最後だけ少し違う
    //       つま先関節の有無で追加の仕方が違いそう
    //       XやZ方向のthresholdでtoeのON/OFFを切り替える？
    //       最初のswing toeの時，重み0かも

    std::vector<ConstraintsWithCount> footstep_constraints;
    {
        const size_t num_constraints = use_toe_heel ? 4 : 2;
        footstep_constraints.reserve(num_constraints);
        footstep_constraints.push_back(last_constraints);
    }

    const size_t landing_count = swing_start_count + one_step_count;

    ConstraintsWithCount& swing_phase_constraints = footstep_constraints.back();
    {
        swing_phase_constraints.start_count = swing_start_count;
        swing_phase_constraints.clearLimbViaPoints();
        for (const size_t swing_idx : swing_indices) {
            swing_phase_constraints.constraints[swing_idx].changeDefaultContacts();
            swing_phase_constraints.constraints[swing_idx].setConstraintType(LinkConstraint::FLOAT);
        }
    }

    if (use_toe_heel) {
        const size_t toe_start_count  = swing_start_count + (one_step_count - heel_support_count - toe_support_count);
        const size_t heel_start_count = toe_start_count + toe_support_count;

        // TODO BUG: Toe-Heel両方追加するとZMPが1制御周期で跳ぶ
        if (!toe_support_indices.empty()) {
            footstep_constraints.push_back(footstep_constraints.back());
            ConstraintsWithCount& toe_phase_constraints = footstep_constraints.back();
            toe_phase_constraints.start_count = toe_start_count;
            toe_phase_constraints.clearLimbViaPoints();
            for (const size_t support_idx : toe_support_indices) {
                LinkConstraint& toe_constraint = toe_phase_constraints.constraints[support_idx];
                if (!toe_constraint.hasToeHeelContacts()) continue;
                toe_constraint.changeToeContacts();
                toe_constraint.setConstraintType(LinkConstraint::ROTATE);
            }
        }

        {
            footstep_constraints.push_back(footstep_constraints.back());
            ConstraintsWithCount& heel_phase_constraints = footstep_constraints.back();
            heel_phase_constraints.start_count = heel_start_count;
            heel_phase_constraints.clearLimbViaPoints();

            // TODO: これはtoeに依存しているので，heelがなくても動くように ?
            for (const size_t support_idx : toe_support_indices) {
                heel_phase_constraints.constraints[support_idx].targetRot() = Eigen::AngleAxisd(toe_kick_angle, Eigen::Vector3d::UnitY()).toRotationMatrix() * heel_phase_constraints.constraints[support_idx].targetRot();
                heel_phase_constraints.constraints[support_idx].setConstraintType(LinkConstraint::FIX);
            }

            const size_t swing_indices_size = swing_indices.size();
            for (size_t i = 0; i < swing_indices_size; ++i) {
                LinkConstraint& heel_constraint = heel_phase_constraints.constraints[swing_indices[i]];
                if (!heel_constraint.hasToeHeelContacts()) continue;

                // TODO: heel contactの座標で計算して，changeDefaultContactsで良さそう？ <- どのconstraintでやるのか
                // const Eigen::Isometry3d heel_rotated_target =
                //     heel_constraint.calcRotatedTargetAroundHeel(targets[i], Eigen::AngleAxisd(-heel_contact_angle, Eigen::Vector3d::UnitY()));

                heel_constraint.changeDefaultContacts();
                heel_constraint.targetCoord() = targets[i];
                heel_constraint.changeHeelContacts();
                heel_constraint.targetRot() = heel_constraint.targetRot() * Eigen::AngleAxisd(-heel_contact_angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
                heel_constraint.setConstraintType(LinkConstraint::ROTATE);
            }
        }
    }

    {
        footstep_constraints.push_back(footstep_constraints.back());
        ConstraintsWithCount& landing_phase_constraints = footstep_constraints.back();
        landing_phase_constraints.start_count = landing_count;
        landing_phase_constraints.clearLimbViaPoints();

        const size_t swing_indices_size = swing_indices.size();
        for (size_t i = 0; i < swing_indices_size; ++i) {
            LinkConstraint& landing_constraint = landing_phase_constraints.constraints[swing_indices[i]];
            landing_constraint.changeDefaultContacts();
            landing_constraint.targetCoord() = targets[i];
            landing_constraint.setConstraintType(LinkConstraint::FIX);
        }
    }

    return footstep_constraints;
}

void GaitGenerator::addFootStep(const ConstraintsWithCount& last_constraints,
                                const std::vector<size_t>& swing_indices,
                                const std::vector<Eigen::Isometry3d>& targets,
                                const size_t swing_start_count,
                                const size_t one_step_count,
                                const bool use_toe_heel,
                                const std::vector<size_t>& toe_support_indices,
                                const size_t toe_support_count,
                                const size_t heel_support_count)
{
    addConstraintsList(calcFootStepConstraints(last_constraints,
                                               swing_indices,
                                               targets,
                                               swing_start_count,
                                               one_step_count,
                                               use_toe_heel,
                                               toe_support_indices,
                                               toe_support_count,
                                               heel_support_count));
}

// TODO: 名前をFootStepではなくする
// Constraintsの存在チェックバージョン．
void GaitGenerator::addFootStep(const std::vector<int>& swing_link_ids, const std::vector<Eigen::Isometry3d>& targets,
                                const size_t swing_start_count, const size_t one_step_count)
{
    // TODO: Last決め打ちで良さそう
    const size_t cur_idx = getConstraintIndexFromCount(constraints_list, swing_start_count - 1);

    std::vector<size_t> swing_indices;
    std::vector<Eigen::Isometry3d> swing_targets;
    swing_indices.reserve(swing_link_ids.size());
    swing_targets.reserve(swing_link_ids.size());
    for (size_t i = 0; i < swing_link_ids.size(); ++i) {
        const int link_id = constraints_list[cur_idx].getConstraintIndexFromLinkId(swing_link_ids[i]);
        if (link_id == -1) continue;
        swing_indices.push_back(swing_link_ids[i]);
        swing_targets.push_back(targets[i]);
    }

    if (swing_indices.empty()) {
        std::cerr << "[GaitGenerator] Cannot find constraints" << std::endl; // TODO: error message
        return;
    }

    addFootStep(constraints_list[cur_idx], swing_indices, swing_targets, swing_start_count, one_step_count);
}

// TODO: foot stepの生成はm_robotを持っている上位に移すべきか
//       Use addFootStep (swing_indicesを与える関数を作ると最初の処理を省略できる)
void GaitGenerator::addFootStepFromVelocity(const double vel_x, const double vel_y, const double vel_yaw,
                                            const size_t swing_start_count, const double step_time, const double dt,
                                            const double max_step_length, const double max_rotate_angle /*[rad]*/,
                                            const int support_id, const int swing_id)
{
    const size_t cur_idx     = getConstraintIndexFromCount(constraints_list, swing_start_count - 1);
    const size_t support_idx = constraints_list[cur_idx].getConstraintIndexFromLinkId(support_id);
    const size_t swing_idx   = constraints_list[cur_idx].getConstraintIndexFromLinkId(swing_id);

    if (support_idx == -1 || swing_idx == -1) {
        std::cerr << "[GaitGenerator] Cannot find constraint " << support_id << "or constraint " << swing_id << std::endl; // TODO: error message
        return;
    }

    ConstraintsWithCount& current_constraints = constraints_list[cur_idx];
    LinkConstraint& support = current_constraints.constraints[support_idx];
    LinkConstraint& landing = current_constraints.constraints[swing_idx];

    // Support limb coordinate
    // TODO: ここあやしい
    double stride_x = vel_x / step_time;
    double stride_y = vel_y / step_time + (support.targetRot().transpose() * (landing.targetPos() - support.targetPos()))[1];
    double yaw_angle = vel_yaw / step_time;

    // TODO: stride limitation

    Eigen::Isometry3d trans_sup_local;
    trans_sup_local.translation() = Eigen::Vector3d(stride_x, stride_y, 0);
    trans_sup_local.linear() = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    std::vector<Eigen::Isometry3d> swing_target;
    swing_target.reserve(1);
    swing_target.push_back(trans_sup_local * support.targetCoord());

    std::vector<size_t> swing_idx_vec;
    swing_idx_vec.reserve(1);
    swing_idx_vec.push_back(swing_idx);

    addFootStep(current_constraints, swing_idx_vec, swing_target, swing_start_count, static_cast<size_t>(step_time / dt));
}

std::vector<int> GaitGenerator::getConstraintLinkIds()
{
    ConstraintsWithCount& last_constraints = constraints_list.back();
    std::vector<int> ids;
    ids.reserve(last_constraints.constraints.size());
    for (const LinkConstraint& constraint : last_constraints.constraints) {
        ids.push_back(constraint.getLinkId());
    }

    return ids;
}

bool GaitGenerator::setToeContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points)
{
    ConstraintsWithCount& last_constraints = constraints_list.back();
    const int index = last_constraints.getConstraintIndexFromLinkId(link_id);
    if (index == -1) return false;

    last_constraints.constraints[index].setToeContactPoints(contact_points);
    return true;
}

bool GaitGenerator::setHeelContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points)
{
    ConstraintsWithCount& last_constraints = constraints_list.back();
    const int index = last_constraints.getConstraintIndexFromLinkId(link_id);
    if (index == -1) return false;

    last_constraints.constraints[index].setHeelContactPoints(contact_points);
    return true;
}

// Rot は world base rotを使う ?
// 元の実装は，支持脚座標を入れ替えながらwhileで進んでいく
bool GaitGenerator::goPos(const Eigen::Isometry3d& target,
                          const std::vector<int>& support_link_cycle,
                          const std::vector<int>& swing_link_cycle)
{
    std::vector<ConstraintsWithCount> new_constraints;

    size_t cur_cycle = 0;
    const size_t cycle_size = support_link_cycle.size(); // TODO: エラーチェック support == swing
    hrp::Vector3 dp;
    Eigen::AngleAxisd dr;
    Eigen::Isometry3d sup_to_swing_trans;
    std::vector<Eigen::Isometry3d> translationFromLimbToCOP;

    const auto getSupportSwingIndex = [&](int& support_idx, int& swing_idx,
                                          const ConstraintsWithCount& constraints,
                                          const size_t _cur_cycle)
        -> bool {
        support_idx = constraints.getConstraintIndexFromLinkId(support_link_cycle[_cur_cycle]);
        swing_idx   = constraints.getConstraintIndexFromLinkId(swing_link_cycle[_cur_cycle]);
        if (support_idx == -1 || swing_idx == -1) {
            std::cerr << "Error" << std::endl; // TODO: error message
            return false;
        }
        return true;
    };

    const auto calcdpAnddr = [&dp, &dr, &target, &translationFromLimbToCOP](const Eigen::Isometry3d& support_coord,
                                                                            const int limb_idx) {
        const Eigen::Isometry3d diff_coord = (support_coord * translationFromLimbToCOP[limb_idx]).inverse() * target;
        dp = diff_coord.translation();
        dr = Eigen::AngleAxisd(diff_coord.linear());
    };

    {
        ConstraintsWithCount cur_constraints = getCurrentConstraints(loop);
        cur_constraints.start_count = loop;

        int support_idx, swing_idx;
        if (!getSupportSwingIndex(support_idx, swing_idx, cur_constraints, cur_cycle)) return false;

        translationFromLimbToCOP.reserve(cur_constraints.constraints.size());
        const Eigen::Isometry3d init_cop = cur_constraints.calcCOPCoord();
        for (const auto& constraint : cur_constraints.constraints) {
            translationFromLimbToCOP.push_back(constraint.targetCoord().inverse() * init_cop);
        }

        const Eigen::Isometry3d& support_target = cur_constraints.constraints[support_idx].targetCoord();
        calcdpAnddr(support_target, support_idx);

        const Eigen::Isometry3d& swing_target = cur_constraints.constraints[swing_idx].targetCoord();
        sup_to_swing_trans = support_target.inverse() * swing_target;

        const size_t min_num_steps = 1 + std::max(static_cast<size_t>(std::ceil(dp.head<2>().norm() / max_stride)),
                                                  static_cast<size_t>(std::ceil(dr.angle() / max_rot_angle)));
        new_constraints.reserve(min_num_steps + 3);
        new_constraints.push_back(cur_constraints);
    }

    const auto notdpZero = [&dp]() -> bool { return !eps_eq(dp.head<2>().squaredNorm(), 0.0, 1e-3 * 0.1); };
    const auto notdrZero = [&dr]() -> bool { return !eps_eq(dr.angle(), 0.0, deg2rad(0.5)); };

    const auto addNewFootSteps = [&](const ConstraintsWithCount& last_constraints, const size_t swing_idx,
                                     const size_t support_idx, const Eigen::Isometry3d& landing_target,
                                     const bool use_toe_heel) {
        // Take longer time for first moving
        const size_t swing_start_count = last_constraints.start_count +
        (new_constraints.size() == 1 ?
         (default_single_support_count + default_double_support_count) : default_double_support_count);
        const std::vector<size_t> swing_indices{swing_idx};
        const std::vector<Eigen::Isometry3d> landing_targets{landing_target};
        const std::vector<size_t> toe_support_indices{support_idx};

        const std::vector<ConstraintsWithCount> footstep_constraints =
        calcFootStepConstraints(last_constraints, swing_indices, landing_targets,
                                swing_start_count, default_single_support_count,
                                use_toe_heel, toe_support_indices,
                                default_toe_support_count, default_heel_support_count);

        for (const auto& footstep : footstep_constraints) {
            new_constraints.push_back(footstep);
        }
    };

    const double squared_max_stride = max_stride * max_stride;

    while (notdrZero() || notdpZero()) {
        const ConstraintsWithCount& last_constraints = new_constraints.back();

        int support_idx, swing_idx;
        if (!getSupportSwingIndex(support_idx, swing_idx, last_constraints, cur_cycle)) return false;

        if (notdpZero()) {
            std::cerr << "dp: " << dp.transpose() << std::endl;
            dp = dp.squaredNorm() > squared_max_stride ? safeNormalize(dp) * max_stride : dp;
        } else dp.setZero();

        if (notdrZero()) {
            dr.angle() = std::min(dr.angle(), max_rot_angle);
            std::cerr << "dr angle: " << dr.angle() << std::endl;
        } else dr.angle() = 0;

        // TODO: goPos(1, 0, 90)のときうまくいかない
        Eigen::Isometry3d landing_target = last_constraints.constraints[support_idx].targetCoord() * sup_to_swing_trans;
        Eigen::Isometry3d rotate_around_cop = translationFromLimbToCOP[swing_idx].inverse();
        rotate_around_cop.prerotate(dr);
        landing_target = landing_target * translationFromLimbToCOP[swing_idx] * rotate_around_cop; // TODO
        landing_target.translation() += dp;

        addNewFootSteps(last_constraints, swing_idx, support_idx, landing_target, default_use_toe_heel);

        // Update to next step
        calcdpAnddr(landing_target, swing_idx);
        sup_to_swing_trans = sup_to_swing_trans.inverse(); // TODO: biped
        cur_cycle = (cur_cycle + 1) % cycle_size;
    }

    // Finalize, TODO: biped only
    const ConstraintsWithCount& last_constraints = new_constraints.back();
    int support_idx, swing_idx;
    if (!getSupportSwingIndex(support_idx, swing_idx, last_constraints, cur_cycle)) return false;
    const Eigen::Isometry3d landing_target = last_constraints.constraints[support_idx].targetCoord() * sup_to_swing_trans;

    addNewFootSteps(last_constraints, swing_idx, support_idx, landing_target, false);

    // Update constraints_list
    std::lock_guard<std::mutex> lock(m_mutex);
    const size_t diff_count = loop - new_constraints[0].start_count + 1;
    for (auto& constraints : new_constraints) {
        constraints.start_count += diff_count;
    }
    setConstraintsList(std::move(new_constraints));
    setRefZMPList(loop);

    return true;
}

}
