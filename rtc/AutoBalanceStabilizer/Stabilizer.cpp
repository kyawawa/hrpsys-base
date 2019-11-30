// -*- C++ -*-
/*!
 * @file  Stabilizer.cpp
 * @brief stabilizer filter
 * @date  $Date$
 *
 * $Id$
 */

#include <boost/make_shared.hpp>
#include <rtm/RTC.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "Stabilizer.h"

// Utility functions
using hrp::deg2rad;
using hrp::rad2deg;
using hrp::calcInteriorPoint;
using hrp::clamp;
using hrp::copyJointAnglesToRobotModel;
using hrp::copyJointAnglesFromRobotModel;

namespace {
constexpr unsigned int DEBUG_LEVEL = 0;
inline bool DEBUGP(unsigned int loop) { return (DEBUG_LEVEL == 1 && loop % 200 == 0) || DEBUG_LEVEL > 1; }
}

namespace hrp {

Stabilizer::Stabilizer(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt)
    : //m_robot(boost::make_shared<hrp::Body>(*_robot)), // TODO: copy constructor
      m_robot(_robot),
      comp_name(_comp_name),
      dt(_dt)
{
}

void Stabilizer::initStabilizer(const RTC::Properties& prop, const size_t ee_num)
{
    // parameters for TPCC
    act_zmp = hrp::Vector3::Zero();

    for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        hrp::Link* root = m_robot->link(ikp.target_name);
        if (ikp.ee_name.find("leg") == std::string::npos) { // Arm default
            ikp.eefm_ee_forcemoment_distribution_weight = hrp::dvector6::Zero();
        } else { // Leg default
            for (size_t j = 0; j < 3; j++) {
                ikp.eefm_ee_forcemoment_distribution_weight[j] = 1.0; // Force
                ikp.eefm_ee_forcemoment_distribution_weight[j+3] = 1e-2; // Moment
            }
        }
        while (!root->isRoot()) {
            ikp.max_limb_length += root->b.norm();
            ikp.parent_name = root->name;
            root = root->parent;
        }

        // Set low pass filter (50 [Hz])
        ikp.target_ee_diff_p_filter = std::make_shared<FirstOrderLowPassFilter<hrp::Vector3>>(50.0, dt, hrp::Vector3::Zero());
        ikp.target_ee_diff_r_filter = std::make_shared<FirstOrderLowPassFilter<hrp::Vector3>>(50.0, dt, hrp::Vector3::Zero());
    }

    // TODO: 初期化方法の見直し
    //eefm_leg_inside_margin = 0.065; // [m]
    //eefm_leg_front_margin = 0.05;
    //eefm_leg_rear_margin = 0.05;
    //fm_wrench_alpha_blending = 1.0; // fz_alpha
    cp_offset = hrp::Vector3(0.0, 0.0, 0.0); // [m]
    contact_decision_threshold = 50; // [N]
    initial_cp_too_large_error = true;
    is_walking = false;
    is_estop_while_walking = false;
    sbp_cog_offset = hrp::Vector3(0.0, 0.0, 0.0);
    use_limb_stretch_avoidance = false;
    use_zmp_truncation = false;
    limb_stretch_avoidance_time_const = 1.5;
    limb_stretch_avoidance_vlimit[0] = -100 * 1e-3 * dt; // lower limit
    limb_stretch_avoidance_vlimit[1] = 50 * 1e-3 * dt; // upper limit
    root_rot_compensation_limit[0] = root_rot_compensation_limit[1] = deg2rad(90.0);
    detection_count_to_mode_air = static_cast<int>(0.0 / dt);

    // Check is legged robot or not
    is_legged_robot = false;
    for (size_t i = 0; i < stikp.size(); i++) {
        if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
        hrp::Sensor* sen= m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
        if (sen != NULL) is_legged_robot = true;
    }
    is_emergency = false;
    reset_emergency_flag = false;
    whether_send_emergency_signal = false;

    // m_tau.data.length(m_robot->numJoints());
    transition_joint_q = hrp::dvector::Zero(m_robot->numJoints());
    qorg = hrp::dvector::Zero(m_robot->numJoints());
    qrefv = hrp::dvector::Zero(m_robot->numJoints());
    transition_count = 0;
    m_is_falling_counter = 0;
    total_mass = m_robot->totalMass();
    ref_zmp_aux = hrp::Vector3::Zero();

    control_swing_support_time.resize(ee_num, 1.0);
    for (size_t i = 0; i < ee_num; i++) {
        target_ee_p.push_back(hrp::Vector3::Zero());
        target_ee_R.push_back(hrp::Matrix33::Identity());
        act_ee_p.push_back(hrp::Vector3::Zero());
        act_ee_R.push_back(hrp::Matrix33::Identity());
        projected_normal.push_back(hrp::Vector3::Zero());
        act_force.push_back(hrp::Vector3::Zero());
        ref_force.push_back(hrp::Vector3::Zero());
        ref_moment.push_back(hrp::Vector3::Zero());
        prev_act_force_z.push_back(0.0);
        ref_contact_states.push_back(true);
        prev_ref_contact_states.push_back(true);
        // m_actContactStates.data[i] = false;
        act_contact_states.push_back(false);
        toe_heel_ratio.push_back(1.0);
        contact_cop_info.push_back(hrp::Vector3::Zero());
        wrenches.push_back(hrp::dvector6::Zero());

        const bool is_ee_leg = stikp[i].ee_name.find("leg") != std::string::npos ? true : false;
        // Hands ik => disabled, feet ik => enabled, by default
        is_ik_enable.push_back(is_ee_leg);
        // Hands feedback control => disabled, feet feedback control => enabled, by default
        is_feedback_control_enable.push_back(is_ee_leg);
        // To zmp calculation, hands are disabled and feet are enabled, by default
        is_zmp_calc_enable.push_back(is_ee_leg);

        jpe_v.push_back(std::make_shared<hrp::JointPathEx>(m_robot,
                                                           m_robot->link(stikp[i].ee_base),
                                                           m_robot->link(stikp[i].target_name),
                                                           dt, false, comp_name));
        // Fix for toe joint
        const size_t num_joints = jpe_v.back()->numJoints();
        if (stikp[i].ee_name.find("leg") != std::string::npos && num_joints == 7) { // leg and has 7dof joint (6dof leg +1dof toe)
            std::vector<double> optw;
            for (size_t j = 0; j < num_joints; ++j) {
                if (j == num_joints - 1) optw.push_back(0.0);
                else optw.push_back(1.0);
            }
            jpe_v.back()->setOptionalWeightVector(optw);
        }

        contact_states_index_map.insert(std::make_pair(stikp[i].ee_name, i));
    }

    {
        std::vector<std::pair<hrp::Link*, hrp::Link*>> interlocking_joints;
        readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], comp_name);
        if (interlocking_joints.size() > 0) {
            for (size_t i = 0; i < jpe_v.size(); i++) {
                std::cerr << "[" << comp_name << "] Interlocking Joints for [" << stikp[i].ee_name << "]" << std::endl;
                jpe_v[i]->setInterlockingJointPairIndices(interlocking_joints, comp_name);
            }
        }
    }

    foot_origin_offset[0] = hrp::Vector3::Zero();
    foot_origin_offset[1] = hrp::Vector3::Zero();

    act_cogvel_filter = std::make_unique<FirstOrderLowPassFilter<hrp::Vector3>>(4.0, dt, hrp::Vector3::Zero()); // 4.0 Hz ?

    szd = std::make_unique<SimpleZMPDistributor>(dt);
    std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
    for (size_t i = 0; i < stikp.size(); i++) {
        support_polygon_vec.push_back(std::vector<Eigen::Vector2d>(1, Eigen::Vector2d::Zero()));
    }
    szd->set_vertices(support_polygon_vec);

    rel_ee_pos.reserve(stikp.size());
    rel_ee_rot.reserve(stikp.size());
    rel_ee_name.reserve(stikp.size());

    const hrp::Sensor* const sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (!sen) {
        std::cerr << "[" << comp_name << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
    }
}

// TODO: 右辺値？
void Stabilizer::execStabilizer(const paramsFromAutoBalancer& abc_param,
                                const paramsFromSensors& sensor_param)
{
    loop++; // TODO: remove?
    if (!is_legged_robot) return;

    calcTargetParameters(abc_param);
    calcActualParameters(sensor_param);
    calcStateForEmergencySignal();

    switch (control_mode) {
      case MODE_IDLE:
          break;
      case MODE_AIR:
          if (transition_count == 0 && on_ground) syncToSt();
          break;
      case MODE_ST:
          if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
              calcEEForceMomentControl();
          } else {
              calcTPCC();
          }
          if (transition_count == 0 && !on_ground) {
              if (is_air_counter < detection_count_to_mode_air) ++is_air_counter;
              else control_mode = MODE_SYNC_TO_AIR;
          } else is_air_counter = 0;
          break;
      case MODE_SYNC_TO_IDLE:
          syncToIdle();
          control_mode = MODE_IDLE;
          break;
      case MODE_SYNC_TO_AIR:
          syncToIdle();
          control_mode = MODE_AIR;
          break;
    }

    storeCurrentStates();
}

void Stabilizer::calcTargetParameters(const paramsFromAutoBalancer& abc_param)
{
    // Ref: qRef, zmpRef, baseRpy, basePos, control_swing_support_time, ref_wrenches
    // m_COPInfo ?
    // Act: rpy, wrenches,

    ref_contact_states = abc_param.ref_contact_states;
    // toe_heel_ratio = abc_param.toe_heel_ratio;
    control_swing_support_time = abc_param.control_swing_support_time;
    is_walking = abc_param.is_walking;
    sbp_cog_offset = abc_param.sbp_cog_offset;

    // Reference world frame =>
    // update internal robot model

    // TODO: transition_count のif文が多すぎる
    if (transition_count == 0) {
        transition_smooth_gain = 1.0;
    } else {
        const double max_transition_count = calcMaxTransitionCount();
        transition_smooth_gain = 1 / (1 + exp(-9.19 * (((max_transition_count - std::fabs(transition_count)) / max_transition_count) - 0.5))); // TODO: 意味がわからない
    }

    if (transition_count > 0) {
        copyJointAnglesToRobotModel(m_robot, calcInteriorPoint(transition_joint_q, abc_param.q_ref, transition_smooth_gain));
    } else {
        copyJointAnglesToRobotModel(m_robot, abc_param.q_ref);
    }

    if (transition_count < 0) {
        transition_count++;
    } else if (transition_count > 0) {
        if (transition_count == 1) {
            std::cerr << "[" << comp_name << "] [" << "] Move to MODE_IDLE" << std::endl;
            reset_emergency_flag = true;
        }
        transition_count--;
    }

    copyJointAnglesFromRobotModel(qrefv, m_robot);

    target_root_p = abc_param.base_pos_ref;
    target_root_R = hrp::rotFromRpy(abc_param.base_rpy_ref);
    m_robot->rootLink()->p = target_root_p;
    m_robot->rootLink()->R = target_root_R;
    m_robot->calcForwardKinematics();

    ref_zmp = target_root_R * abc_param.zmp_ref + target_root_p; // base frame -> world frame
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        // apply inverse system
        const hrp::Vector3 modified_ref_zmp = ref_zmp + eefm_zmp_delay_time_const[0] * (ref_zmp - prev_ref_zmp) / dt;
        prev_ref_zmp = ref_zmp;
        ref_zmp = modified_ref_zmp;
    }

    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    calcFootOriginCoords(foot_origin_pos, foot_origin_rot);

    // ref_cog = m_robot->calcCM(); // TODO: done: 下に移動した
    ref_total_force = hrp::Vector3::Zero();
    ref_total_moment = hrp::Vector3::Zero(); // Total moment around reference ZMP tmp
    ref_total_foot_origin_moment = hrp::Vector3::Zero();
    for (size_t i = 0; i < stikp.size(); i++) {
        // TODO: なぜかyは0
        // const hrp::Vector3 limb_cop_offset(abc_param.limb_cop_offsets[i][0], 0, abc_param.limb_cop_offsets[i][2]);
        const hrp::Vector3 limb_cop_offset = hrp::Vector3::Zero(); // TODO: tmp
        stikp[i].localCOPPos = stikp[i].localp + stikp[i].localR * limb_cop_offset;
        const hrp::Link* target = m_robot->link(stikp[i].target_name);
        target_ee_p[i] = target->p + target->R * stikp[i].localp;
        target_ee_R[i] = target->R * stikp[i].localR;
        ref_force[i]  = abc_param.wrenches_ref[i].head<3>();
        ref_moment[i] = abc_param.wrenches_ref[i].tail<3>();
        ref_total_force += ref_force[i];
        ref_total_moment += (target_ee_p[i] - ref_zmp).cross(ref_force[i]);
#ifndef FORCE_MOMENT_DIFF_CONTROL
        // Force/moment control
        ref_total_moment += ref_moment[i];
#endif

        if (is_feedback_control_enable[i]) {
            ref_total_foot_origin_moment += (target_ee_p[i]-foot_origin_pos).cross(ref_force[i]) + ref_moment[i];
        }
    }
    // <= Reference world frame

    ref_cog = m_robot->calcCM();

    // TODO: prev_ref_cog と ref_cogの座標系が異なるためref_cogvelが過大になる．現状はsmooth_gainで強引に吹っ飛ばなくしている
    // Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST) because the coordinates for ref_cog differs among st algorithms.
    if (transition_count == (-calcMaxTransitionCount() + 1)) { // max transition count. In MODE_IDLE => MODE_ST, transition_count is < 0 and upcounter. "+ 1" is upcount at the beginning of this function.
        prev_ref_cog = ref_cog;
        std::cerr << "[" << comp_name
                  << "]   Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST)."
                  << std::endl;
    }

    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        // Reference foot_origin frame =>
        // initialize for new_refzmp
        new_refzmp = ref_zmp;
        rel_cog = m_robot->rootLink()->R.transpose() * (ref_cog - m_robot->rootLink()->p);
        // convert world (current-tmp) => local (foot_origin)
        zmp_origin_off = ref_zmp(2) - foot_origin_pos(2);
        ref_zmp = foot_origin_rot.transpose() * (ref_zmp - foot_origin_pos);
        ref_cog = foot_origin_rot.transpose() * (ref_cog - foot_origin_pos);
        new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);

        if (ref_contact_states != prev_ref_contact_states) {
            ref_cogvel = (foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * ref_cogvel;
        } else {
            ref_cogvel = (ref_cog - prev_ref_cog)/dt;
        }
        prev_ref_foot_origin_rot = ref_foot_origin_rot = foot_origin_rot;

        for (size_t i = 0; i < stikp.size(); i++) {
            stikp[i].target_ee_diff_p = foot_origin_rot.transpose() * (target_ee_p[i] - foot_origin_pos);
            stikp[i].target_ee_diff_r = foot_origin_rot.transpose() * target_ee_R[i];
            ref_force[i] = foot_origin_rot.transpose() * ref_force[i];
            ref_moment[i] = foot_origin_rot.transpose() * ref_moment[i];
        }

        ref_total_foot_origin_moment = foot_origin_rot.transpose() * ref_total_foot_origin_moment;
        ref_total_force = foot_origin_rot.transpose() * ref_total_force;
        ref_total_moment = foot_origin_rot.transpose() * ref_total_moment;
        target_foot_origin_rot = foot_origin_rot;

        // capture point
        ref_cp = ref_cog + ref_cogvel / std::sqrt(gravitational_acceleration / (ref_cog - ref_zmp)(2));
        rel_ref_cp = hrp::Vector3(ref_cp(0), ref_cp(1), ref_zmp(2));
        rel_ref_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_ref_cp) - m_robot->rootLink()->p);
        sbp_cog_offset = foot_origin_rot.transpose() * sbp_cog_offset;
        // <= Reference foot_origin frame
    } else {
        ref_cogvel = (ref_cog - prev_ref_cog) / dt;
    } // st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFM

    prev_ref_cog = ref_cog;
    // Calc swing support limb gain param
    calcSwingSupportLimbGain();
}

// TODO: rename
void Stabilizer::calcActualParameters(const paramsFromSensors& sensor_param)
{
    // Actual world frame =>
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        copyJointAnglesToRobotModel(m_robot, sensor_param.q_current);

        // tempolary
        m_robot->rootLink()->p = hrp::Vector3::Zero();
        m_robot->calcForwardKinematics(); // TODO: いらなさそう

        const hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        const hrp::Matrix33 senR = sen->link->R * sen->localR;
        act_Rs = hrp::rotFromRpy(sensor_param.rpy);
        m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
        m_robot->calcForwardKinematics();

        act_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
        calcFootOriginCoords(foot_origin_pos, foot_origin_rot);
    } else {
        // TODO: restore関数, current_root_pという名前も微妙かも
        copyJointAnglesToRobotModel(m_robot, qorg);
        m_robot->rootLink()->p = current_root_p;
        m_robot->rootLink()->R = current_root_R;
        m_robot->calcForwardKinematics();
    }

    std::copy(sensor_param.wrenches.begin(), sensor_param.wrenches.end(), wrenches.begin());

    // cog
    act_cog = m_robot->calcCM();
    // zmp
    on_ground = false;
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        on_ground = calcZMP(act_zmp, zmp_origin_off + foot_origin_pos(2));
    } else {
        on_ground = calcZMP(act_zmp, ref_zmp(2));
    }

    // set actual contact states
    for (size_t i = 0; i < stikp.size(); i++) {
        const size_t idx = contact_states_index_map[stikp[i].ee_name];
        act_contact_states[idx] = isContact(idx);
    }
    // <= Actual world frame

    // convert absolute (in st) -> root-link relative
    rel_act_zmp = m_robot->rootLink()->R.transpose() * (act_zmp - m_robot->rootLink()->p);
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        // Actual foot_origin frame =>
        act_zmp = foot_origin_rot.transpose() * (act_zmp - foot_origin_pos);
        act_cog = foot_origin_rot.transpose() * (act_cog - foot_origin_pos);
        //act_cogvel = foot_origin_rot.transpose() * act_cogvel;
        if (ref_contact_states != prev_ref_contact_states) {
            act_cogvel = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * act_cogvel;
        } else {
            act_cogvel = (act_cog - prev_act_cog)/dt;
        }
        prev_act_foot_origin_rot = foot_origin_rot;
        act_cogvel = act_cogvel_filter->passFilter(act_cogvel);
        prev_act_cog = act_cog;
        //act_root_rot = m_robot->rootLink()->R;
        for (size_t i = 0; i < stikp.size(); i++) {
            const hrp::Link* target = m_robot->link(stikp[i].target_name);
            act_ee_p[i] = target->p + target->R * stikp[i].localp;
            act_ee_p[i] = foot_origin_rot.transpose() * (act_ee_p[i] - foot_origin_pos);
            act_ee_R[i] = foot_origin_rot.transpose() * (target->R * stikp[i].localR);
        }

        // capture point
        act_cp = act_cog + act_cogvel / std::sqrt(gravitational_acceleration / (act_cog - act_zmp)(2));
        rel_act_cp = hrp::Vector3(act_cp(0), act_cp(1), act_zmp(2));
        rel_act_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_act_cp) - m_robot->rootLink()->p);
        // <= Actual foot_origin frame

        // Actual world frame =>
        // new ZMP calculation
        // Kajita's feedback law
        //   Basically Equation (26) in the paper [1].
        const hrp::Vector3 dcog    = foot_origin_rot * (ref_cog - act_cog);
        const hrp::Vector3 dcogvel = foot_origin_rot * (ref_cogvel - act_cogvel);
        const hrp::Vector3 dzmp    = foot_origin_rot * (ref_zmp - act_zmp);
        new_refzmp = foot_origin_rot * new_refzmp + foot_origin_pos;
        for (size_t i = 0; i < 2; i++) {
            new_refzmp(i) += eefm_k1[i] * transition_smooth_gain * dcog(i) + eefm_k2[i] * transition_smooth_gain * dcogvel(i) + eefm_k3[i] * transition_smooth_gain * dzmp(i) + ref_zmp_aux(i);
        }

        // std::cerr << "act_cogvel = " << hrp::Vector3(act_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;

        if (DEBUGP(loop)) {
            // All state variables are foot_origin coords relative
            std::cerr << "[" << comp_name << "] state values" << std::endl;
            std::cerr << "[" << comp_name << "]   "
                      << "ref_cog    = "
                      << hrp::Vector3(ref_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", act_cog    = "
                      << hrp::Vector3(act_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm]" << std::endl;
            std::cerr << "[" << comp_name << "]   "
                      << "ref_cogvel = " << hrp::Vector3(ref_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", act_cogvel = " << hrp::Vector3(act_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm/s]" << std::endl;
            std::cerr << "[" << comp_name << "]   "
                      << "ref_zmp    = " << hrp::Vector3(ref_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", act_zmp    = " << hrp::Vector3(act_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm]" << std::endl;
            const hrp::Vector3 tmpnew_refzmp = foot_origin_rot.transpose() * (new_refzmp-foot_origin_pos); // Actual world -> foot origin relative
            std::cerr << "[" << comp_name << "]   "
                      << "new_zmp    = " << hrp::Vector3(tmpnew_refzmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", dif_zmp    = " << hrp::Vector3((tmpnew_refzmp-ref_zmp)*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm]" << std::endl;
        }

        // TODO: reserve stikp.size() or resize
        std::vector<std::string> ee_name;
        // distribute new ZMP into foot force & moment
        std::vector<hrp::Vector3> tmp_ref_force, tmp_ref_moment;
        std::vector<double> limb_gains;
        std::vector<hrp::dvector6> ee_forcemoment_distribution_weight;
        std::vector<double> tmp_toe_heel_ratio;
        if (control_mode == MODE_ST) {
            std::vector<hrp::Vector3> ee_pos, cop_pos;
            std::vector<hrp::Matrix33> ee_rot;
            std::vector<bool> is_contact_list; // TODO: delete
            is_contact_list.reserve(stikp.size());
            for (size_t i = 0; i < stikp.size(); i++) {
                if (!is_feedback_control_enable[i]) continue;

                const STIKParam& ikp = stikp[i];
                const hrp::Link* target = m_robot->link(ikp.target_name);
                ee_name.push_back(ikp.ee_name);
                ee_pos.push_back(target->p + target->R * ikp.localp);
                cop_pos.push_back(target->p + target->R * ikp.localCOPPos);
                ee_rot.push_back(target->R * ikp.localR);
                limb_gains.push_back(ikp.swing_support_gain);
                tmp_ref_force.push_back(hrp::Vector3(foot_origin_rot * ref_force[i]));
                tmp_ref_moment.push_back(hrp::Vector3(foot_origin_rot * ref_moment[i]));
                rel_ee_pos.push_back(foot_origin_rot.transpose() * (ee_pos.back() - foot_origin_pos));
                rel_ee_rot.push_back(foot_origin_rot.transpose() * ee_rot.back());
                rel_ee_name.push_back(ee_name.back());
                is_contact_list.push_back(act_contact_states[i]);
                // std::cerr << ee_forcemoment_distribution_weight[i] << std::endl;
                ee_forcemoment_distribution_weight.push_back(hrp::dvector6::Zero(6,1));
                for (size_t j = 0; j < 6; j++) {
                    ee_forcemoment_distribution_weight[i][j] = ikp.eefm_ee_forcemoment_distribution_weight[j];
                }
                tmp_toe_heel_ratio.push_back(toe_heel_ratio[i]);
            }

            // All state variables are foot_origin coords relative
            if (DEBUGP(loop)) {
                std::cerr << "[" << comp_name << "] ee values" << std::endl;
                hrp::Vector3 tmpp;
                for (size_t i = 0; i < ee_name.size(); i++) {
                    tmpp = foot_origin_rot.transpose()*(ee_pos[i]-foot_origin_pos);
                    std::cerr << "[" << comp_name << "]   "
                              << "ee_pos (" << ee_name[i] << ")    = "
                              << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"));
                    tmpp = foot_origin_rot.transpose()*(cop_pos[i]-foot_origin_pos);
                    std::cerr << ", cop_pos (" << ee_name[i] << ")    = "
                              << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                              << "[mm]" << std::endl;
                }
            }

            // truncate ZMP
            if (use_zmp_truncation) {
                Eigen::Vector2d new_refzmp_xy(new_refzmp.head(2));
                szd->get_vertices(support_polygon_vetices);
                szd->calc_convex_hull(support_polygon_vetices, ref_contact_states, ee_pos, ee_rot);
                if (!szd->is_inside_support_polygon(new_refzmp_xy, hrp::Vector3::Zero(), true, comp_name))
                    new_refzmp.head(2) = new_refzmp_xy;
            }

            // Distribute ZMP into each EE force/moment at each COP
            if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFM) {
                // Modified version of distribution in Equation (4)-(6) and (10)-(13) in the paper [1].
                szd->distributeZMPToForceMoments(tmp_ref_force, tmp_ref_moment,
                                                 ee_pos, cop_pos, ee_rot, ee_name, limb_gains,
                                                 new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                 gravitational_acceleration * total_mass, dt,
                                                 DEBUGP(loop), comp_name);
            } else if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQP) {
                szd->distributeZMPToForceMomentsQP(tmp_ref_force, tmp_ref_moment,
                                                   ee_pos, cop_pos, ee_rot, ee_name, limb_gains,
                                                   new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                   gravitational_acceleration * total_mass, dt,
                                                   DEBUGP(loop), comp_name,
                                                   (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP));
            } else if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP) {
                szd->distributeZMPToForceMomentsPseudoInverse(tmp_ref_force, tmp_ref_moment,
                                                              ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toe_heel_ratio,
                                                              new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                              gravitational_acceleration * total_mass, dt,
                                                              DEBUGP(loop), comp_name,
                                                              (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP), is_contact_list);
            } else if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP2) {
                szd->distributeZMPToForceMomentsPseudoInverse2(tmp_ref_force, tmp_ref_moment,
                                                               ee_pos, cop_pos, ee_rot, ee_name, limb_gains,
                                                               new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                               foot_origin_rot * ref_total_force, foot_origin_rot * ref_total_moment,
                                                               ee_forcemoment_distribution_weight,
                                                               gravitational_acceleration * total_mass, dt,
                                                               DEBUGP(loop), comp_name);
            }

            // for debug output
            new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
        }

        // foot modif
        if (control_mode == MODE_ST) {
            hrp::Vector3 f_diff(hrp::Vector3::Zero());
            std::vector<bool> large_swing_f_diff(3, false);
            // moment control
            act_total_foot_origin_moment = hrp::Vector3::Zero();
            for (size_t i = 0; i < stikp.size(); i++) {
                if (!is_feedback_control_enable[i]) continue;
                STIKParam& ikp = stikp[i];
                std::vector<bool> large_swing_m_diff(3, false);
                hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(ikp.sensor_name);
                hrp::Link* target = m_robot->link(ikp.target_name);
                // Convert moment at COP => moment at ee
                const size_t idx = contact_states_index_map[ikp.ee_name];
                ikp.ref_moment = tmp_ref_moment[idx] + ((target->R * ikp.localCOPPos + target->p) - (target->R * ikp.localp + target->p)).cross(tmp_ref_force[idx]);
                ikp.ref_force = tmp_ref_force[idx];
                // Actual world frame =>
                hrp::Vector3 sensor_force = (sensor->link->R * sensor->localR) * wrenches[i].head<3>();
                const hrp::Vector3 sensor_moment = (sensor->link->R * sensor->localR) * wrenches[i].tail<3>();
                //hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localCOPPos + target->p)).cross(sensor_force) + sensor_moment;
                hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localp + target->p)).cross(sensor_force) + sensor_moment;
                // <= Actual world frame
                // Convert force & moment as foot origin coords relative
                ikp.ref_moment = foot_origin_rot.transpose() * ikp.ref_moment;
                ikp.ref_force = foot_origin_rot.transpose() * ikp.ref_force;
                sensor_force = foot_origin_rot.transpose() * sensor_force;
                ee_moment = foot_origin_rot.transpose() * ee_moment;
                if (i == 0) f_diff -= sensor_force;
                else f_diff += sensor_force;
                for (size_t j = 0; j < 3; ++j) {
                    if ((!ref_contact_states[i] || !act_contact_states[i]) && fabs(ikp.ref_force(j) - sensor_force(j)) > eefm_swing_damping_force_thre[j]) large_swing_f_diff[j] = true;
                    if ((!ref_contact_states[i] || !act_contact_states[i]) && fabs(ikp.ref_moment(j) - ee_moment(j)) > eefm_swing_damping_moment_thre[j]) large_swing_m_diff[j] = true;
                }
                // Moment limitation
                const hrp::Matrix33 ee_R(target->R * ikp.localR);
                ikp.ref_moment = ee_R * clamp((ee_R.transpose() * ikp.ref_moment), ikp.eefm_ee_moment_limit);
                // calcDampingControl
                // d_foot_rpy and d_foot_pos is (actual) foot origin coords relative value because these use foot origin coords relative force & moment
                { // Rot
                    //   Basically Equation (16) and (17) in the paper [1]
                    hrp::Vector3 damping_gain; // TODO: rename
                    for (size_t j = 0; j < 3; ++j) {
                        if (!eefm_use_swing_damping || !large_swing_m_diff[j]) damping_gain(j) = (1 - transition_smooth_gain) * ikp.eefm_rot_damping_gain(j) * 10 + transition_smooth_gain * ikp.eefm_rot_damping_gain(j);
                        else damping_gain(j) = (1 - transition_smooth_gain) * eefm_swing_rot_damping_gain(j) * 10 + transition_smooth_gain * eefm_swing_rot_damping_gain(j);
                    }
                    ikp.d_foot_rpy = calcDampingControl(ikp.ref_moment, ee_moment, ikp.d_foot_rpy, damping_gain, ikp.eefm_rot_time_const);
                    ikp.d_foot_rpy = clamp(ikp.d_foot_rpy, -1 * ikp.eefm_rot_compensation_limit, ikp.eefm_rot_compensation_limit);
                }
                if (!eefm_use_force_difference_control) { // Pos
                    const hrp::Vector3 damping_gain = (1 - transition_smooth_gain) * ikp.eefm_pos_damping_gain * 10 + transition_smooth_gain * ikp.eefm_pos_damping_gain; // TODO: rename
                    ikp.d_foot_pos = calcDampingControl(ikp.ref_force, sensor_force, ikp.d_foot_pos, damping_gain, ikp.eefm_pos_time_const_support);
                    ikp.d_foot_pos = clamp(ikp.d_foot_pos, -1 * ikp.eefm_pos_compensation_limit, ikp.eefm_pos_compensation_limit);
                }
                // Actual ee frame =>
                ikp.ee_d_foot_rpy = ee_R.transpose() * (foot_origin_rot * ikp.d_foot_rpy);
                // tilt Check : only flat plane is supported
                {
                    const hrp::Vector3 plane_x = target_ee_R[i].col(0);
                    const hrp::Vector3 plane_y = target_ee_R[i].col(1);
                    const hrp::Matrix33 act_ee_R_world = target->R * stikp[i].localR;
                    const hrp::Vector3 normal_vector = act_ee_R_world.col(2);
                    /* projected_normal = c1 * plane_x + c2 * plane_y : c1 = plane_x.dot(normal_vector), c2 = plane_y.dot(normal_vector) because (normal-vector - projected_normal) is orthogonal to plane */
                    projected_normal.at(i) = plane_x.dot(normal_vector) * plane_x + plane_y.dot(normal_vector) * plane_y;
                    act_force.at(i) = sensor_force;
                }
                act_total_foot_origin_moment += (target->R * ikp.localp + target->p - foot_origin_pos).cross(sensor_force) + ee_moment;
            }
            act_total_foot_origin_moment = foot_origin_rot.transpose() * act_total_foot_origin_moment;

            if (eefm_use_force_difference_control) {
                // fxyz control
                // foot force difference control version
                //   Basically Equation (18) in the paper [1]
                const hrp::Vector3 ref_f_diff = (stikp[1].ref_force - stikp[0].ref_force);

                if (eefm_use_swing_damping) {
                    hrp::Vector3 damping_gain;
                    for (size_t i = 0; i < 3; ++i) {
                        if (!large_swing_f_diff[i]) damping_gain(i) = (1 - transition_smooth_gain) * stikp[0].eefm_pos_damping_gain(i) * 10 +
                                                        transition_smooth_gain * stikp[0].eefm_pos_damping_gain(i);
                        else damping_gain(i) = (1 - transition_smooth_gain) * eefm_swing_pos_damping_gain(i) * 10 +
                                 transition_smooth_gain * eefm_swing_pos_damping_gain(i);
                    }
                    pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                                   damping_gain, stikp[0].eefm_pos_time_const_support);
                } else {
                    if ((ref_contact_states[contact_states_index_map["rleg"]] && ref_contact_states[contact_states_index_map["lleg"]]) // Reference : double support phase
                         || (act_contact_states[0] && act_contact_states[1])) { // Actual : double support phase
                        // Temporarily use first pos damping gain (stikp[0])
                        hrp::Vector3 damping_gain = (1 - transition_smooth_gain) * stikp[0].eefm_pos_damping_gain * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain;
                        pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                                       damping_gain, stikp[0].eefm_pos_time_const_support);
                    } else {
                        double remain_swing_time;
                        if ( !ref_contact_states[contact_states_index_map["rleg"]] ) { // rleg swing
                            remain_swing_time = control_swing_support_time[contact_states_index_map["rleg"]];
                        } else { // lleg swing
                            remain_swing_time = control_swing_support_time[contact_states_index_map["lleg"]];
                        }
                        const double swing_ratio = hrp::clamp(1.0 - (remain_swing_time - eefm_pos_margin_time) / eefm_pos_transition_time, 0.0, 1.0); // 0=>1
                        // Temporarily use first pos damping gain (stikp[0])
                        const hrp::Vector3 damping_gain = (1 - transition_smooth_gain) * stikp[0].eefm_pos_damping_gain * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain;
                        const hrp::Vector3 tmp_time_const = (1 - swing_ratio) * eefm_pos_time_const_swing*hrp::Vector3::Ones() + swing_ratio*stikp[0].eefm_pos_time_const_support;
                        pos_ctrl = calcDampingControl (swing_ratio * ref_f_diff, swing_ratio * f_diff, pos_ctrl, damping_gain, tmp_time_const);
                    }
                }

                // Temporarily use first pos compensation limit (stikp[0])
                pos_ctrl = clamp(pos_ctrl, -1 * stikp[0].eefm_pos_compensation_limit * 2, stikp[0].eefm_pos_compensation_limit * 2);
                stikp[0].d_foot_pos = -0.5 * pos_ctrl; // TODO: use contact index map ?
                stikp[1].d_foot_pos = 0.5 * pos_ctrl;
            }

            if (DEBUGP(loop)) {
                std::cerr << "[" << comp_name << "] Control values" << std::endl;
                if (eefm_use_force_difference_control) {
                    std::cerr << "[" << comp_name << "]   "
                              << "pos_ctrl    = [" << pos_ctrl(0)*1e3 << " " << pos_ctrl(1)*1e3 << " "<< pos_ctrl(2)*1e3 << "] [mm]" << std::endl;
                }
                for (size_t i = 0; i < ee_name.size(); i++) {
                    std::cerr << "[" << comp_name << "]   "
                              << "d_foot_pos (" << ee_name[i] << ")  = [" << stikp[i].d_foot_pos(0)*1e3 << " " << stikp[i].d_foot_pos(1)*1e3 << " " << stikp[i].d_foot_pos(2)*1e3 << "] [mm], "
                              << "d_foot_rpy (" << ee_name[i] << ")  = [" << rad2deg(stikp[i].d_foot_rpy(0)) << " " << rad2deg(stikp[i].d_foot_rpy(1)) << " " << rad2deg(stikp[i].d_foot_rpy(2)) << "] [deg]" << std::endl;
                }
            }

            calcDiffFootOriginExtMoment();
        }
    } // st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC

    copyJointAnglesToRobotModel(m_robot, qrefv);
    m_robot->rootLink()->p = target_root_p;
    m_robot->rootLink()->R = target_root_R;
    if (!(control_mode == MODE_IDLE || control_mode == MODE_AIR)) {
        for (size_t i = 0; i < jpe_v.size(); i++) {
            if (!is_ik_enable[i]) continue;
            for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
                int idx = jpe_v[i]->joint(j)->jointId;
                m_robot->joint(idx)->q = qorg[idx];
            }
        }
        m_robot->rootLink()->p(0) = current_root_p(0);
        m_robot->rootLink()->p(1) = current_root_p(1);
        m_robot->rootLink()->R = current_root_R;
        m_robot->calcForwardKinematics();
    }
    prev_ref_contact_states = ref_contact_states;
    if (control_mode != MODE_ST) d_pos_z_root = 0.0;
}

void Stabilizer::storeCurrentStates()
{
    current_root_p = m_robot->rootLink()->p;
    current_root_R = m_robot->rootLink()->R;
    copyJointAnglesFromRobotModel(qorg, m_robot);
}

void Stabilizer::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
    // member variable : ref_contact_states, m_robot, stikp
    rats::coordinates leg_c[2];
    const hrp::Vector3 ez = hrp::Vector3::UnitZ();
    const hrp::Vector3 ex = hrp::Vector3::UnitX();

    for (size_t i = 0; i < stikp.size(); i++) {
        if (stikp[i].ee_name.find("leg") == std::string::npos) continue;

        const hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name)->link;
        leg_c[i].pos = target->p + target->R * foot_origin_offset[i];

        hrp::Vector3 xv1(target->R * ex);
        xv1(2)=0.0;
        xv1.normalize();
        const hrp::Vector3 yv1(ez.cross(xv1));
        // TODO: 整理
        leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
        leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
        leg_c[i].rot(0,2) = ez(0);  leg_c[i].rot(1,2) = ez(1);  leg_c[i].rot(2,2) = ez(2);
    }

    if (ref_contact_states[contact_states_index_map["rleg"]] &&
        ref_contact_states[contact_states_index_map["lleg"]]) {
        rats::coordinates mid_coords;
        rats::mid_coords(mid_coords, 0.5, leg_c[0], leg_c[1]);
        foot_origin_pos = mid_coords.pos;
        foot_origin_rot = mid_coords.rot;
    } else if (ref_contact_states[contact_states_index_map["rleg"]]) {
        foot_origin_pos = leg_c[contact_states_index_map["rleg"]].pos;
        foot_origin_rot = leg_c[contact_states_index_map["rleg"]].rot;
    } else {
        foot_origin_pos = leg_c[contact_states_index_map["lleg"]].pos;
        foot_origin_rot = leg_c[contact_states_index_map["lleg"]].rot;
    }
}

bool Stabilizer::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z)
{
    // member variable : m_robot, stikp, is_zmp_calc_enable
    double tmpzmpx = 0;
    double tmpzmpy = 0;
    double tmpfz = 0, tmpfz2 = 0.0; // TODO: rename

    for (size_t i = 0; i < stikp.size(); i++) {
        if (!is_zmp_calc_enable[i]) continue;
        hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
        // const hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
        hrp::Matrix33 sensor_R;
        rats::rotm3times(sensor_R, sensor->link->R, sensor->localR);
        hrp::Vector3 nf = sensor_R * wrenches[i].head<3>();
        hrp::Vector3 nm = sensor_R * wrenches[i].tail<3>();
        const hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
        tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
        tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
        tmpfz += nf(2);
        // calc ee-local COP
        const hrp::Link* target = m_robot->link(stikp[i].target_name);
        const hrp::Matrix33 eeR = target->R * stikp[i].localR;
        const hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * stikp[i].localp)); // ee-local force sensor pos
        nf = eeR.transpose() * nf;
        nm = eeR.transpose() * nm;
        // ee-local total moment and total force at ee position
        const double tmp_cop_mx = nf(2) * ee_fsp(1) - nf(1) * ee_fsp(2) + nm(0);
        const double tmp_cop_my = nf(2) * ee_fsp(0) - nf(0) * ee_fsp(2) - nm(1);
        const double tmp_cop_fz = nf(2);
        contact_cop_info[i][0] = tmp_cop_mx;
        contact_cop_info[i][1] = tmp_cop_my;
        contact_cop_info[i][2] = tmp_cop_fz;
        prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
        tmpfz2 += prev_act_force_z[i];
    }

    if (tmpfz2 < contact_decision_threshold) {
        ret_zmp = act_zmp;
        return false; // in the air
    } else {
        ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
        return true; // on ground
    }
}

bool Stabilizer::calcIfCOPisOutside()
{
    bool is_cop_outside = false;
    if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
        if (DEBUGP(loop)) std::cerr << "[" << comp_name << "] COP check" << std::endl;

        for (size_t i = 0; i < stikp.size(); i++) {
            if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
            // check COP inside
            // TODO: if のネストが多い?
            if (contact_cop_info[i][2] > 20.0) {
                const hrp::Vector3 tmp_cop(contact_cop_info[i][1] / contact_cop_info[i][2],
                                           contact_cop_info[i][0] / contact_cop_info[i][2],
                                           0);
                is_cop_outside = is_cop_outside ||
                    (!szd->is_inside_foot(tmp_cop, stikp[i].ee_name=="lleg", cop_check_margin) ||
                     szd->is_front_of_foot(tmp_cop, cop_check_margin) ||
                     szd->is_rear_of_foot(tmp_cop, cop_check_margin));

                if (DEBUGP(loop)) {
                    std::cerr << "[" << comp_name << "]   [" << stikp[i].ee_name << "] "
                              << "outside(" << !szd->is_inside_foot(tmp_cop, stikp[i].ee_name=="lleg", cop_check_margin) << ") "
                              << "front(" << szd->is_front_of_foot(tmp_cop, cop_check_margin) << ") "
                              << "rear(" << szd->is_rear_of_foot(tmp_cop, cop_check_margin) << ")" << std::endl;
                }
            } else {
                is_cop_outside = true;
            }
        }
    }

    return is_cop_outside;
}

bool Stabilizer::calcIfCPisOutside()
{
    bool is_cp_outside = false;
    if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
        Eigen::Vector2d current_cp = act_cp.head(2);
        szd->get_margined_vertices(margined_support_polygon_vetices);
        szd->calc_convex_hull(margined_support_polygon_vetices, act_contact_states, rel_ee_pos, rel_ee_rot);
        if (!is_walking || is_estop_while_walking) is_cp_outside = !szd->is_inside_support_polygon(current_cp, -sbp_cog_offset);

        if (DEBUGP(loop)) {
            std::cerr << "[" << comp_name << "] CP value " << "[" << act_cp(0) << "," << act_cp(1) << "] [m], "
                      << "sbp cog offset [" << sbp_cog_offset(0) << " " << sbp_cog_offset(1) << "], outside ? "
                      << (is_cp_outside ? "Outside" : "Inside")
                      << std::endl;
        }

        if (is_cp_outside) {
            if (initial_cp_too_large_error || loop % static_cast<int>(0.2/dt) == 0 ) { // once per 0.2[s]
                std::cerr << "[" << comp_name << "] ["
                          << "] CP too large error " << "[" << act_cp(0) << "," << act_cp(1) << "] [m]" << std::endl;
            }
            initial_cp_too_large_error = false;
        } else {
            initial_cp_too_large_error = true;
        }
    }

    return is_cp_outside;
}

bool Stabilizer::calcFalling() // TODO: rename
{
    // TODO: 整理
    bool is_falling = false;
    bool will_fall = false;
    hrp::Vector3 fall_direction = hrp::Vector3::Zero();
    double total_force = 0.0;

    // TODO: ifが多い
    for (size_t i = 0; i < stikp.size(); i++) {
        if (is_zmp_calc_enable[i]) {
            if (is_walking) {
                if (projected_normal.at(i).norm() > sin(tilt_margin[0])) {
                    will_fall = true;
                    if (m_will_fall_counter[i] % static_cast <int>(1.0/dt) == 0 ) { // once per 1.0[s]
                        std::cerr << "[" << comp_name << "] ["
                                  << "] " << stikp[i].ee_name << " cannot support total weight, "
                                  << "swgsuptime : " << control_swing_support_time[i] << ", state : " << ref_contact_states[i]
                                  << ", otherwise robot will fall down toward " << "(" << projected_normal.at(i)(0) << "," << projected_normal.at(i)(1) << ") direction" << std::endl;
                    }
                    m_will_fall_counter[i]++;
                } else {
                    m_will_fall_counter[i] = 0;
                }
            }
            fall_direction += projected_normal.at(i) * act_force.at(i).norm();
            total_force += act_force.at(i).norm();
        }
    }

    if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
        fall_direction = fall_direction / total_force;
    } else {
        fall_direction = hrp::Vector3::Zero();
    }

    if (fall_direction.norm() > sin(tilt_margin[1])) {
        is_falling = true;
        if (m_is_falling_counter % static_cast <int>(0.2/dt) == 0) { // once per 0.2[s]
            std::cerr << "[" << comp_name << "] ["
                      << "] robot is falling down toward " << "(" << fall_direction(0) << "," << fall_direction(1) << ") direction" << std::endl;
        }
        m_is_falling_counter++;
    } else {
        m_is_falling_counter = 0;
    }

    return is_falling || will_fall;
}

void Stabilizer::calcStateForEmergencySignal()
{
    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] Check Emergency State (seq = " << (is_seq_interpolating ? "interpolating" : "empty") << ")" << std::endl;
    }

    // TODO: 組み合わせにする？ CP かつ TILTみたいな
    // Total check for emergency signal
    switch (emergency_check_mode) {
      case OpenHRP::AutoBalanceStabilizerService::NO_CHECK:
          is_emergency = false;
          break;
      case OpenHRP::AutoBalanceStabilizerService::COP:
          is_emergency = calcIfCOPisOutside() && is_seq_interpolating;
          break;
      case OpenHRP::AutoBalanceStabilizerService::CP:
          is_emergency = calcIfCPisOutside();
          break;
      case OpenHRP::AutoBalanceStabilizerService::TILT:
          is_emergency = calcFalling();
          break;
      default:
          break;
    }

    // TODO: TILTがない
    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] EmergencyCheck ("
                  << (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::NO_CHECK ? "NO_CHECK":
                      (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::COP ? "COP" :
                       "CP"))
                  << ") " << (is_emergency ? "emergency" : "non-emergency") << std::endl;
    }

    whether_send_emergency_signal = false;
    if (reset_emergency_flag) {
        emergency_signal = 0;
        reset_emergency_flag = false;
        whether_send_emergency_signal = true;
    } else if (is_emergency) {
        emergency_signal = 1;
        whether_send_emergency_signal = true;
    }

    rel_ee_pos.clear();
    rel_ee_rot.clear();
    rel_ee_name.clear();
}

void Stabilizer::moveBasePosRotForBodyRPYControl ()
{
    // member variable : transition_smooth_gain, eefm_body_attitude_control_gain, eefm_body_attitude_control_time_const, d_rpy, root_rot_compensation_limit, current_root_R, target_root_R, target_root_p, rel_cog, current_base_rpy, current_base_pos

    // Body rpy control
    //   Basically Equation (1) and (2) in the paper [1]
    const hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(target_root_R);
    bool is_root_rot_limit = false;

    for (size_t i = 0; i < 2; i++) {
        d_rpy[i] = transition_smooth_gain * (eefm_body_attitude_control_gain[i] * (ref_root_rpy(i) - act_base_rpy(i)) - 1 / eefm_body_attitude_control_time_const[i] * d_rpy[i]) * dt + d_rpy[i];
        d_rpy[i] = clamp(d_rpy[i], -root_rot_compensation_limit[i], root_rot_compensation_limit[i]);
        is_root_rot_limit = is_root_rot_limit || (std::fabs(std::fabs(d_rpy[i]) - root_rot_compensation_limit[i]) < 1e-5); // near the limit
    }

    rats::rotm3times(current_root_R, target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
    m_robot->rootLink()->R = current_root_R;
    m_robot->rootLink()->p = target_root_p + target_root_R * rel_cog - current_root_R * rel_cog;
    m_robot->calcForwardKinematics();

    current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    current_base_pos = m_robot->rootLink()->p;

    if (DEBUGP(loop) || (is_root_rot_limit && loop % 200 == 0)) {
        std::cerr << "[" << comp_name << "] Root rot control" << std::endl;
        if (is_root_rot_limit) std::cerr << "[" << comp_name << "]   Root rot limit reached!!" << std::endl;
        std::cerr << "[" << comp_name << "]   ref = [" << rad2deg(ref_root_rpy(0)) << " " << rad2deg(ref_root_rpy(1)) << "], "
                  << "act = [" << rad2deg(act_base_rpy(0)) << " " << rad2deg(act_base_rpy(1)) << "], "
                  << "cur = [" << rad2deg(current_base_rpy(0)) << " " << rad2deg(current_base_rpy(1)) << "], "
                  << "limit = [" << rad2deg(root_rot_compensation_limit[0]) << " " << rad2deg(root_rot_compensation_limit[1])
                  << "][deg]" << std::endl;
    }
}

void Stabilizer::calcSwingSupportLimbGain ()
{
    for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        if (ref_contact_states[i]) { // Support
            // Limit too large support time increment. Max time is 3600.0[s] = 1[h], this assumes that robot's one step time is smaller than 1[h].
            ikp.support_time = std::min(3600.0, ikp.support_time + dt);
            // In some PC, does not work because the first line is optimized out.
            // ikp.support_time += dt;
            // ikp.support_time = std::min(3600.0, ikp.support_time);
            if (ikp.support_time > eefm_pos_transition_time) {
                ikp.swing_support_gain = (control_swing_support_time[i] / eefm_pos_transition_time);
            } else {
                ikp.swing_support_gain = (ikp.support_time / eefm_pos_transition_time);
            }
            ikp.swing_support_gain = clamp(ikp.swing_support_gain, 0.0, 1.0);
        } else { // Swing
            ikp.swing_support_gain = 0.0;
            ikp.support_time = 0.0;
        }
    }

    if (DEBUGP(loop)) {
        const size_t stikp_size = stikp.size();

        std::cerr << "[" << comp_name << "] SwingSupportLimbGain = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << stikp[i].swing_support_gain << " ";

        std::cerr << "], ref_contact_states = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << ref_contact_states[i] << " ";

        std::cerr << "], sstime = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << control_swing_support_time[i] << " ";

        std::cerr << "], toe_heel_ratio = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << toe_heel_ratio[i] << " ";

        std::cerr << "], support_time = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << stikp[i].support_time << " ";

        std::cerr << "]" << std::endl;
    }
}

void Stabilizer::calcTPCC()
{
    // stabilizer loop
    // Choi's feedback law
    const hrp::Vector3 cog = m_robot->calcCM();
    const hrp::Vector3 dcog(ref_cog - act_cog);
    const hrp::Vector3 dzmp(ref_zmp - act_zmp);
    hrp::Vector3 newcog = hrp::Vector3::Zero();

    for (size_t i = 0; i < 2; i++) {
        double uu = ref_cogvel(i)
            - k_tpcc_p[i] * transition_smooth_gain * dzmp(i)
            + k_tpcc_x[i] * transition_smooth_gain * dcog(i);
        newcog(i) = uu * dt + cog(i);
    }

    moveBasePosRotForBodyRPYControl();

    // target at ee => target at link-origin
    hrp::Vector3 target_link_p[stikp.size()];
    hrp::Matrix33 target_link_R[stikp.size()];
    for (size_t i = 0; i < stikp.size(); i++) {
        rats::rotm3times(target_link_R[i], target_ee_R[i], stikp[i].localR.transpose());
        target_link_p[i] = target_ee_p[i] - target_ee_R[i] * stikp[i].localCOPPos;
    }

    // TODO: IKはここじゃない気がする
    // solveIK
    //   IK target is link origin pos and rot, not ee pos and rot.
    size_t max_ik_loop_count = 0;
    for (size_t i = 0; i < stikp.size(); i++) {
        max_ik_loop_count = std::max(max_ik_loop_count, stikp[i].ik_loop_count);
    }

    const double move_cog_ratio = 0.9;
    for (size_t _ = 0; _ < max_ik_loop_count; ++_) {
        const hrp::Vector3 com = m_robot->calcCM();
        for (size_t i = 0; i < 2; i++) {
            m_robot->rootLink()->p(i) = m_robot->rootLink()->p(i) + 0.9 * (newcog(i) - com(i));
        }
        m_robot->calcForwardKinematics();

        for (size_t i = 0; i < stikp.size(); i++) {
            if (!is_ik_enable[i]) continue;
            jpe_v[i]->calcInverseKinematics2Loop(target_link_p[i], target_link_R[i], 1.0, stikp[i].avoid_gain, stikp[i].reference_gain, &qrefv, transition_smooth_gain);
        }
    }
}

void Stabilizer::calcEEForceMomentControl()
{
    // TODO: この復元ここ？
    // stabilizer loop
    // return to reference
    copyJointAnglesToRobotModel(m_robot, qrefv);

    for (size_t i = 0; i < jpe_v.size(); i++) {
        if (!is_ik_enable[i]) continue;
        for (size_t j = 0; j < jpe_v[i]->numJoints(); j++){
            const int idx = jpe_v[i]->joint(j)->jointId;
            m_robot->joint(idx)->q = qorg[idx];
        }

        // Fix for toe joint
        if (jpe_v[i]->numJoints() == 7) {
            const int toe_idx = jpe_v[i]->joint(jpe_v[i]->numJoints() - 1)->jointId;
            m_robot->joint(toe_idx)->q = qrefv[toe_idx];
        }
    }

    // State calculation for swing ee compensation
    //   joint angle : current control output
    //   root pos : target root p
    //   root rot : actual root rot
    {
        // Calc status
        m_robot->rootLink()->R = target_root_R;
        m_robot->rootLink()->p = target_root_p;
        m_robot->calcForwardKinematics(); // TODO: これいる？
        const hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        const hrp::Matrix33 senR = sen->link->R * sen->localR;
        m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
        m_robot->calcForwardKinematics();

        hrp::Vector3 foot_origin_pos;
        hrp::Matrix33 foot_origin_rot;
        calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
        // Calculate foot_origin_coords-relative ee pos and rot
        // Subtract them from target_ee_diff_xx
        for (size_t i = 0; i < stikp.size(); i++) {
            const hrp::Link* target = m_robot->link(stikp[i].target_name);
            stikp[i].target_ee_diff_p -= foot_origin_rot.transpose() * (target->p + target->R * stikp[i].localp - foot_origin_pos);
            stikp[i].target_ee_diff_r  = (foot_origin_rot.transpose() * target->R * stikp[i].localR).transpose() * stikp[i].target_ee_diff_r;
        }
    }

    // State calculation for control : calculate "current" state
    //   joint angle : current control output
    //   root pos : target + keep COG against rpy control
    //   root rot : target + rpy control
    moveBasePosRotForBodyRPYControl();

    // Convert d_foot_pos in foot origin frame => "current" world frame
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    calcFootOriginCoords(foot_origin_pos, foot_origin_rot);
    std::vector<hrp::Vector3> current_d_foot_pos(stikp.size());
    for (size_t i = 0; i < stikp.size(); i++) {
        current_d_foot_pos[i] = foot_origin_rot * stikp[i].d_foot_pos;
    }

    // Swing ee compensation.
    calcSwingEEModification();

    // TODO: IKの位置はここじゃない？ or stikpのループを1つにする
    // solveIK
    //   IK target is link origin pos and rot, not ee pos and rot.
    std::vector<hrp::Vector3>  ref_ee_p(stikp.size()); // TODO: should be target_ee_p ?
    std::vector<hrp::Matrix33> ref_ee_R(stikp.size());
    double tmp_d_pos_z_root = 0.0;
    for (size_t i = 0; i < stikp.size(); i++) {
        if (!is_ik_enable[i]) continue;

        // Add damping_control compensation to target value
        if (is_feedback_control_enable[i]) {
            rats::rotm3times(ref_ee_R[i], target_ee_R[i], hrp::rotFromRpy(-1 * stikp[i].ee_d_foot_rpy));
            // foot force difference control version
            // total_target_foot_p[i](2) = target_foot_p[i](2) + (i==0?0.5:-0.5)*zctrl;
            // foot force independent damping control
            ref_ee_p[i] = target_ee_p[i] - current_d_foot_pos[i];
        } else {
            ref_ee_p[i] = target_ee_p[i];
            ref_ee_R[i] = target_ee_R[i];
        }
        // Add swing ee compensation
        rats::rotm3times(ref_ee_R[i], ref_ee_R[i], hrp::rotFromRpy(stikp[i].d_rpy_swing));
        ref_ee_p[i] = ref_ee_p[i] + foot_origin_rot * stikp[i].d_pos_swing;
    }

    for (size_t i = 0; i < stikp.size(); i++) {
        if (!is_ik_enable[i]) continue;
        for (size_t _ = 0; _ < stikp[i].ik_loop_count; ++_) {
            jpe_v[i]->calcInverseKinematics2Loop(ref_ee_p[i], ref_ee_R[i], 1.0, 0.001, 0.01,
                                                 &qrefv, transition_smooth_gain,
                                                 //stikp[i].localCOPPos;
                                                 stikp[i].localp, stikp[i].localR);
        }
    }
}

// Swing ee compensation.
//   Calculate compensation values to minimize the difference between "current" foot-origin-coords-relative pos and rot and "target" foot-origin-coords-relative pos and rot for swing ee.
//   Input  : target_ee_diff_p, target_ee_diff_r
//   Output : d_pos_swing, d_rpy_swing
void Stabilizer::calcSwingEEModification ()
{
    for (size_t i = 0; i < stikp.size(); i++) {
        // Calc compensation values
        constexpr double limit_pos = 30 * 1e-3; // 30[mm] limit
        constexpr double limit_rot = deg2rad(10); // 10[deg] limit
        if (ref_contact_states[contact_states_index_map[stikp[i].ee_name]] ||
            act_contact_states[contact_states_index_map[stikp[i].ee_name]]) {
            // If actual contact or target contact is ON, do not use swing ee compensation. Exponential zero retrieving.
            stikp[i].d_rpy_swing = calcDampingControl(stikp[i].d_rpy_swing, stikp[i].eefm_swing_rot_time_const);
            stikp[i].d_pos_swing = calcDampingControl(stikp[i].d_pos_swing, stikp[i].eefm_swing_pos_time_const);
            stikp[i].target_ee_diff_p_filter->reset(stikp[i].d_pos_swing);
            stikp[i].target_ee_diff_r_filter->reset(stikp[i].d_rpy_swing);
        } else {
            /* position */
            {
                const hrp::Vector3 diff_p = stikp[i].eefm_swing_pos_spring_gain.cwiseProduct(stikp[i].target_ee_diff_p_filter->passFilter(stikp[i].target_ee_diff_p));
                const double lvlimit = -50 * 1e-3 * dt;
                const double uvlimit =  50 * 1e-3 * dt; // 50 [mm/s]
                const hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_pos_swing + lvlimit * hrp::Vector3::Ones();
                const hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_pos_swing + uvlimit * hrp::Vector3::Ones();
                stikp[i].d_pos_swing = clamp(clamp(diff_p, -limit_pos, limit_pos), limit_by_lvlimit, limit_by_uvlimit);
            }
            /* rotation */
            {
                const hrp::Vector3 diff_r = stikp[i].eefm_swing_rot_spring_gain.cwiseProduct(stikp[i].target_ee_diff_r_filter->passFilter(hrp::rpyFromRot(stikp[i].target_ee_diff_r)));
                const double lvlimit = deg2rad(-20.0 * dt);
                const double uvlimit = deg2rad(20.0 * dt); // 20 [deg/s]
                const hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_rpy_swing + lvlimit * hrp::Vector3::Ones();
                const hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_rpy_swing + uvlimit * hrp::Vector3::Ones();
                stikp[i].d_rpy_swing = clamp(clamp(diff_r, -limit_rot, limit_rot), limit_by_lvlimit, limit_by_uvlimit);
            }
        }
        stikp[i].prev_d_pos_swing = stikp[i].d_pos_swing;
        stikp[i].prev_d_rpy_swing = stikp[i].d_rpy_swing;
    }

    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] Swing foot control" << std::endl;
        for (size_t i = 0; i < stikp.size(); i++) {
            std::cerr << "[" << comp_name << "]   "
                      << "d_rpy_swing (" << stikp[i].ee_name << ")  = "
                      << (stikp[i].d_rpy_swing / M_PI * 180.0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[deg], "
                      << "d_pos_swing (" << stikp[i].ee_name << ")  = "
                      << (stikp[i].d_pos_swing * 1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[mm]" << std::endl;
        }
    }
}

// Damping control functions
//   Basically Equation (14) in the paper [1]
double Stabilizer::calcDampingControl(const double tau_d, const double tau, const double prev_d,
                                       const double DD, const double TT)
{
    return (1 / DD * (tau_d - tau) - 1 / TT * prev_d) * dt + prev_d;
}

hrp::Vector3 Stabilizer::calcDampingControl(const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                             const hrp::Vector3& DD, const hrp::Vector3& TT)
{
    return ((tau_d - tau).cwiseQuotient(DD) - prev_d.cwiseQuotient(TT)) * dt + prev_d;
}

// Retrieving only
double Stabilizer::calcDampingControl(const double prev_d, const double TT)
{
    return -1 / TT * prev_d * dt + prev_d;
}

// Retrieving only
hrp::Vector3 Stabilizer::calcDampingControl(const hrp::Vector3& prev_d, const hrp::Vector3& TT)
{
    return (-prev_d.cwiseQuotient(TT)) * dt + prev_d;
}

void Stabilizer::calcDiffFootOriginExtMoment()
{
    // calc reference ext moment around foot origin pos
    const double mg = total_mass * gravitational_acceleration; // TODO: gravitational_accelerationを消す
    const hrp::Vector3 ref_ext_moment(mg  * ref_cog(1) - ref_total_foot_origin_moment(0),
                                      -mg * ref_cog(0) - ref_total_foot_origin_moment(1),
                                       0);
    // calc act ext moment around foot origin pops
    hrp::Vector3 act_ext_moment;
    // Do not calculate actual value if in the air, because of invalid act_zmp.
    if (on_ground) act_ext_moment = hrp::Vector3(mg  * act_cog(1) - act_total_foot_origin_moment(0),
                                                 -mg * act_cog(0) - act_total_foot_origin_moment(1),
                                                 0);
    else act_ext_moment = ref_ext_moment;

    // Calc diff
    diff_foot_origin_ext_moment = ref_ext_moment - act_ext_moment;

    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] DiffStaticBalancePointOffset\n";
        std::cerr << "[" << comp_name << "]   "
                  << "ref_ext_moment = "
                  << ref_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                  << "[mm], " << "act_ext_moment = "
                  << act_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                  << "[mm], " << "diff ext_moment = "
                  << diff_foot_origin_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                  << "[mm]" << std::endl;
    }
}

void Stabilizer::syncToSt()
{
    std::cerr << "[" << comp_name << "] ["
              << "] Sync IDLE => ST"  << std::endl;
    // TODO: この辺の初期化をまとめたい
    d_rpy[0] = d_rpy[1] = 0;
    pos_ctrl = hrp::Vector3::Zero();
    for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        ikp.target_ee_diff_p = hrp::Vector3::Zero();
        ikp.target_ee_diff_r = hrp::Matrix33::Identity();
        ikp.d_pos_swing = ikp.prev_d_pos_swing = hrp::Vector3::Zero();
        ikp.d_rpy_swing = ikp.prev_d_rpy_swing = hrp::Vector3::Zero();
        ikp.target_ee_diff_p_filter->reset(hrp::Vector3::Zero());
        ikp.target_ee_diff_r_filter->reset(hrp::Vector3::Zero());
        ikp.d_foot_pos = ikp.ee_d_foot_pos = ikp.d_foot_rpy = ikp.ee_d_foot_rpy = hrp::Vector3::Zero();
    }

    if (on_ground) {
        transition_count = -calcMaxTransitionCount();
        control_mode = MODE_ST;
    } else {
        transition_count = 0;
        control_mode = MODE_AIR;
    }
}

void Stabilizer::syncToIdle()
{
    std::cerr << "[" << comp_name << "] ["
              << "] Sync ST => IDLE"  << std::endl;
    transition_count = calcMaxTransitionCount();
    for (size_t i = 0; i < m_robot->numJoints(); i++) {
        transition_joint_q[i] = m_robot->joint(i)->q;
    }
}

void Stabilizer::startStabilizer()
{
    waitSTTransition(); // Wait until all transition has finished
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (control_mode == MODE_IDLE) {
            std::cerr << "[" << comp_name << "] " << "Start ST"  << std::endl;
            syncToSt();
        }
    }
    waitSTTransition();
}

void Stabilizer::stopStabilizer()
{
    waitSTTransition(); // Wait until all transition has finished
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if ((control_mode == MODE_ST || control_mode == MODE_AIR)) {
            std::cerr << "[" << comp_name << "] " << "Stop ST"  << std::endl;
            control_mode = (control_mode == MODE_ST) ? MODE_SYNC_TO_IDLE : MODE_IDLE;
        }
    }
    waitSTTransition();
    std::cerr << "[" << comp_name << "] " << "Stop ST DONE"  << std::endl;
}

std::string Stabilizer::getStabilizerAlgorithmString(const OpenHRP::AutoBalanceStabilizerService::STAlgorithm _st_algorithm)
{
    switch (_st_algorithm) {
    case OpenHRP::AutoBalanceStabilizerService::TPCC:
        return "TPCC";
    case OpenHRP::AutoBalanceStabilizerService::EEFM:
        return "EEFM";
    case OpenHRP::AutoBalanceStabilizerService::EEFMQP:
        return "EEFMQP";
    case OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP:
        return "EEFMQPCOP";
    case OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP2:
        return "EEFMQPCOP2";
    default:
        return "";
    }
}

void Stabilizer::setBoolSequenceParam(std::vector<bool>& st_bool_values,
                                      const OpenHRP::AutoBalanceStabilizerService::BoolSequence& output_bool_values,
                                      const std::string& prop_name)
{
    const std::vector<bool> prev_values(st_bool_values);

    if (st_bool_values.size() != output_bool_values.length()) {
        std::cerr << "[" << comp_name << "]   " << prop_name
                  << " cannot be set. Length " << st_bool_values.size()
                  << " != " << output_bool_values.length() << std::endl;
    } else if (control_mode != MODE_IDLE) {
        std::cerr << "[" << comp_name << "]   " << prop_name
                  << " cannot be set. Current control_mode is "
                  << control_mode << std::endl;
    } else {
        for (size_t i = 0; i < st_bool_values.size(); i++) {
            st_bool_values[i] = output_bool_values[i];
        }
    }

    std::cerr << "[" << comp_name << "]   " << prop_name << " is ";
    for (size_t i = 0; i < st_bool_values.size(); i++) {
        std::cerr <<"[" << st_bool_values[i] << "]";
    }
    std::cerr << "(set = ";
    for (size_t i = 0; i < output_bool_values.length(); i++) {
        std::cerr <<"[" << output_bool_values[i] << "]";
    }
    std::cerr << ", prev = ";
    for (size_t i = 0; i < prev_values.size(); i++) {
        std::cerr <<"[" << prev_values[i] << "]";
    }
    std::cerr << ")" << std::endl;
}

// TODO: 上のを合わせられない?
void Stabilizer::setBoolSequenceParamWithCheckContact(std::vector<bool>& st_bool_values,
                                                      const OpenHRP::AutoBalanceStabilizerService::BoolSequence& output_bool_values,
                                                      const std::string& prop_name)
{
    std::vector<bool> prev_values(st_bool_values);

    if (st_bool_values.size() != output_bool_values.length()) {
        std::cerr << "[" << comp_name << "]   " << prop_name
                  << " cannot be set. Length " << st_bool_values.size()
                  << " != " << output_bool_values.length() << std::endl;
    } else if ( control_mode == MODE_IDLE ) {
        for (size_t i = 0; i < st_bool_values.size(); i++) {
            st_bool_values[i] = output_bool_values[i];
        }
    } else {
        std::vector<size_t> failed_indices;
        for (size_t i = 0; i < st_bool_values.size(); i++) {
            if (st_bool_values[i] != output_bool_values[i]) { // If mode change
                // reference contact_states should be OFF
                if (!ref_contact_states[i]) st_bool_values[i] = output_bool_values[i];
                else failed_indices.push_back(i);
            }
        }
        if (failed_indices.size() > 0) {
            std::cerr << "[" << comp_name << "]   " << prop_name << " cannot be set partially. failed_indices is [";
            for (size_t i = 0; i < failed_indices.size(); i++) {
                std::cerr << failed_indices[i] << " ";
            }
            std::cerr << "]" << std::endl;
        }
    }

    std::cerr << "[" << comp_name << "]   " << prop_name << " is ";
    for (size_t i = 0; i < st_bool_values.size(); i++) {
        std::cerr <<"[" << st_bool_values[i] << "]";
    }
    std::cerr << "(set = ";
    for (size_t i = 0; i < output_bool_values.length(); i++) {
        std::cerr <<"[" << output_bool_values[i] << "]";
    }
    std::cerr << ", prev = ";
    for (size_t i = 0; i < prev_values.size(); i++) {
        std::cerr <<"[" << prev_values[i] << "]";
    }
    std::cerr << ")" << std::endl;
}

void Stabilizer::waitSTTransition()
{
    // Wait condition
    //   1. Check transition_count : Wait until transition is finished
    //   2. Check control_mode : Once control_mode is SYNC mode, wait until control_mode moves to the next mode (MODE_AIR or MODE_IDLE)
    bool flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
    while (transition_count != 0 ||
           (flag ? !(control_mode == MODE_IDLE || control_mode == MODE_AIR) : false)) {
        usleep(10);
        flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
    }
    usleep(10);
}

void Stabilizer::getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_stp)
{
    std::cerr << "[" << comp_name << "] getStabilizerParam" << std::endl;

    for (size_t i = 0; i < 2; i++) {
        i_stp.k_tpcc_p[i] = k_tpcc_p[i];
        i_stp.k_tpcc_x[i] = k_tpcc_x[i];
        i_stp.k_brot_p[i] = k_brot_p[i];
        i_stp.k_brot_tc[i] = k_brot_tc[i];
    }

    for (size_t i = 0; i < 2; i++) {
        i_stp.eefm_k1[i] = eefm_k1[i];
        i_stp.eefm_k2[i] = eefm_k2[i];
        i_stp.eefm_k3[i] = eefm_k3[i];
        i_stp.eefm_zmp_delay_time_const[i] = eefm_zmp_delay_time_const[i];
        i_stp.eefm_ref_zmp_aux[i] = ref_zmp_aux(i);
        i_stp.eefm_body_attitude_control_time_const[i] = eefm_body_attitude_control_time_const[i];
        i_stp.eefm_body_attitude_control_gain[i] = eefm_body_attitude_control_gain[i];
        i_stp.ref_capture_point[i] = ref_cp(i);
        i_stp.act_capture_point[i] = act_cp(i);
        i_stp.cp_offset[i] = cp_offset(i);
    }

    const size_t stikp_size = stikp.size();
    i_stp.eefm_pos_time_const_support.length(stikp_size);
    i_stp.eefm_pos_damping_gain.length(stikp_size);
    i_stp.eefm_pos_compensation_limit.length(stikp_size);
    i_stp.eefm_swing_pos_spring_gain.length(stikp_size);
    i_stp.eefm_swing_pos_time_const.length(stikp_size);
    i_stp.eefm_rot_time_const.length(stikp_size);
    i_stp.eefm_rot_damping_gain.length(stikp_size);
    i_stp.eefm_rot_compensation_limit.length(stikp_size);
    i_stp.eefm_swing_rot_spring_gain.length(stikp_size);
    i_stp.eefm_swing_rot_time_const.length(stikp_size);
    i_stp.eefm_ee_moment_limit.length(stikp_size);
    i_stp.eefm_ee_forcemoment_distribution_weight.length(stikp_size);
    for (size_t j = 0; j < stikp_size; j++) {
        i_stp.eefm_pos_damping_gain[j].length(3);
        i_stp.eefm_pos_time_const_support[j].length(3);
        i_stp.eefm_swing_pos_spring_gain[j].length(3);
        i_stp.eefm_swing_pos_time_const[j].length(3);
        i_stp.eefm_rot_damping_gain[j].length(3);
        i_stp.eefm_rot_time_const[j].length(3);
        i_stp.eefm_swing_rot_spring_gain[j].length(3);
        i_stp.eefm_swing_rot_time_const[j].length(3);
        i_stp.eefm_ee_moment_limit[j].length(3);
        i_stp.eefm_ee_forcemoment_distribution_weight[j].length(6);
        for (size_t i = 0; i < 3; i++) {
            i_stp.eefm_pos_damping_gain[j][i] = stikp[j].eefm_pos_damping_gain(i);
            i_stp.eefm_pos_time_const_support[j][i] = stikp[j].eefm_pos_time_const_support(i);
            i_stp.eefm_swing_pos_spring_gain[j][i] = stikp[j].eefm_swing_pos_spring_gain(i);
            i_stp.eefm_swing_pos_time_const[j][i] = stikp[j].eefm_swing_pos_time_const(i);
            i_stp.eefm_rot_damping_gain[j][i] = stikp[j].eefm_rot_damping_gain(i);
            i_stp.eefm_rot_time_const[j][i] = stikp[j].eefm_rot_time_const(i);
            i_stp.eefm_swing_rot_spring_gain[j][i] = stikp[j].eefm_swing_rot_spring_gain(i);
            i_stp.eefm_swing_rot_time_const[j][i] = stikp[j].eefm_swing_rot_time_const(i);
            i_stp.eefm_ee_moment_limit[j][i] = stikp[j].eefm_ee_moment_limit(i);
            i_stp.eefm_ee_forcemoment_distribution_weight[j][i] = stikp[j].eefm_ee_forcemoment_distribution_weight(i);
            i_stp.eefm_ee_forcemoment_distribution_weight[j][i+3] = stikp[j].eefm_ee_forcemoment_distribution_weight(i+3);
        }
        i_stp.eefm_pos_compensation_limit[j] = stikp[j].eefm_pos_compensation_limit;
        i_stp.eefm_rot_compensation_limit[j] = stikp[j].eefm_rot_compensation_limit;
    }

    for (size_t i = 0; i < 3; i++) {
        i_stp.eefm_swing_pos_damping_gain[i] = eefm_swing_pos_damping_gain(i);
        i_stp.eefm_swing_rot_damping_gain[i] = eefm_swing_rot_damping_gain(i);
    }

    i_stp.eefm_pos_time_const_swing = eefm_pos_time_const_swing;
    i_stp.eefm_pos_transition_time = eefm_pos_transition_time;
    i_stp.eefm_pos_margin_time = eefm_pos_margin_time;
    i_stp.eefm_leg_inside_margin = szd->get_leg_inside_margin();
    i_stp.eefm_leg_outside_margin = szd->get_leg_outside_margin();
    i_stp.eefm_leg_front_margin = szd->get_leg_front_margin();
    i_stp.eefm_leg_rear_margin = szd->get_leg_rear_margin();

    std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
    szd->get_vertices(support_polygon_vec);
    i_stp.eefm_support_polygon_vertices_sequence.length(support_polygon_vec.size());
    for (size_t ee_idx = 0; ee_idx < support_polygon_vec.size(); ee_idx++) {
        i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(support_polygon_vec[ee_idx].size());
        for (size_t v_idx = 0; v_idx < support_polygon_vec[ee_idx].size(); v_idx++) {
            i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0] = support_polygon_vec[ee_idx][v_idx](0);
            i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1] = support_polygon_vec[ee_idx][v_idx](1);
        }
    }

    i_stp.eefm_cogvel_cutoff_freq = act_cogvel_filter->getCutOffFreq();
    i_stp.eefm_wrench_alpha_blending = szd->get_wrench_alpha_blending();
    i_stp.eefm_alpha_cutoff_freq = szd->get_alpha_cutoff_freq();
    i_stp.gravitational_acceleration = gravitational_acceleration;
    i_stp.eefm_ee_error_cutoff_freq = stikp[0].target_ee_diff_p_filter->getCutOffFreq();
    i_stp.eefm_use_force_difference_control = eefm_use_force_difference_control;
    i_stp.eefm_use_swing_damping = eefm_use_swing_damping;

    for (size_t i = 0; i < 3; ++i) {
        i_stp.eefm_swing_damping_force_thre[i] = eefm_swing_damping_force_thre[i];
        i_stp.eefm_swing_damping_moment_thre[i] = eefm_swing_damping_moment_thre[i];
    }
    i_stp.is_ik_enable.length(is_ik_enable.size());
    for (size_t i = 0; i < is_ik_enable.size(); i++) {
        i_stp.is_ik_enable[i] = is_ik_enable[i];
    }
    i_stp.is_feedback_control_enable.length(is_feedback_control_enable.size());
    for (size_t i = 0; i < is_feedback_control_enable.size(); i++) {
        i_stp.is_feedback_control_enable[i] = is_feedback_control_enable[i];
    }
    i_stp.is_zmp_calc_enable.length(is_zmp_calc_enable.size());
    for (size_t i = 0; i < is_zmp_calc_enable.size(); i++) {
        i_stp.is_zmp_calc_enable[i] = is_zmp_calc_enable[i];
    }

    i_stp.foot_origin_offset.length(2);
    for (size_t i = 0; i < i_stp.foot_origin_offset.length(); i++) {
        i_stp.foot_origin_offset[i].length(3);
        i_stp.foot_origin_offset[i][0] = foot_origin_offset[i](0);
        i_stp.foot_origin_offset[i][1] = foot_origin_offset[i](1);
        i_stp.foot_origin_offset[i][2] = foot_origin_offset[i](2);
    }

    i_stp.st_algorithm = st_algorithm;
    i_stp.transition_time = transition_time;
    i_stp.cop_check_margin = cop_check_margin;
    for (size_t i = 0; i < cp_check_margin.size(); i++) {
        i_stp.cp_check_margin[i] = cp_check_margin[i];
    }
    for (size_t i = 0; i < tilt_margin.size(); i++) {
        i_stp.tilt_margin[i] = tilt_margin[i];
    }
    i_stp.contact_decision_threshold = contact_decision_threshold;
    i_stp.is_estop_while_walking = is_estop_while_walking;
    switch(control_mode) {
      case MODE_IDLE: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_IDLE; break;
      case MODE_AIR: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_AIR; break;
      case MODE_ST: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_ST; break;
      case MODE_SYNC_TO_IDLE: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_SYNC_TO_IDLE; break;
      case MODE_SYNC_TO_AIR: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_SYNC_TO_AIR; break;
      default: break;
    }
    i_stp.emergency_check_mode = emergency_check_mode;
    i_stp.end_effector_list.length(stikp_size);
    i_stp.use_limb_stretch_avoidance = use_limb_stretch_avoidance;
    i_stp.use_zmp_truncation = use_zmp_truncation;
    i_stp.limb_stretch_avoidance_time_const = limb_stretch_avoidance_time_const;
    i_stp.limb_length_margin.length(stikp_size);
    i_stp.detection_time_to_air = detection_count_to_mode_air * dt;
    for (size_t i = 0; i < 2; i++) {
        i_stp.limb_stretch_avoidance_vlimit[i] = limb_stretch_avoidance_vlimit[i];
        i_stp.root_rot_compensation_limit[i] = root_rot_compensation_limit[i];
    }
    for (size_t i = 0; i < stikp_size; i++) {
        const rats::coordinates cur_ee = rats::coordinates(stikp.at(i).localp, stikp.at(i).localR);
        OpenHRP::AutoBalanceStabilizerService::Footstep ret_ee;
        // position
        memcpy(ret_ee.pos, cur_ee.pos.data(), sizeof(double)*3);
        // rotation
        Eigen::Quaternion<double> qt(cur_ee.rot);
        ret_ee.rot[0] = qt.w();
        ret_ee.rot[1] = qt.x();
        ret_ee.rot[2] = qt.y();
        ret_ee.rot[3] = qt.z();
        // name
        ret_ee.leg = stikp.at(i).ee_name.c_str();
        // set
        i_stp.end_effector_list[i] = ret_ee;
        i_stp.limb_length_margin[i] = stikp[i].limb_length_margin;
    }
    i_stp.ik_limb_parameters.length(jpe_v.size());
    for (size_t i = 0; i < jpe_v.size(); i++) {
        OpenHRP::AutoBalanceStabilizerService::IKLimbParameters& ilp = i_stp.ik_limb_parameters[i];
        ilp.ik_optional_weight_vector.length(jpe_v[i]->numJoints());
        std::vector<double> ov;
        ov.resize(jpe_v[i]->numJoints());
        jpe_v[i]->getOptionalWeightVector(ov);
        for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
            ilp.ik_optional_weight_vector[j] = ov[j];
        }
        ilp.sr_gain = jpe_v[i]->getSRGain();
        ilp.avoid_gain = stikp[i].avoid_gain;
        ilp.reference_gain = stikp[i].reference_gain;
        ilp.manipulability_limit = jpe_v[i]->getManipulabilityLimit();
        ilp.ik_loop_count = stikp[i].ik_loop_count; // size_t -> unsigned short, value may change, but ik_loop_count is small value and value not change
    }
}

void Stabilizer::setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_stp)
{
    std::cerr << "[" << comp_name << "] setStabilizerParam" << std::endl;
    std::lock_guard<std::mutex> lock(m_mutex);

    for (size_t i = 0; i < 2; i++) {
        k_tpcc_p[i] = i_stp.k_tpcc_p[i];
        k_tpcc_x[i] = i_stp.k_tpcc_x[i];
        k_brot_p[i] = i_stp.k_brot_p[i];
        k_brot_tc[i] = i_stp.k_brot_tc[i];
    }

    std::cerr << "[" << comp_name << "]  TPCC" << std::endl;
    std::cerr << "[" << comp_name
              << "]   k_tpcc_p  = [" << k_tpcc_p[0] << ", " << k_tpcc_p[1]
              << "], k_tpcc_x  = [" << k_tpcc_x[0] << ", " << k_tpcc_x[1]
              << "], k_brot_p  = [" << k_brot_p[0] << ", " << k_brot_p[1]
              << "], k_brot_tc = [" << k_brot_tc[0] << ", " << k_brot_tc[1]
              << "]" << std::endl;

    std::cerr << "[" << comp_name << "]  EEFM" << std::endl;
    for (size_t i = 0; i < 2; i++) {
        eefm_k1[i] = i_stp.eefm_k1[i];
        eefm_k2[i] = i_stp.eefm_k2[i];
        eefm_k3[i] = i_stp.eefm_k3[i];
        eefm_zmp_delay_time_const[i] = i_stp.eefm_zmp_delay_time_const[i];
        ref_zmp_aux(i) = i_stp.eefm_ref_zmp_aux[i];
        eefm_body_attitude_control_gain[i] = i_stp.eefm_body_attitude_control_gain[i];
        eefm_body_attitude_control_time_const[i] = i_stp.eefm_body_attitude_control_time_const[i];
        ref_cp(i) = i_stp.ref_capture_point[i];
        act_cp(i) = i_stp.act_capture_point[i];
        cp_offset(i) = i_stp.cp_offset[i];
    }

    const size_t stikp_size = stikp.size();
    bool is_damping_parameter_ok = false;
    if (i_stp.eefm_pos_damping_gain.length()                   == stikp_size &&
        i_stp.eefm_pos_time_const_support.length()             == stikp_size &&
        i_stp.eefm_pos_compensation_limit.length()             == stikp_size &&
        i_stp.eefm_swing_pos_spring_gain.length()              == stikp_size &&
        i_stp.eefm_swing_pos_time_const.length()               == stikp_size &&
        i_stp.eefm_rot_damping_gain.length()                   == stikp_size &&
        i_stp.eefm_rot_time_const.length()                     == stikp_size &&
        i_stp.eefm_rot_compensation_limit.length()             == stikp_size &&
        i_stp.eefm_swing_rot_spring_gain.length()              == stikp_size &&
        i_stp.eefm_swing_rot_time_const.length()               == stikp_size &&
        i_stp.eefm_ee_moment_limit.length()                    == stikp_size &&
        i_stp.eefm_ee_forcemoment_distribution_weight.length() == stikp_size) {

        is_damping_parameter_ok = true;
        for (size_t j = 0; j < stikp_size; j++) {
            for (size_t i = 0; i < 3; i++) {
                stikp[j].eefm_pos_damping_gain(i) = i_stp.eefm_pos_damping_gain[j][i];
                stikp[j].eefm_pos_time_const_support(i) = i_stp.eefm_pos_time_const_support[j][i];
                stikp[j].eefm_swing_pos_spring_gain(i) = i_stp.eefm_swing_pos_spring_gain[j][i];
                stikp[j].eefm_swing_pos_time_const(i) = i_stp.eefm_swing_pos_time_const[j][i];
                stikp[j].eefm_rot_damping_gain(i) = i_stp.eefm_rot_damping_gain[j][i];
                stikp[j].eefm_rot_time_const(i) = i_stp.eefm_rot_time_const[j][i];
                stikp[j].eefm_swing_rot_spring_gain(i) = i_stp.eefm_swing_rot_spring_gain[j][i];
                stikp[j].eefm_swing_rot_time_const(i) = i_stp.eefm_swing_rot_time_const[j][i];
                stikp[j].eefm_ee_moment_limit(i) = i_stp.eefm_ee_moment_limit[j][i];
                stikp[j].eefm_ee_forcemoment_distribution_weight(i) = i_stp.eefm_ee_forcemoment_distribution_weight[j][i];
                stikp[j].eefm_ee_forcemoment_distribution_weight(i+3) = i_stp.eefm_ee_forcemoment_distribution_weight[j][i+3];
            }
            stikp[j].eefm_pos_compensation_limit = i_stp.eefm_pos_compensation_limit[j];
            stikp[j].eefm_rot_compensation_limit = i_stp.eefm_rot_compensation_limit[j];
        }
    }

    for (size_t i = 0; i < 3; i++) {
        eefm_swing_pos_damping_gain(i) = i_stp.eefm_swing_pos_damping_gain[i];
        eefm_swing_rot_damping_gain(i) = i_stp.eefm_swing_rot_damping_gain[i];
    }
    eefm_pos_time_const_swing = i_stp.eefm_pos_time_const_swing;
    eefm_pos_transition_time = i_stp.eefm_pos_transition_time;
    eefm_pos_margin_time = i_stp.eefm_pos_margin_time;
    szd->set_leg_inside_margin(i_stp.eefm_leg_inside_margin);
    szd->set_leg_outside_margin(i_stp.eefm_leg_outside_margin);
    szd->set_leg_front_margin(i_stp.eefm_leg_front_margin);
    szd->set_leg_rear_margin(i_stp.eefm_leg_rear_margin);
    szd->set_vertices_from_margin_params();

    if (i_stp.eefm_support_polygon_vertices_sequence.length() != stikp_size) {
        std::cerr << "[" << comp_name << "]   eefm_support_polygon_vertices_sequence cannot be set. Length "
                  << i_stp.eefm_support_polygon_vertices_sequence.length()
                  << " != " << stikp_size << std::endl;
    } else {
        std::cerr << "[" << comp_name << "]   eefm_support_polygon_vertices_sequence set" << std::endl;
        std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
        for (size_t ee_idx = 0; ee_idx < i_stp.eefm_support_polygon_vertices_sequence.length(); ee_idx++) {
            std::vector<Eigen::Vector2d> tvec; // TODO: サイズ分からない？ rename
            for (size_t v_idx = 0; v_idx < i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(); v_idx++) {
                tvec.push_back(Eigen::Vector2d(i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0],
                                               i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1]));
            }
            support_polygon_vec.push_back(tvec);
        }
        szd->set_vertices(support_polygon_vec);
        szd->print_vertices(comp_name);
    }

    eefm_use_force_difference_control = i_stp.eefm_use_force_difference_control;
    eefm_use_swing_damping = i_stp.eefm_use_swing_damping;
    for (size_t i = 0; i < 3; ++i) {
        eefm_swing_damping_force_thre[i] = i_stp.eefm_swing_damping_force_thre[i];
        eefm_swing_damping_moment_thre[i] = i_stp.eefm_swing_damping_moment_thre[i];
    }
    act_cogvel_filter->setCutOffFreq(i_stp.eefm_cogvel_cutoff_freq);
    szd->set_wrench_alpha_blending(i_stp.eefm_wrench_alpha_blending);
    szd->set_alpha_cutoff_freq(i_stp.eefm_alpha_cutoff_freq);
    gravitational_acceleration = i_stp.gravitational_acceleration;
    for (size_t i = 0; i < stikp_size; i++) {
        stikp[i].target_ee_diff_p_filter->setCutOffFreq(i_stp.eefm_ee_error_cutoff_freq);
        stikp[i].target_ee_diff_r_filter->setCutOffFreq(i_stp.eefm_ee_error_cutoff_freq);
        stikp[i].limb_length_margin = i_stp.limb_length_margin[i];
    }

    setBoolSequenceParam(is_ik_enable, i_stp.is_ik_enable, std::string("is_ik_enable"));
    setBoolSequenceParamWithCheckContact(is_feedback_control_enable, i_stp.is_feedback_control_enable, std::string("is_feedback_control_enable"));
    setBoolSequenceParam(is_zmp_calc_enable, i_stp.is_zmp_calc_enable, std::string("is_zmp_calc_enable"));

    emergency_check_mode = i_stp.emergency_check_mode;
    transition_time = i_stp.transition_time;
    cop_check_margin = i_stp.cop_check_margin;
    for (size_t i = 0; i < cp_check_margin.size(); i++) {
        cp_check_margin[i] = i_stp.cp_check_margin[i];
    }
    szd->set_vertices_from_margin_params(cp_check_margin);
    for (size_t i = 0; i < tilt_margin.size(); i++) {
        tilt_margin[i] = i_stp.tilt_margin[i];
    }
    contact_decision_threshold = i_stp.contact_decision_threshold;
    is_estop_while_walking = i_stp.is_estop_while_walking;
    use_limb_stretch_avoidance = i_stp.use_limb_stretch_avoidance;
    use_zmp_truncation = i_stp.use_zmp_truncation;
    limb_stretch_avoidance_time_const = i_stp.limb_stretch_avoidance_time_const;
    for (size_t i = 0; i < 2; i++) {
        limb_stretch_avoidance_vlimit[i] = i_stp.limb_stretch_avoidance_vlimit[i];
        root_rot_compensation_limit[i] = i_stp.root_rot_compensation_limit[i];
    }
    detection_count_to_mode_air = static_cast<int>(i_stp.detection_time_to_air / dt);
    if (control_mode == MODE_IDLE) {
        for (size_t i = 0; i < i_stp.end_effector_list.length(); i++) {
            auto ikp_itr = std::find_if(stikp.begin(), stikp.end(), [&i_stp, i](STIKParam& ikp) { return ikp.ee_name == std::string(i_stp.end_effector_list[i].leg); });
            memcpy(ikp_itr->localp.data(), i_stp.end_effector_list[i].pos, sizeof(double)*3);
            ikp_itr->localR = (Eigen::Quaternion<double>(i_stp.end_effector_list[i].rot[0], i_stp.end_effector_list[i].rot[1], i_stp.end_effector_list[i].rot[2], i_stp.end_effector_list[i].rot[3])).normalized().toRotationMatrix();
        }
    } else {
        std::cerr << "[" << comp_name << "] cannot change end-effectors except during MODE_IDLE" << std::endl;
    }
    for (std::vector<STIKParam>::const_iterator it = stikp.begin(); it != stikp.end(); it++) {
        std::cerr << "[" << comp_name << "]  End Effector [" << it->ee_name << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   localpos = " << it->localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        std::cerr << "[" << comp_name << "]   localR = " << it->localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", "    [", "]")) << std::endl;
    }
    if (i_stp.foot_origin_offset.length () != 2) {
        std::cerr << "[" << comp_name << "]   foot_origin_offset cannot be set. Length " << i_stp.foot_origin_offset.length() << " != " << 2 << std::endl;
    } else if (control_mode != MODE_IDLE) {
        std::cerr << "[" << comp_name << "]   foot_origin_offset cannot be set. Current control_mode is " << control_mode << std::endl;
    } else {
        for (size_t i = 0; i < i_stp.foot_origin_offset.length(); i++) {
            foot_origin_offset[i](0) = i_stp.foot_origin_offset[i][0];
            foot_origin_offset[i](1) = i_stp.foot_origin_offset[i][1];
            foot_origin_offset[i](2) = i_stp.foot_origin_offset[i][2];
        }
    }
    std::cerr << "[" << comp_name << "]   foot_origin_offset is ";
    for (size_t i = 0; i < 2; i++) {
        std::cerr << foot_origin_offset[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"));
    }
    std::cerr << "[m]" << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_k1  = [" << eefm_k1[0] << ", " << eefm_k1[1] << "], eefm_k2  = [" << eefm_k2[0] << ", " << eefm_k2[1] << "], eefm_k3  = [" << eefm_k3[0] << ", " << eefm_k3[1] << "]" << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_zmp_delay_time_const  = [" << eefm_zmp_delay_time_const[0] << ", " << eefm_zmp_delay_time_const[1] << "][s], eefm_ref_zmp_aux  = [" << ref_zmp_aux(0) << ", " << ref_zmp_aux(1) << "][m]" << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_body_attitude_control_gain  = [" << eefm_body_attitude_control_gain[0] << ", " << eefm_body_attitude_control_gain[1] << "], eefm_body_attitude_control_time_const  = [" << eefm_body_attitude_control_time_const[0] << ", " << eefm_body_attitude_control_time_const[1] << "][s]" << std::endl;
    if (is_damping_parameter_ok) {
        for (size_t j = 0; j < stikp_size; j++) {
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] eefm_rot_damping_gain = "
                      << stikp[j].eefm_rot_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << ", eefm_rot_time_const = "
                      << stikp[j].eefm_rot_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[s]" << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] eefm_pos_damping_gain = "
                      << stikp[j].eefm_pos_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << ", eefm_pos_time_const_support = "
                      << stikp[j].eefm_pos_time_const_support.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[s]" << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] "
                      << "eefm_pos_compensation_limit = " << stikp[j].eefm_pos_compensation_limit << "[m], "
                      << "eefm_rot_compensation_limit = " << stikp[j].eefm_rot_compensation_limit << "[rad], "
                      << "eefm_ee_moment_limit = " << stikp[j].eefm_ee_moment_limit.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] "
                      << "eefm_swing_pos_spring_gain = " << stikp[j].eefm_swing_pos_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << "eefm_swing_pos_time_const = " << stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << "eefm_swing_rot_spring_gain = " << stikp[j].eefm_swing_rot_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << "eefm_swing_pos_time_const = " << stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] "
                      << "eefm_ee_forcemoment_distribution_weight = " << stikp[j].eefm_ee_forcemoment_distribution_weight.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "" << std::endl;
        }
    } else {
        std::cerr << "[" << comp_name << "]   eefm damping parameters cannot be set because of invalid param." << std::endl;
    }
    std::cerr << "[" << comp_name << "]   eefm_pos_transition_time = " << eefm_pos_transition_time << "[s], eefm_pos_margin_time = " << eefm_pos_margin_time << "[s] eefm_pos_time_const_swing = " << eefm_pos_time_const_swing << "[s]" << std::endl;
    std::cerr << "[" << comp_name << "]   cogvel_cutoff_freq = " << act_cogvel_filter->getCutOffFreq() << "[Hz]" << std::endl;
    szd->print_params(std::string(comp_name));
    std::cerr << "[" << comp_name << "]   gravitational_acceleration = " << gravitational_acceleration << "[m/s^2], eefm_use_force_difference_control = " << (eefm_use_force_difference_control? "true":"false") << ", eefm_use_swing_damping = " << (eefm_use_swing_damping? "true":"false") << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_ee_error_cutoff_freq = " << stikp[0].target_ee_diff_p_filter->getCutOffFreq() << "[Hz]" << std::endl;
    std::cerr << "[" << comp_name << "]  COMMON" << std::endl;
    if (control_mode == MODE_IDLE) {
        st_algorithm = i_stp.st_algorithm;
        std::cerr << "[" << comp_name << "]   st_algorithm changed to [" << getStabilizerAlgorithmString(st_algorithm) << "]" << std::endl;
    } else {
        std::cerr << "[" << comp_name << "]   st_algorithm cannot be changed to [" << getStabilizerAlgorithmString(st_algorithm) << "] during MODE_AIR or MODE_ST." << std::endl;
    }
    std::cerr << "[" << comp_name << "]   emergency_check_mode changed to [" << (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::NO_CHECK?"NO_CHECK": (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::COP?"COP":"CP") ) << "]" << std::endl;
    std::cerr << "[" << comp_name << "]   transition_time = " << transition_time << "[s]" << std::endl;
    std::cerr << "[" << comp_name << "]   cop_check_margin = " << cop_check_margin << "[m], "
              << "cp_check_margin = [" << cp_check_margin[0] << ", " << cp_check_margin[1] << ", " << cp_check_margin[2] << ", " << cp_check_margin[3] << "] [m], "
              << "tilt_margin = [" << tilt_margin[0] << ", " << tilt_margin[1] << "] [rad]" << std::endl;
    std::cerr << "[" << comp_name << "]   contact_decision_threshold = " << contact_decision_threshold << "[N], detection_time_to_air = " << detection_count_to_mode_air * dt << "[s]" << std::endl;
    std::cerr << "[" << comp_name << "]   root_rot_compensation_limit = [" << root_rot_compensation_limit[0] << " " << root_rot_compensation_limit[1] << "][rad]" << std::endl;
    // IK limb parameters
    std::cerr << "[" << comp_name << "]  IK limb parameters" << std::endl;
    bool is_ik_limb_parameter_valid_length = true;
    if (i_stp.ik_limb_parameters.length() != jpe_v.size()) {
        is_ik_limb_parameter_valid_length = false;
        std::cerr << "[" << comp_name << "]   ik_limb_parameters invalid length! Cannot be set. (input = " << i_stp.ik_limb_parameters.length() << ", desired = " << jpe_v.size() << ")" << std::endl;
    } else {
        for (size_t i = 0; i < jpe_v.size(); i++) {
            if (jpe_v[i]->numJoints() != i_stp.ik_limb_parameters[i].ik_optional_weight_vector.length())
                is_ik_limb_parameter_valid_length = false;
        }
        if (is_ik_limb_parameter_valid_length) {
            for (size_t i = 0; i < jpe_v.size(); i++) {
                const OpenHRP::AutoBalanceStabilizerService::IKLimbParameters& ilp = i_stp.ik_limb_parameters[i];
                std::vector<double> ov;
                ov.resize(jpe_v[i]->numJoints());
                for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
                    ov[j] = ilp.ik_optional_weight_vector[j];
                }
                jpe_v[i]->setOptionalWeightVector(ov);
                jpe_v[i]->setSRGain(ilp.sr_gain);
                stikp[i].avoid_gain = ilp.avoid_gain;
                stikp[i].reference_gain = ilp.reference_gain;
                jpe_v[i]->setManipulabilityLimit(ilp.manipulability_limit);
                stikp[i].ik_loop_count = ilp.ik_loop_count; // unsigned short -> size_t, value not change
            }
        } else {
            std::cerr << "[" << comp_name << "]   ik_optional_weight_vector invalid length! Cannot be set. (input = [";
            for (size_t i = 0; i < jpe_v.size(); i++) {
                std::cerr << i_stp.ik_limb_parameters[i].ik_optional_weight_vector.length() << ", ";
            }
            std::cerr << "], desired = [";
            for (size_t i = 0; i < jpe_v.size(); i++) {
                std::cerr << jpe_v[i]->numJoints() << ", ";
            }
            std::cerr << "])" << std::endl;
        }
    }
    if (is_ik_limb_parameter_valid_length) {
        std::cerr << "[" << comp_name << "]   ik_optional_weight_vectors = ";
        for (size_t i = 0; i < jpe_v.size(); i++) {
            std::vector<double> ov;
            ov.resize(jpe_v[i]->numJoints());
            jpe_v[i]->getOptionalWeightVector(ov);
            std::cerr << "[";
            for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
                std::cerr << ov[j] << " ";
            }
            std::cerr << "]";
        }
        std::cerr << std::endl;
        std::cerr << "[" << comp_name << "]   sr_gains = [";
        for (size_t i = 0; i < jpe_v.size(); i++) {
            std::cerr << jpe_v[i]->getSRGain() << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   avoid_gains = [";
        for (size_t i = 0; i < stikp_size; i++) {
            std::cerr << stikp[i].avoid_gain << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   reference_gains = [";
        for (size_t i = 0; i < stikp_size; i++) {
            std::cerr << stikp[i].reference_gain << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   manipulability_limits = [";
        for (size_t i = 0; i < jpe_v.size(); i++) {
            std::cerr << jpe_v[i]->getManipulabilityLimit() << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   ik_loop_count = [";
        for (size_t i = 0; i < stikp_size; i++) {
            std::cerr << stikp[i].ik_loop_count << ", ";
        }
        std::cerr << "]" << std::endl;
    }
}

}
