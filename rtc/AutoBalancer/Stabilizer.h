// -*- C++ -*-
/*!
 * @file  Stabilizer.h
 * @brief stabilizer filter
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ABS_STABILIZER_H
#define ABS_STABILIZER_H

#include <memory>
#include <mutex>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"
#include "hrpsys/idl/AutoBalancerService.hh"
#include "TwoDofController.h"
#include "ZMPDistributor.h"

struct paramsFromAutoBalancer {
    hrp::dvector q_ref;
    hrp::Vector3 zmp_ref;
    hrp::Vector3 base_pos_ref;
    hrp::Vector3 base_rpy_ref;
    std::vector<bool> ref_contact_states;
    std::vector<double> toe_heel_ratio;
    std::vector<double> controlSwingSupportTime; // TODO: rename
    std::vector<hrp::dvector6> wrenches_ref;
};

struct paramsFromSensors {
    hrp::dvector q_current;
    hrp::Vector3 rpy;
    std::vector<hrp::dvector6> wrenches;
};

class Stabilizer
{
  public: // TODO: public関数をprivateにする
    Stabilizer(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt);
    virtual ~Stabilizer() {};

    void initStabilizer(const RTC::Properties& prop, const size_t ee_num);
    void execStabilizer(const paramsFromAutoBalancer& abc_param,
                        const paramsFromSensors& sensor_param);
    void startStabilizer();
    void stopStabilizer();
    void storeCurrentStates();
    void calcTargetParameters(const paramsFromAutoBalancer& abc_param);
    void calcActualParameters(const paramsFromSensors& sensor_param);
    void calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot);
    void sync_2_st();
    void sync_2_idle();
    bool calcZMP(hrp::Vector3& ret_zmp, const double zmp_z);
    void calcRUNST();
    void moveBasePosRotForBodyRPYControl ();
    void calcSwingSupportLimbGain();
    void calcTPCC();
    void calcEEForceMomentControl();
    void calcSwingEEModification ();
    void getStabilizerParam(OpenHRP::AutoBalancerService::StabilizerParam& i_stp);
    void setStabilizerParam(const OpenHRP::AutoBalancerService::StabilizerParam& i_stp);
    void setBoolSequenceParam (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name);
    void setBoolSequenceParamWithCheckContact (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name);
    std::string getStabilizerAlgorithmString (OpenHRP::AutoBalancerService::STAlgorithm _st_algorithm);
    void waitSTTransition();
    // funcitons for calc final torque output
    void calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p);
    void calcTorque();
    void fixLegToCoords (const std::string& leg, const rats::coordinates& coords);
    void getFootmidCoords (rats::coordinates& ret);
    double calcDampingControl (const double tau_d, const double tau, const double prev_d,
                               const double DD, const double TT);
    hrp::Vector3 calcDampingControl (const hrp::Vector3& prev_d, const hrp::Vector3& TT);
    double calcDampingControl (const double prev_d, const double TT);
    hrp::Vector3 calcDampingControl (const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                     const hrp::Vector3& DD, const hrp::Vector3& TT);
    void calcDiffFootOriginExtMoment ();

    bool isContact (const size_t idx) // 0 = right, 1 = left
    {
        return (prev_act_force_z[idx] > 25.0);
    }
    int calcMaxTransitionCount ()
    {
        return (transition_time / dt);
    }
    // TODO: tmporarary function: delete this function after merging autobalancer IK and stabilizer IK
    void addSTIKParam(const std::string& ee_name, const std::string& target_name, const std::string& sensor_name)
    {
        STIKParam ikp;
        ikp.ee_name = ee_name;
        ikp.target_name = target_name;
        ikp.sensor_name = sensor_name;
        stikp.push_back(ikp);
    }

  private:
    bool calcIfCOPisOutside();
    bool calcIfCPisOutside();
    bool calcFalling();
    void calcStateForEmergencySignal();

    // Stabilizer Parameters
    struct STIKParam {
        STIKParam()
            : eefm_rot_damping_gain(hrp::Vector3(20*5, 20*5, 1e5)),
              eefm_rot_time_const(hrp::Vector3(1.5, 1.5, 1.5)),
              eefm_rot_compensation_limit(0.17453), // 10 [deg]
              eefm_swing_rot_spring_gain(hrp::Vector3(0.0, 0.0, 0.0)),
              eefm_swing_rot_time_const(hrp::Vector3(1.5, 1.5, 1.5)),
              eefm_pos_damping_gain(hrp::Vector3(3500*10, 3500*10, 3500)),
              eefm_pos_time_const_support(hrp::Vector3(1.5, 1.5, 1.5)),
              eefm_pos_compensation_limit(0.025),
              eefm_swing_pos_spring_gain(hrp::Vector3(0.0, 0.0, 0.0)),
              eefm_swing_pos_time_const(hrp::Vector3(1.5, 1.5, 1.5)),
              eefm_ee_moment_limit(hrp::Vector3(1e4, 1e4, 1e4)), // Default limit [Nm] is too large. Same as no limit.
              support_time(0.0),
              // For swing end-effector modification
              target_ee_diff_p(hrp::Vector3::Zero()),
              target_ee_diff_r(hrp::Matrix33::Identity()),
              d_rpy_swing(hrp::Vector3::Zero()),
              d_pos_swing(hrp::Vector3::Zero()),
              // target_ee_diff_p_filter(std::make_shared<FirstOrderLowPassFilter<hrp::Vector3>>(50.0, m_dt, hrp::Vector3::Zero())), // [Hz]
              // target_ee_diff_r_filter(std::make_shared<FirstOrderLowPassFilter<hrp::Vector3>>(50.0, m_dt, hrp::Vector3::Zero())), // [Hz]
              prev_d_pos_swing(hrp::Vector3::Zero()),
              prev_d_rpy_swing(hrp::Vector3::Zero()),
              // IK param
              avoid_gain(0.001),
              reference_gain(0.01),
              max_limb_length(0.0),
              limb_length_margin(0.13),
              ik_loop_count(3)
        {}
        std::string ee_name; // Name of ee (e.g., rleg, lleg, ...)
        std::string target_name; // Name of end link
        std::string sensor_name; // Name of force sensor in the limb
        std::string parent_name; // Name of parent ling in the limb
        hrp::Vector3 localp; // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
        hrp::Vector3 localCOPPos; // Position offset of reference COP in end link frame (^{l}p_{cop} = R_l^T (p_{cop} - p_l) - ^{l}p_e)
        hrp::Matrix33 localR; // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)
        // For eefm
        hrp::Vector3 d_foot_pos, ee_d_foot_pos, d_foot_rpy, ee_d_foot_rpy;
        hrp::Vector3 eefm_pos_damping_gain, eefm_pos_time_const_support, eefm_rot_damping_gain, eefm_rot_time_const, eefm_swing_rot_spring_gain, eefm_swing_pos_spring_gain, eefm_swing_rot_time_const, eefm_swing_pos_time_const, eefm_ee_moment_limit;
        double eefm_pos_compensation_limit, eefm_rot_compensation_limit;
        hrp::Vector3 ref_force, ref_moment;
        hrp::dvector6 eefm_ee_forcemoment_distribution_weight;
        double swing_support_gain, support_time;
        // For swing ee modification
        std::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3>> target_ee_diff_p_filter, target_ee_diff_r_filter;
        hrp::Vector3 target_ee_diff_p, d_pos_swing, d_rpy_swing, prev_d_pos_swing, prev_d_rpy_swing;
        hrp::Matrix33 target_ee_diff_r;
        // IK parameter
        double avoid_gain, reference_gain, max_limb_length, limb_length_margin;
        size_t ik_loop_count;
    };

    enum cmode {MODE_IDLE, MODE_AIR, MODE_ST, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_AIR} control_mode;

    hrp::BodyPtr m_robot;
    std::string comp_name;
    double dt;
    unsigned int loop;
    std::mutex m_mutex;

    // TODO: 整理
    unsigned int transition_count;
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::vector<hrp::JointPathExPtr> jpe_v;
    hrp::dvector transition_joint_q, qorg, qrefv;
    std::vector<STIKParam> stikp;
    std::map<std::string, size_t> contact_states_index_map;
    std::vector<bool> ref_contact_states, prev_ref_contact_states, act_contact_states, is_ik_enable, is_feedback_control_enable, is_zmp_calc_enable;
    std::vector<double> toe_heel_ratio;
    int m_is_falling_counter;
    std::vector<int> m_will_fall_counter;
    int is_air_counter, detection_count_to_air;
    bool is_legged_robot, on_ground, is_emergency, is_seq_interpolating, reset_emergency_flag, eefm_use_force_difference_control, eefm_use_swing_damping, initial_cp_too_large_error, use_limb_stretch_avoidance, use_zmp_truncation;
    bool is_walking, is_estop_while_walking;
    hrp::Vector3 current_root_p, target_root_p;
    hrp::Matrix33 current_root_R, target_root_R, prev_act_foot_origin_rot, prev_ref_foot_origin_rot, target_foot_origin_rot, ref_foot_origin_rot, act_Rs;
    std::vector <hrp::Vector3> target_ee_p, rel_ee_pos, act_ee_p, projected_normal, act_force, ref_force, ref_moment;
    std::vector <hrp::Matrix33> target_ee_R, rel_ee_rot, act_ee_R;
    std::vector<std::string> rel_ee_name;
    rats::coordinates target_foot_midcoords;
    hrp::Vector3 ref_zmp, ref_cog, ref_cp, ref_cogvel, rel_ref_cp, prev_ref_cog, prev_ref_zmp;
    hrp::Vector3 act_zmp, act_cog, act_cogvel, act_cp, rel_act_zmp, rel_act_cp, prev_act_cog, act_base_rpy, current_base_rpy, current_base_pos, sbp_cog_offset, cp_offset, diff_cp;
    hrp::Vector3 foot_origin_offset[2];
    std::vector<double> prev_act_force_z;
    double zmp_origin_off, transition_smooth_gain, d_pos_z_root, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2], root_rot_compensation_limit[2];
    std::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_cogvel_filter;
    OpenHRP::AutoBalancerService::STAlgorithm st_algorithm;
    std::unique_ptr<SimpleZMPDistributor> szd;
    std::vector<std::vector<Eigen::Vector2d> > support_polygon_vetices, margined_support_polygon_vetices;
    std::vector<hrp::Vector3> contact_cop_info;
    std::vector<hrp::dvector6> wrenches;
    std::vector<double> controlSwingSupportTime;

    // TPCC
    double k_tpcc_p[2], k_tpcc_x[2], d_rpy[2], k_brot_p[2], k_brot_tc[2];

    // RUN ST
    TwoDofController m_tau_x[2], m_tau_y[2], m_f_z;
    hrp::Vector3 pdr;
    double m_torque_k[2], m_torque_d[2]; // 3D-LIP parameters (0: x, 1: y)
    double pangx_ref, pangy_ref, pangx, pangy;
    double k_run_b[2], d_run_b[2];
    double rdx, rdy, rx, ry;

    // EEFM ST
    double eefm_k1[2], eefm_k2[2], eefm_k3[2], eefm_zmp_delay_time_const[2], eefm_body_attitude_control_gain[2], eefm_body_attitude_control_time_const[2];
    double eefm_pos_time_const_swing, eefm_pos_transition_time, eefm_pos_margin_time, eefm_gravitational_acceleration;
    std::vector<double> eefm_swing_damping_force_thre, eefm_swing_damping_moment_thre;
    hrp::Vector3 new_refzmp, rel_cog, ref_zmp_aux, diff_foot_origin_ext_moment;
    hrp::Vector3 pos_ctrl;
    hrp::Vector3 ref_total_force, ref_total_moment;

    // Total foot moment around the foot origin coords (relative to foot origin coords)
    hrp::Vector3 ref_total_foot_origin_moment, act_total_foot_origin_moment;
    hrp::Vector3 eefm_swing_pos_damping_gain, eefm_swing_rot_damping_gain;
    double total_mass, transition_time, cop_check_margin, contact_decision_threshold;
    std::vector<double> cp_check_margin, tilt_margin;
    OpenHRP::AutoBalancerService::EmergencyCheckMode emergency_check_mode;
};

#endif // ABS_STABILIZER_H
