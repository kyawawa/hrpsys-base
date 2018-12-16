// -*- C++ -*-
/*!
 * @file  Balancer.cpp
 * @brief Update ReferenceForce
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <iomanip>
// #include "hrpsys/util/VectorConvert.h"
// #include "hrpsys/util/Hrpsys.h"
#include "Balancer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/EigenUtil>
// #include <cnoid/Referenced>
#include "util/Interpolator.h"

using Guard = coil::Guard<coil::Mutex>;
// using BodyPtrWeak = cnoid::weak_ref_ptr<cnoid::Body>;

namespace {
constexpr double G_ACC = -9.80665;
inline bool eps_eq(const double a, const double b, const double eps = 1e-3) { return std::abs(a - b) <= eps; };
}

// Module specification
// <rtc-template block="module_spec">
static const char* Balancer_spec[] =
  {
    "implementation_id", "Balancer",
    "type_name",         "Balancer",
    "description",       "update reference force",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

Balancer::Balancer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    // Input
    m_BalancerServicePort("BalancerService"),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_rpyIn("rpy", m_rpy),
    m_qRefIn("qRef", m_qRef),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    // Output
    m_qRefOut("q", m_qRef),
    m_basePosOut("basePosOut", m_basePos),
    // </rtc-template>
    // m_robot(hrp::BodyPtr()),
    m_debugLevel(0)
{
    m_BalancerService.balancer(this);
}

Balancer::~Balancer()
{
}

RTC::ReturnCode_t Balancer::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");

    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("rpy", m_rpyIn);
    addInPort("qRef", m_qRefIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    // Set OutPort buffers
    addOutPort("q", m_qRefOut);

    // Set service provider to Ports
    m_BalancerServicePort.registerProvider("service0", "BalancerService", m_BalancerService);

    // Set service consumers to Ports
    // Set CORBA Service Ports
    addPort(m_BalancerServicePort);

    loop = 0;
    control_startcount = 0;
    control_endcount = 0;
    control_mode = NOCONTROL;

    // Get dt
    RTC::Properties& prop = getProperties(); // get properties information from .wrl file
    coil::stringTo(m_dt, prop["dt"].c_str());
    std::cerr << "dt: " << m_dt << std::endl;

    // Load Model File
    {
        cnoid::BodyLoader bodyLoader;
        std::string model_file = prop["model"];
        std::string erage_str = "file://";
        size_t pos = model_file.find(erage_str);
        if (pos != std::string::npos) model_file.erase(pos, erage_str.length());
        std::cerr << "model: " << model_file << std::endl;
        ioBody = bodyLoader.load(model_file);
        if (!ioBody) {
            std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
            return RTC::RTC_ERROR;
        }
    }

    // RTC::Manager& rtcManager = RTC::Manager::instance();
    // std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    // int comPos = nameServer.find(",");
    // if (comPos < 0){
    //     comPos = nameServer.length();
    // }
    // nameServer = nameServer.substr(0, comPos);
    // RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    {
        // check if the dof of ioBody match the number of joint in ioBody
        int dof = ioBody->numJoints();
        for (int i = 0; i < dof; i++) {
            if (i != ioBody->joint(i)->jointId()) {
                std::cerr << "[" << m_profile.instance_name << "] jointId is not equal to the index number" << std::endl;
                return RTC::RTC_ERROR;
            }
        }
    }

    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    // find names for real force sensors
    auto fsensors = ioBody->devices<cnoid::ForceSensor>().getSortedById();
    fsensor_names.reserve(fsensors.size());
    for (const auto& fsensor : fsensors) {
        fsensor_names.push_back(fsensor->name());
    }
    // TODO
    // // load virtual force sensors
    // readVirtualForceSensorParamFromProperties(m_vfs, ioBody, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    // unsigned num_virtual_fsensors = m_vfs.size();
    // for (int i = 0; i < num_virtual_fsensors; i++) {
    //     for (auto it = m_vfs.begin(); it != m_vfs.end(); it++) {
    //         if (it->second.id == i) fsensor_names.push_back(it->first);
    //     }
    // }

    // add ports for all force sensors
    size_t num_fsensors  = fsensor_names.size();
    m_force.resize(num_fsensors);
    m_forceIn.resize(num_fsensors);
    act_force.resize(num_fsensors);
    std::cerr << "[" << m_profile.instance_name << "] create force sensor ports" << std::endl;
    for (size_t i = 0; i < num_fsensors; ++i) {
        // actual inport
        m_forceIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
    }

    // setting from conf file
    // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    if (end_effectors_str.size() > 0) {
        size_t prop_num = 10;
        size_t num = end_effectors_str.size() / prop_num;
        target_ee.resize(num);
        ee_trans_vec.reserve(num);

        for (size_t i = 0; i < num; i++) {
            const std::string ee_name(std::move(end_effectors_str[i*prop_num]));
            const std::string ee_target(std::move(end_effectors_str[i*prop_num+1]));
            const std::string ee_base(std::move(end_effectors_str[i*prop_num+2]));

            EETrans ee_local_trans;
            {
                const cnoid::Vector3 local_pos(std::stod(end_effectors_str[i*prop_num+3]),
                                               std::stod(end_effectors_str[i*prop_num+3+1]),
                                               std::stod(end_effectors_str[i*prop_num+3+2]));
                // for (size_t j = 0; j < 3; j++) {
                //     coil::stringTo(local_pos(j), end_effectors_str[i*prop_num+3+j].c_str());
                // }
                ee_local_trans.local_trans.translation() = std::move(local_pos);
            }
            {
                // double rotation_vector[4]; // rotation in VRML is represented by axis + angle
                // for (size_t j = 0; j < 4; ++j)
                //     coil::stringTo(rotation_vector[j], end_effectors_str[i*prop_num+6+j].c_str());
                // ee_local_trans.local_trans.linear() =
                //     Eigen::AngleAxisd(rotation_vector[3],
                //                       cnoid::Vector3(rotation_vector[0],
                //                                      rotation_vector[1],
                //                                      rotation_vector[2])).toRotationMatrix();
                ee_local_trans.local_trans.linear() =
                    Eigen::AngleAxisd(stod(end_effectors_str[i*prop_num+6+3]),
                                      cnoid::Vector3(stod(end_effectors_str[i*prop_num+6]),
                                                     stod(end_effectors_str[i*prop_num+6+1]),
                                                     stod(end_effectors_str[i*prop_num+6+2]))).toRotationMatrix();
            }

            ee_local_trans.target_name = ee_target;
            ee_local_trans.ee_path = std::make_shared<cnoid::JointPath>(ioBody->rootLink(), ioBody->link(ee_target));

            {
                bool is_ee_exists = false;
                // TODO: virtual force sensor
                for (const auto& fsensor : fsensors) {
                    cnoid::Link* alink = ioBody->link(ee_target);
                    while (alink != NULL && alink->name() != ee_base && !is_ee_exists) {
                        if (alink->name() == fsensor->link()->name()) {
                            is_ee_exists = true;
                            ee_local_trans.sensor_name = fsensor->name();
                        }
                        alink = alink->parent();
                    }
                }
            }
            ee_trans_vec.push_back(ee_local_trans);

            std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "] " << ee_target << " " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   target = " << ee_target << ", base = " << ee_base << ", sensor_name = " << ee_local_trans.sensor_name << std::endl;
            // std::cerr << "[" << m_profile.instance_name << "]   localPos = " << ee_local_trans.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            // std::cerr << "[" << m_profile.instance_name << "]   localR = " << ee_local_trans.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        }
    }

    // Reserve IK Parameter Vector
    ik_params.reserve(end_effectors_str.size() + 2); // EE + COM Pos + COM Ang Vel
    setIKLimbAsDefault();

    // Resize joint vector
    {
        const size_t num_joints = ioBody->numJoints();
        q_prev.resize(num_joints);
        m_qCurrent.data.length(num_joints);
        m_qRef.data.length(num_joints);
    }

    return RTC::RTC_OK;
}

RTC::ReturnCode_t Balancer::onFinalize()
{
    std::cerr << "[" << m_profile.instance_name << "] onFinalize()" << std::endl;
    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Balancer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Balancer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Balancer::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Balancer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}


// #define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t Balancer::onExecute(RTC::UniqueId ec_id)
{
    ++loop;
    readInPortData();

    if (control_mode >= SYNC_TO_NOCONTROL) getCurrentStates();

    if (control_mode == SYNC_TO_NOCONTROL) {
        if (loop > control_endcount) control_mode = NOCONTROL;
        {
            size_t i = 0;
            for (auto joint : ioBody->joints()) {
                joint->q() = calcNthOrderSpline(spline_coeff_vec[i], (loop - control_startcount) * m_dt);
                ++i;
            }
        }
    } else if (control_mode == JUMPING) {
        if (loop > control_endcount) control_mode = IDLE;
        target_com_pos = ioBody->centerOfMass();
        // std::cerr << "cur com z: " << target_com_pos[2] << std::endl;
        target_com_pos[2] = calcNthOrderSpline(spline_coeff, (loop - control_startcount) * m_dt);
        // std::cerr << "ref com z: " << target_com_pos[2] << std::endl;
        target_com_ang_vel = cnoid::Vector3::Zero();
        for (size_t i = 0; i < target_ee.size(); ++i) {
            target_ee[i].translation() = ee_trans_vec[i].ee_path->endLink()->p();
            target_ee[i].linear() = ee_trans_vec[i].ee_path->endLink()->R();
        }

        // std::cerr << "root_t1: \n" << ioBody->rootLink()->T().matrix() << std::endl;

        // solveWeightedWholebodyIK(&(ioBody->rootLink()->T()), ioBody->joints(),
        //                          ik_params,
        //                          [this] () { this->ioBody->calcForwardKinematics(); this->ioBody->calcCenterOfMass();
        //                              std::cerr << "root_t2: \n" << this->ioBody->rootLink()->T().matrix() << std::endl;
        //                          },
        //                          1e-2, 10, 1e-1);
        // std::cerr << "q bef: ";
        // for (auto joint : ioBody->joints()) {
        //     std::cerr << joint->q() << ", ";
        // }
        // std::cerr << std::endl;
        solveWeightedWholebodyIK(&(ioBody->rootLink()->T()), ioBody->joints(),
                                 ik_params, [this]() { this->calcFK(); },
                                 1e-6, 1e-3, 100, 1e-1);
        // std::cerr << "q aft: ";
        // for (auto joint : ioBody->joints()) {
        //     std::cerr << joint->q() << ", ";
        // }
        // std::cerr << std::endl;

        // std::cerr << "root_t2: \n" << ioBody->rootLink()->T().matrix() << std::endl;
    }
    else if (control_mode == SQUAT) {
        static double target = ioBody->centerOfMass()[2];
        target_com_pos = ioBody->centerOfMass();
        std::cerr << "current com pos: " << target_com_pos[2] << std::endl;
        // target_com_pos[2] += 0.05 * sin(2 * M_PI * (loop - control_startcount + 500 * 2) / (500 * 4.0)) - 0.05; // 1 squat in 4 seconds
        // target_com_pos[2] += 0.05 * cos(2 * M_PI * (loop - control_startcount) / (500 * 4.0)) - 0.05; // 1 squat in 4 seconds
        // target_com_pos[2] += -0.07 * (2 * M_PI / (500 * 4.0)) * sin(2 * M_PI * (loop - control_startcount) / (500 * 4.0)); // 1 squat in 4 seconds
        target_com_pos[2] = target + 0.05 * cos(2 * M_PI * (loop - control_startcount) / (500 * 4.0)) - 0.05;
        std::cerr << "target  com pos: " << target_com_pos[2] << std::endl;
        target_com_ang_vel = cnoid::Vector3::Zero();
        for (size_t i = 0; i < target_ee.size(); ++i) {
            target_ee[i].translation() = ee_trans_vec[i].ee_path->endLink()->p();
            target_ee[i].linear() = ee_trans_vec[i].ee_path->endLink()->R();
        }
        solveWeightedWholebodyIK(&(ioBody->rootLink()->T()), ioBody->joints(),
                                 ik_params, [this]() { this->calcFK(); },
                                 1e-6, 1e-3, 100, 1e-1);
    }

    setCurrentStates();
    writeOutPortData();

    return RTC::RTC_OK;
}

inline void Balancer::readInPortData()
{
    // check dataport input
    if (m_basePosIn.isNew()) m_basePosIn.read();
    if (m_baseRpyIn.isNew()) m_baseRpyIn.read();
    if (m_rpyIn.isNew())     m_rpyIn.read();
    if (m_qRefIn.isNew())    m_qRefIn.read();
    for (size_t i = 0; i < m_forceIn.size(); i++) {
        if (m_forceIn[i]->isNew()) m_forceIn[i]->read();
    }

    //syncronize ioBody to the real robot
    // ioBody->rootLink()->p() = cnoid::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
    // ioBody->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);

    if (m_qRef.data.length() == ioBody->numJoints()) {
        size_t i = 0;
        // std::cerr << "m_q bef: ";
        for (auto joint : ioBody->joints()) {
            joint->q() = m_qRef.data[i++];
            // std::cerr << m_qRef.data[i - 1] << " ";
        }
        // std::cerr << std::endl;
    }

    // Get force sensor values
    // Force sensor's force value is absolute in reference frame
    auto fsensors = ioBody->devices<cnoid::ForceSensor>().getSortedById();
    for (size_t i = 0, n = m_force.size(); i < n; ++i) {
        act_force[i] = fsensors[i]->link()->R() * fsensors[i]->R_local() *
            cnoid::Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
    }
}

inline void Balancer::writeOutPortData()
{
    {
        size_t i = 0;
        for (auto joint : ioBody->joints()) {
            m_qRef.data[i++] = joint->q();
        }
        m_qRefOut.write();
    }

    m_basePos.tm = m_qRef.tm;
    m_basePos.data.x = ioBody->rootLink()->p().x();
    m_basePos.data.y = ioBody->rootLink()->p().x();
    m_basePos.data.z = ioBody->rootLink()->p().x();
}

void Balancer::getCurrentStates()
{
    {
        size_t i = 0;
        for (auto joint : ioBody->joints()) joint->q() = q_prev[i++];
    }

    calcFK();
}

inline void Balancer::setCurrentStates()
{
    {
        size_t i = 0;
        for (auto joint : ioBody->joints()) q_prev[i++] = joint->q();
    }
}

bool Balancer::startBalancer()
{
    control_mode = SYNC_TO_IDLE;
    std::cerr << "[" << m_profile.instance_name << "] start Balancer" << std::endl;
    return true;
}

bool Balancer::stopBalancer(const double migration_time)
{
    std::cerr << "[" << m_profile.instance_name << "] stop Balancer" << std::endl;

    control_mode = SYNC_TO_NOCONTROL;
    control_startcount = loop;
    control_endcount = loop + migration_time / m_dt;

    {
        const size_t num_joints = ioBody->numJoints();
        spline_coeff_vec.resize(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
            const cnoid::Vector3 start(q_prev[i], 0, 0); // pos, vel, acc
            const cnoid::Vector3 finish(m_qRef.data[i], 0, 0);
            spline_coeff_vec[i] = calcMinJerkCoeff(start, finish, migration_time);
        }
    }

    return true;
}

bool Balancer::startJump(const double height, const double squat)
{
    control_mode = JUMPING;

    const cnoid::Vector3 startPos = ioBody->calcCenterOfMass();
    const cnoid::Vector3 start(startPos[2], 0, 0); // pos, vel, acc
    const cnoid::Vector3 finish(startPos[2] + 0.15, sqrt(2 * -G_ACC * height), G_ACC);
    const minJerkCoeffTime coeff_time = calcMinJerkCoeffWithTimeInitJerkZero(start, finish);
    std::copy(coeff_time.begin(), coeff_time.end() - 1, spline_coeff.begin()); // Remove time from coeff_time

    control_startcount = loop;
    control_endcount = loop + coeff_time.back() / m_dt;

    // auto calcCOM = std::bind(calcNthOrderSpline, coeff, std::placeholders::_1);
    // calc = calcNthOrderSpline(coeff); // curry

    // setCOMCalcRule(std::bind(jumpControl, coeff), T);
    return true;
}

bool Balancer::startSquat()
{
    control_mode = SQUAT;
    control_startcount = loop;
    ik_params[1].IK_weight *= 0; // CoM Angular Momentum
    return true;
}

bool Balancer::stopSquat()
{
    control_mode = IDLE;
    return true;
}

inline void Balancer::calcFK()
{
    ioBody->calcForwardKinematics();
    ioBody->calcCenterOfMass();
}

bool Balancer::setBalancerParam(const OpenHRP::BalancerService::BalancerParam& i_param)
{
    return true;
}

bool Balancer::getBalancerParam(OpenHRP::BalancerService::BalancerParam_out i_param)
{
    return true;
}

bool Balancer::setIKLimbAsDefault()
{
    {
        const size_t cap = ik_params.capacity();
        ik_params.clear();
        ik_params.reserve(cap);
    }

    // COM Position
    {
        IKParam com_ik(ik_trans);
        // com_ik.calcJacobian = [ioBodyWeak = BodyPtrWeak(ioBody)]()
        com_ik.calcJacobian = [&]()
            {
                Eigen::MatrixXd J;
                cnoid::calcCMJacobian(ioBody, nullptr, J); // RMC Eq.1 upper [M/m E -r]
                // std::cerr << "J:\n" << J << std::endl;
                return std::move(J);
            };
        com_ik.calcError = [&]()
            {
                return target_com_pos - ioBody->centerOfMass();
            };
        ik_params.emplace_back(std::move(com_ik));
    }

    // COM Angular Velocity
    {
        IKParam com_ang(ik_axisrot); // Angular Vel
        com_ang.calcJacobian = [&]()
            {
                Eigen::MatrixXd J;
                cnoid::calcAngularMomentumJacobian(ioBody, nullptr, J); // RMC Eq.1 lower [H 0 I]
                // std::cerr << "J:\n" << J << std::endl;
                return std::move(J);
            };
        com_ang.calcError = [&]()
            {
                cnoid::Vector3 tmp_P, L;
                ioBody->calcTotalMomentum(tmp_P, L);
                return target_com_ang_vel - L;
            };
        ik_params.emplace_back(std::move(com_ang));
    }

    // End-effector IK parameters
    for (size_t i = 0; i < target_ee.size(); ++i) {
        IKParam ee_ikparam(ik_3daffine);
        ee_ikparam.calcJacobian = [&, i]()
            {
                auto ee_path = ee_trans_vec[i].ee_path;
                const size_t path_numjoints = ee_path->numJoints();

                Eigen::MatrixXd J_path(6, path_numjoints);
                cnoid::setJacobian<0x3f, 0, 0>(*ee_path, ee_path->endLink(), J_path);

                Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6 + ioBody->numJoints());
                for (size_t idx = 0; idx < path_numjoints; ++idx) {
                    J.col(ee_path->joint(idx)->jointId()) = std::move(J_path.col(idx));
                }

                Eigen::MatrixXd J_bodypart(6, 6);
                J_bodypart << Eigen::Matrix3d::Identity(), cnoid::hat(ioBody->rootLink()->p() - ee_path->endLink()->p()),
                              Eigen::Matrix3d::Zero(),     Eigen::Matrix3d::Identity();
                J.block<6, 6>(0, ioBody->numJoints()) = std::move(J_bodypart);

                // std::cerr << "J:\n" << J << std::endl;
                return std::move(J);
            };
        ee_ikparam.calcError = [&, i]()
            {
                cnoid::Vector6 error;
                error.head<3>() = target_ee[i].translation() - ee_trans_vec[i].ee_path->endLink()->p();
                error.segment<3>(3) = target_ee[i].linear() * cnoid::omegaFromRot(target_ee[i].linear() * ee_trans_vec[i].ee_path->endLink()->R().transpose());
                return std::move(error);
            };
        ik_params.emplace_back(std::move(ee_ikparam));
    }

    return true;
}

void Balancer::calcFootOriginCoords (cnoid::Vector3& foot_origin_pos, cnoid::Matrix3& foot_origin_rot)
{
    // rats::coordinates leg_c[2], tmpc;
    // cnoid::Vector3 ez = cnoid::Vector3::UnitZ();
    // cnoid::Vector3 ex = cnoid::Vector3::UnitX();
    // size_t i = 0;
    // // for (auto itr = ee_map.begin(); itr != ee_map.end(); itr++ ) {
    // //     if (itr->first.find("leg") == std::string::npos) continue;
    // //     hrp::Link* target = ioBody->sensor<hrp::ForceSensor>(itr->second.sensor_name)->link;
    // //     leg_c[i].pos = target->p;
    // //     hrp::Vector3 xv1(target->R * ex);
    // //     xv1(2)=0.0;
    // //     xv1.normalize();
    // //     hrp::Vector3 yv1(ez.cross(xv1));
    // //     leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
    // //     leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
    // //     leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);
    // //     i++;
    // // }
    // // Only double support is assumed
    // // void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2, const double eps) {
    // //     mid_coords.pos = (1 - p) * c1.pos + p * c2.pos;
    // //     mid_rot(mid_coords.rot, p, c1.rot, c2.rot, eps);
    // // };
    // // rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
    // foot_origin_pos = tmpc.pos;
    // foot_origin_rot = tmpc.rot;
}

/*
RTC::ReturnCode_t Balancer::onAborting(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Balancer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Balancer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Balancer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Balancer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C"
{
void BalancerInit(RTC::Manager* manager)
{
    RTC::Properties profile(Balancer_spec);
    manager->registerFactory(profile,
                             RTC::Create<Balancer>,
                             RTC::Delete<Balancer>);
}
};
