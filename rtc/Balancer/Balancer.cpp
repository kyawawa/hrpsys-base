// -*- C++ -*-
/*!
 * @file  Balancer.cpp
 * @brief Update ReferenceForce
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "hrpsys/util/Hrpsys.h"
#include <boost/assign.hpp>
#include "Balancer.h"
#include "hrpsys/util/VectorConvert.h"

typedef coil::Guard<coil::Mutex> Guard;

namespace {
static inline bool eps_eq(const double a, const double b, const double eps = 1e-3) { return std::abs(a - b) <= eps; };
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
    m_BalancerServicePort("BalancerService"),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_rpyIn("rpy", m_rpy),
    m_qRefIn("qRef", m_qRef),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
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

    // Set service provider to Ports
    m_BalancerServicePort.registerProvider("service0", "BalancerService", m_BalancerService);

    // Set service consumers to Ports
    // Set CORBA Service Ports
    addPort(m_BalancerServicePort);

    loop = 0;

    // Get dt
    RTC::Properties& prop = getProperties(); // get properties information from .wrl file
    coil::stringTo(m_dt, prop["dt"].c_str());
    std::cerr << "m_dt: " << m_dt << std::endl;

    // Make m_robot instance
    m_robot = hrp::BodyPtr(new hrp::Body());
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    std::cerr << "nameServer: " << nameServer << std::endl;
    int comPos = nameServer.find(",");
    if (comPos < 0) comPos = nameServer.length();
    nameServer = nameServer.substr(0, comPos);
    std::cerr << "VRML: " << prop["model"] << std::endl;
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), // load robot model for m_robot
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext()))) {
        std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
        return RTC::RTC_ERROR;
    }

    // check if the dof of m_robot match the number of joint in m_robot
    unsigned dof = m_robot->numJoints();
    for (unsigned i = 0; i < dof; i++) {
        if (i != m_robot->joint(i)->jointId) {
            std::cerr << "[" << m_profile.instance_name << "] jointId is not equal to the index number" << std::endl;
            return RTC::RTC_ERROR;
        }
    }

    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    // find names for real force sensors
    unsigned num_physical_fsensors = m_robot->numSensors(hrp::Sensor::FORCE);
    for (int i = 0; i < num_physical_fsensors; i++) {
        fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    // load virtual force sensors
    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    unsigned num_virtual_fsensors = m_vfs.size();
    for (int i = 0; i < num_virtual_fsensors; i++) {
        for (auto it = m_vfs.begin(); it != m_vfs.end(); it++) {
            if (it->second.id == i) fsensor_names.push_back(it->first);
        }
    }
    // add ports for all force sensors
    unsigned num_fsensors  = num_physical_fsensors + num_virtual_fsensors;
    m_force.resize(num_fsensors);
    m_forceIn.resize(num_fsensors);
    std::cerr << "[" << m_profile.instance_name << "] create force sensor ports" << std::endl;
    for (unsigned i = 0; i < num_fsensors; ++i) {
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
        for (size_t i = 0; i < num; i++) {
            std::string ee_name, ee_target, ee_base;
            coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
            coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
            coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());

            ee_trans ee_local_trans;
            for (size_t j = 0; j < 3; j++) {
                coil::stringTo(ee_local_trans.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
            }
            {
                double rotation_vector[4]; // rotation in VRML is represented by axis + angle
                for (size_t j = 0; j < 4; ++j) coil::stringTo(rotation_vector[j], end_effectors_str[i*prop_num+6+j].c_str());
                ee_local_trans.localR = Eigen::AngleAxis<double>(
                    rotation_vector[3], hrp::Vector3(rotation_vector[0], rotation_vector[1], rotation_vector[2])).toRotationMatrix();
            }

            ee_local_trans.target_name = ee_target;
            {
                bool is_ee_exists = false;
                for (size_t j = 0; j < num_fsensors; j++) {
                    hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, j);
                    hrp::Link* alink = m_robot->link(ee_target);
                    while (alink != NULL && alink->name != ee_base && !is_ee_exists) {
                        if (alink->name == sensor->link->name) {
                            is_ee_exists = true;
                            ee_local_trans.sensor_name = sensor->name;
                        }
                        alink = alink->parent;
                    }
                }
            }
            ee_map.insert(std::pair<std::string, ee_trans>(ee_name , ee_local_trans));

            ee_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
            std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << ee_target << " " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   target = " << ee_target << ", base = " << ee_base << ", sensor_name = " << ee_local_trans.sensor_name << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localPos = " << ee_local_trans.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localR = " << ee_local_trans.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        }
    }

    hrp::Sensor* sensor = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sensor == NULL) {
        std::cerr << "[" << m_profile.instance_name
                  << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! "
                  << std::endl;
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


#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t Balancer::onExecute(RTC::UniqueId ec_id)
{
    ++loop;

    // check dataport input
    for (size_t i = 0; i < m_forceIn.size(); i++) {
        if (m_forceIn[i]->isNew()) {
            m_forceIn[i]->read();
        }
    }
    if (m_basePosIn.isNew()) {
        m_basePosIn.read();
    }
    if (m_baseRpyIn.isNew()) {
        m_baseRpyIn.read();
    }
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
    }
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
    }

    //syncronize m_robot to the real robot
    if (m_qRef.data.length() ==  m_robot->numJoints()) {
        Guard guard(m_mutex);

        // Get current states
        getCurrentStates();

        // Get force sensor values
        // Force sensor's force value is absolute in reference frame
        for (size_t i = 0; i < m_force.size(); ++i) {
            hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, i);
            hrp::Vector3 act_force = (sensor->link->R * sensor->localR) *
                hrp::Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
        }
    }

    return RTC::RTC_OK;
}

bool Balancer::setBalancerParam(const OpenHRP::BalancerService::BalancerParam& i_param)
{
    return true;
};

bool Balancer::getBalancerParam(OpenHRP::BalancerService::BalancerParam_out i_param)
{
    return true;
};

void Balancer::getCurrentStates()
{
    // reference model
    for (unsigned i = 0; i < m_robot->numJoints(); i++) {
        m_robot->joint(i)->q = m_qRef.data[i];
    }
    m_robot->rootLink()->p = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
    m_robot->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    m_robot->calcForwardKinematics();

    hrp::Vector3 foot_origin_pos;
    calcFootOriginCoords(foot_origin_pos, foot_origin_rot);

    // for (size_t i = 0; i < ikparams.size(); i++) {
    //     hrp::Link* target = m_robot->link(ikparams[i].target_name);
    //     // target_ee_p[i] = target->p + target->R * ikparams[i].localp;
    //     // target_ee_R[i] = target->R * ikparams[i].localR;
    // }
}

void Balancer::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
  rats::coordinates leg_c[2], tmpc;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  size_t i = 0;
  for (std::map<std::string, ee_trans>::iterator itr = ee_map.begin(); itr != ee_map.end(); itr++ ) {
      if (itr->first.find("leg") == std::string::npos) continue;
      hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(itr->second.sensor_name)->link;
      leg_c[i].pos = target->p;
      hrp::Vector3 xv1(target->R * ex);
      xv1(2)=0.0;
      xv1.normalize();
      hrp::Vector3 yv1(ez.cross(xv1));
      leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
      leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
      leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);
      i++;
  }
  // Only double support is assumed
  rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
  foot_origin_pos = tmpc.pos;
  foot_origin_rot = tmpc.rot;
}

bool Balancer::startBalancer()
{
    return true;
};

bool Balancer::stopBalancer()
{
    return true;
};

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
