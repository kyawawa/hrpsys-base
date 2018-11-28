// -*- C++ -*-
/*!
 * @file  Balancer.h
 * @brief Balancer
 * @date  $Date$
 *
 * $Id$
 */

#ifndef REFERENCEFORCEUPDATOR_COMPONENT_H
#define REFERENCEFORCEUPDATOR_COMPONENT_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../SequencePlayer/interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
#include <boost/shared_ptr.hpp>

// #include "ImpedanceOutputGenerator.h"
// #include "ObjectTurnaroundDetector.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "BalancerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
*/
class Balancer
    : public RTC::DataFlowComponentBase
{
  public:
    /**
       \brief Constructor
       \param manager pointer to the Manager
    */
    Balancer(RTC::Manager* manager);
    /**
       \brief Destructor
    */
    virtual ~Balancer();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry()
    virtual RTC::ReturnCode_t onInitialize();

    // The finalize action (on ALIVE->END transition)
    // formaer rtc_exiting_entry()
    virtual RTC::ReturnCode_t onFinalize();

    // The startup action when ExecutionContext startup
    // former rtc_starting_entry()
    // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

    // The shutdown action when ExecutionContext stop
    // former rtc_stopping_entry()
    // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

    // The activated action (Active state entry action)
    // former rtc_active_entry()
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    // The deactivated action (Active state exit action)
    // former rtc_active_exit()
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

    // The execution action that is invoked periodically
    // former rtc_active_do()
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    // The aborting action when main logic error occurred.
    // former rtc_aborting_entry()
    // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

    // The error action in ERROR state
    // former rtc_error_do()
    // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

    // The reset action that is invoked resetting
    // This is same but different the former rtc_init_entry()
    // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

    // The state update action that is invoked after onExecute() action
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

    // The action that is invoked when execution context's rate is changed
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

    bool setBalancerParam(const OpenHRP::BalancerService::BalancerParam& i_param);
    bool getBalancerParam(OpenHRP::BalancerService::BalancerParam_out i_param);
    void getCurrentStates();
    void calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot);
    bool startBalancer();
    bool stopBalancer();

  protected:
    // Configuration variable declaration
    // <rtc-template block="config_declare">

    // </rtc-template>

    // DataInPort declaration
    // <rtc-template block="inport_declare">
    RTC::TimedDoubleSeq m_qCurrent;
    RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
    RTC::TimedDoubleSeq m_qRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
    RTC::TimedPoint3D m_basePos;
    RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
    RTC::TimedOrientation3D m_baseRpy;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
    std::vector<RTC::TimedDoubleSeq> m_force;
    std::vector<RTC::InPort<RTC::TimedDoubleSeq> *> m_forceIn;
    std::vector<RTC::TimedDoubleSeq> m_ref_force_in;
    std::vector<RTC::InPort<RTC::TimedDoubleSeq> *> m_ref_forceIn;
    RTC::TimedOrientation3D m_rpy;
    RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;
    RTC::TimedPoint3D m_diffFootOriginExtMoment;
    // RTC::InPort<RTC::TimedPoint3D> m_diffFootOriginExtMomentIn;

    // </rtc-template>

    // DataOutPort declaration
    // <rtc-template block="outport_declare">

    // </rtc-template>

    // CORBA Port declaration
    // <rtc-template block="corbaport_declare">

    // </rtc-template>

    // Service declaration
    // <rtc-template block="service_declare">
    RTC::CorbaPort m_BalancerServicePort;

    // </rtc-template>

    // Consumer declaration
    // <rtc-template block="consumer_declare">
    BalancerService_impl m_BalancerService;

    // </rtc-template>

  private:
    unsigned m_debugLevel;
    coil::Mutex m_mutex;
    hrp::BodyPtr m_robot;
    double m_dt;
    unsigned loop; //counter in onExecute
    struct IKParam {
        std::string target_name; // Name of end link
        std::string ee_name; // Name of ee (e.g., rleg, lleg, ...)
        std::string sensor_name; // Name of force sensor in the limb
        std::string parent_name; // Name of parent ling in the limb
    };
    struct ee_trans {
        std::string target_name, sensor_name;
        hrp::Vector3 localPos;
        hrp::Matrix33 localR;
    };
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::map<std::string, ee_trans> ee_map;
    std::map<std::string, size_t> ee_index_map;
    std::vector<IKParam> ikparams;
    hrp::Matrix33 foot_origin_rot;
    bool use_sh_base_pos_rpy;
    const std::string footoriginextmoment_name, objextmoment0_name;
};


extern "C"
{
void BalancerInit(RTC::Manager* manager);
};

#endif // REFERENCEFORCEUPDATOR_COMPONENT_H
