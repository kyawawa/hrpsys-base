// -*- C++ -*-
/*!
 * @file  SequencePlayer.h
 * @brief sequence player component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef SEQUENCEPLAYER_H
#define SEQUENCEPLAYER_H

//define if you use velocity and angular velocity of root link
#define CALC_VEL_N_ANGVEL

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include "seqplay.h"

#ifdef CALC_VEL_N_ANGVEL
#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../RobotHardware/robot.h"
#endif

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "SequencePlayerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class SequencePlayer
  : public RTC::DataFlowComponentBase
{
 public:
  SequencePlayer(RTC::Manager* manager);
  virtual ~SequencePlayer();

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
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

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

  double dt;
  seqplay *player() { return m_seq; }
  hrp::BodyPtr robot() { return m_robot;}  
  void setClearFlag();
  void waitInterpolation();
  bool waitInterpolationOfGroup(const char *gname);
  bool setJointAngle(short id, double angle, double tm);
  bool setJointAngles(const double *angles, double tm);
  bool setJointAngles(const double *angles, const bool *mask, double tm);
  bool setJointAnglesSequence(const OpenHRP::dSequenceSequence angless, const OpenHRP::bSequence& mask, const OpenHRP::dSequence& times);
  bool setJointAnglesSequenceFull(const OpenHRP::dSequenceSequence i_jvss, const OpenHRP::dSequenceSequence i_vels, const OpenHRP::dSequenceSequence i_torques, const OpenHRP::dSequenceSequence i_poss, const OpenHRP::dSequenceSequence i_rpys, const OpenHRP::dSequenceSequence i_accs, const OpenHRP::dSequenceSequence i_zmps, const OpenHRP::dSequenceSequence i_wrenches, const OpenHRP::dSequenceSequence i_optionals, const dSequence i_tms);
  bool clearJointAngles();
  bool setBasePos(const double *pos, double tm);
  bool setBaseRpy(const double *rpy, double tm);
  bool setZmp(const double *zmp, double tm);
  bool setTargetPose(const char* gname, const double *xyz, const double *rpy, double tm, const char* frame_name);
  bool setWrenches(const double *wrenches, double tm);
  void loadPattern(const char *basename, double time); 
  void playPattern(const OpenHRP::dSequenceSequence& pos, const OpenHRP::dSequenceSequence& rpy, const OpenHRP::dSequenceSequence& zmp, const OpenHRP::dSequence& tm);
  bool setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_);
  bool setInitialState(double tm=0.0);
  bool addJointGroup(const char *gname, const OpenHRP::SequencePlayerService::StrSequence& jnames);
  bool removeJointGroup(const char *gname);
  bool setJointAnglesOfGroup(const char *gname, const OpenHRP::dSequence& jvs, double tm);
  bool setJointAnglesSequenceOfGroup(const char *gname, const OpenHRP::dSequenceSequence angless, const OpenHRP::dSequence& times);
    bool clearJointAnglesOfGroup(const char *gname);
  bool playPatternOfGroup(const char *gname, const OpenHRP::dSequenceSequence& pos, const OpenHRP::dSequence& tm);

  void setMaxIKError(double pos, double rot);
  void setMaxIKIteration(short iter);
#ifdef CALC_VEL_N_ANGVEL
  double calc_theta_hard_coded(double time);
  double calc_theta(double time, double angel);
  enum EMERGENCY_STEP_FLAG{
      OFF,
      WAITHING,
      EMERGENCY
  };
#endif
 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qInit;
  InPort<TimedDoubleSeq> m_qInitIn;
  TimedPoint3D m_basePosInit;
  InPort<TimedPoint3D> m_basePosInitIn;
  TimedOrientation3D m_baseRpyInit;
  InPort<TimedOrientation3D> m_baseRpyInitIn;
  TimedPoint3D m_zmpRefInit;
  InPort<TimedPoint3D> m_zmpRefInitIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;
  TimedDoubleSeq m_tqRef;
  OutPort<TimedDoubleSeq> m_tqRefOut;
  TimedPoint3D m_zmpRef;
  OutPort<TimedPoint3D> m_zmpRefOut;
  TimedAcceleration3D m_accRef;
  OutPort<TimedAcceleration3D> m_accRefOut;
  TimedPoint3D m_basePos;
  OutPort<TimedPoint3D> m_basePosOut;
  TimedOrientation3D m_baseRpy;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  std::vector<TimedDoubleSeq> m_wrenches;
  std::vector<OutPort<TimedDoubleSeq> *> m_wrenchesOut;
  TimedDoubleSeq m_optionalData;
  OutPort<TimedDoubleSeq> m_optionalDataOut;

#ifdef CALC_VEL_N_ANGVEL
    RTC::TimedLong m_emergency_step_flag;
    RTC::InPort<RTC::TimedLong> m_emergency_step_flagIn;
    RTC::TimedChar m_debug_show;
    RTC::InPort<RTC::TimedChar> m_debug_showIn;
    RTC::TimedOrientation3D m_rpy;
    RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;
    RTC::TimedDouble m_pgain_in;
    RTC::InPort<RTC::TimedDouble> m_pgain_inIn;
    RTC::TimedDouble m_pgain_out;
    RTC::OutPort<RTC::TimedDouble> m_pgain_outOut;
    RTC::InPort<RTC::TimedLong> m_target_speedIn;
    RTC::TimedLong m_target_speed;
    RTC::TimedDoubleSeq m_qCurrent;
    RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
    char once_emergency_step_flag;
    double test_qRef;
    double initial_qRef;
    double current_qRef;
    double time_for_calc_qRef;

#endif

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_SequencePlayerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  SequencePlayerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  seqplay *m_seq;
  bool m_clearFlag, m_waitFlag;
  boost::interprocess::interprocess_semaphore m_waitSem;
  hrp::BodyPtr m_robot;
  std::string m_gname;
  unsigned int m_debugLevel;
  int dummy;
  size_t optional_data_dim;
  coil::Mutex m_mutex;
  double m_error_pos, m_error_rot;
  short m_iteration;
#ifdef CALC_VEL_N_ANGVEL
  typedef struct log{
      long time;
      double angle_vector[12];
      double zmp[3];
      double emergency_time;
      double rest;
  }log_struct;
  log_struct log_data[7501];
  double interference[90][100][100][100];
  double angle_vector[91][3];
  char rpy_show;
  char ankle_test;
  string base_parent_name;
  string target_name;
  hrp::JointPathExPtr manip;
  hrp::Vector3 root_rpy;
  double start_av[12];
  hrp::Vector3 root_p;
  hrp::Matrix33 root_R;
  hrp::Vector3 foot_p;
  hrp::Matrix33 foot_R;
  double time;
  double time_for_lleg;
  double l;
  double yaw_angle;
  double yaw_angle_deg;
  double theta;
  double l_xy;
  double root_displacement;
  double step_angle;
  hrp::Vector3 pos_diff;
  hrp::Vector3 goal_p;
  hrp::Matrix33 goal_R;
  double lleg_av[6];
  double target_q[12];
  double prev_target[6];
  char testes; // test no tameno hennsuu desuyo, so do not use usually.  bool ja naikedo, u can switch this variable false/true from true/false by writing debug show #17
  FILE *fp_log;
  FILE *knee_speed_test_log;
  FILE *jump_test_log;
  long time_offset;
  double pgain_counter;
  double servo_pgain;
  char knee_speed_test;
  double start_position;
  double start_offset;
  double set_offset_count;
  const double theta_ad;
  const double theta_cons;
  double target_speed;
  long target_speed_deg;
  char jump_test;
  hrp::Vector3 start_p;
  hrp::Matrix33 start_R;
  hrp::Vector3 target_p;
  hrp::Matrix33 target_R;
  double target_root_speed;
  double upward_displacement;
  double downward_displacement;
  int bending_count;
#endif
};


extern "C"
{
  void SequencePlayerInit(RTC::Manager* manager);
};

#endif // SEQUENCEPLAYER_H
