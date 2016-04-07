// -*- C++ -*-
/*!
 * @file  SequencePlayer.cpp
 * @brief sequence player component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include "SequencePlayer.h"
#include "util/VectorConvert.h"
//#include <hrpModel/JointPath.h>
//#include <hrpUtil/MatrixSolvers.h>
//#include "../ImpedanceController/JointPathEx.h"

//#define CROSS_TIME 750
//#define CROSS_TIME 5000
#define CROSS_TIME 500
#define STEP_ANGLE 45.0

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* sequenceplayer_spec[] =
    {
        "implementation_id", "SequencePlayer",
        "type_name",         "SequencePlayer",
        "description",       "sequence player component",
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

SequencePlayer::SequencePlayer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qInitIn("qInit", m_qInit),
      m_basePosInitIn("basePosInit", m_basePosInit),
      m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
      m_zmpRefInitIn("zmpRefInit", m_zmpRefInit),
      m_qRefOut("qRef", m_qRef),
      m_tqRefOut("tqRef", m_tqRef),
      m_zmpRefOut("zmpRef", m_zmpRef),
      m_accRefOut("accRef", m_accRef),
      m_basePosOut("basePos", m_basePos),
      m_baseRpyOut("baseRpy", m_baseRpy),
      m_optionalDataOut("optionalData", m_optionalData),
      m_SequencePlayerServicePort("SequencePlayerService"),
#ifdef CALC_VEL_N_ANGVEL
      //added by karasawa for chidori
      m_emergency_step_flagIn("emergency_step_flag", m_emergency_step_flag),
      m_debug_showIn("debug_show", m_debug_show),
      m_pgain_inIn("pgain_in", m_pgain_in),
      m_pgain_outOut("pgain_out", m_pgain_out),
      m_rpyIn("rpy", m_rpy),
      m_target_speedIn("target_speed", m_target_speed),
      m_qCurrentIn("qCurrent", m_qCurrent),
      rpy_show(0),
      ankle_test(0),
      testes(0),
      knee_speed_test(0),
      theta_ad(60.0*3.141592/180.0),
      theta_cons(30.0*3.141592/180.0),
      target_speed(0.0),
#endif
      // </rtc-template>
      m_waitSem(0),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      m_error_pos(0.0001),
      m_error_rot(0.001),
      m_iteration(50),
      dummy(0)
{
    m_service0.player(this);
    m_clearFlag = false;
    m_waitFlag = false;
}

SequencePlayer::~SequencePlayer()
{
}


RTC::ReturnCode_t SequencePlayer::onInitialize()
{
    std::cout << "SequencePlayer::onInitialize()" << std::endl;
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qInit", m_qInitIn);
    addInPort("basePosInit", m_basePosInitIn);
    addInPort("baseRpyInit", m_baseRpyInitIn);
    addInPort("zmpRefInit", m_zmpRefInitIn);

    // Set OutPort buffer
    addOutPort("qRef", m_qRefOut);
    addOutPort("tqRef", m_tqRefOut);
    addOutPort("zmpRef", m_zmpRefOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("basePos", m_basePosOut);
    addOutPort("baseRpy", m_baseRpyOut);
    addOutPort("optionalData", m_optionalDataOut);

#ifdef CALC_VEL_N_ANGVEL
    //added by karasawa for chidori
    addInPort("emergency_step_flag", m_emergency_step_flagIn);
    addInPort("debug_show", m_debug_showIn);
    addInPort("rpy", m_rpyIn);
    addInPort("pgain_in", m_pgain_inIn);
    addOutPort("pgain_out", m_pgain_outOut);
    addInPort("target_speed", m_target_speedIn);
    addInPort("qCurrent", m_qCurrentIn);
    m_emergency_step_flag.data = false;
    m_debug_show.data = 0;
    m_pgain_in.data = 0.0;
    m_pgain_out.data = 0.0;
    once_emergency_step_flag = SequencePlayer::OFF;
#endif

    // Set service provider to Ports
    m_SequencePlayerServicePort.registerProvider("service0", "SequencePlayerService", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_SequencePlayerServicePort);

    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable

    bindParameter("debugLevel", m_debugLevel, "0");
    // </rtc-template>

    RTC::Properties& prop = getProperties();
    coil::stringTo(dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new Body());

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                 )){
        std::cerr << "failed to load model[" << prop["model"] << "]"
                  << std::endl;
    }

    unsigned int dof = m_robot->numJoints();


    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    for (unsigned int i=0; i<npforce; i++){
      fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    //   find names for virtual force sensors
    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    int nvforce = virtual_force_sensor.size()/10;
    for (unsigned int i=0; i<nvforce; i++){
      fsensor_names.push_back(virtual_force_sensor[i*10+0]);
    }
    //   add ports for all force sensors
    int nforce  = npforce + nvforce;
    m_wrenches.resize(nforce);
    m_wrenchesOut.resize(nforce);
    for (unsigned int i=0; i<nforce; i++){
      m_wrenchesOut[i] = new OutPort<TimedDoubleSeq>(std::string(fsensor_names[i]+"Ref").c_str(), m_wrenches[i]);
      m_wrenches[i].data.length(6);
      registerOutPort(std::string(fsensor_names[i]+"Ref").c_str(), *m_wrenchesOut[i]);
    }

    if (prop.hasKey("seq_optional_data_dim")) {
        coil::stringTo(optional_data_dim, prop["seq_optional_data_dim"].c_str());
    } else {
        optional_data_dim = 1;
    }

    m_seq = new seqplay(dof, dt, nforce, optional_data_dim);

    m_qInit.data.length(dof);
    for (unsigned int i=0; i<dof; i++) m_qInit.data[i] = 0.0;
    Link *root = m_robot->rootLink();
    m_basePosInit.data.x = root->p[0]; m_basePosInit.data.y = root->p[1]; m_basePosInit.data.z = root->p[2];
    hrp::Vector3 rpy = hrp::rpyFromRot(root->R);
    m_baseRpyInit.data.r = rpy[0]; m_baseRpyInit.data.p = rpy[1]; m_baseRpyInit.data.y = rpy[2];
    m_zmpRefInit.data.x = 0; m_zmpRefInit.data.y = 0; m_zmpRefInit.data.z = 0;

#ifdef CALC_VEL_N_ANGVEL
    m_qCurrent.data.length(m_robot->numJoints());
    FILE *fp_angle_vector;

    std::cerr << "optional_data_dim == " << optional_data_dim << std::endl;
    std::cerr << "file open start" << std::endl;
    std::string angle_vector_file("/home/leus/generated_angle_vector.csv");
    if((fp_angle_vector = std::fopen(angle_vector_file.c_str(), "r")) == NULL) {
        std::cerr << "\x1b[31m" << "[seq]  FILE OPEN ERROR!! ---> " << angle_vector_file << "\x1b[0m" << std::endl;
    } else {
        std::cerr << "[seq]  ANGLE_VECTOR FILE OPEN SUCCEEDED" << std::endl;
        double crotch_yaw;
        double crotch_pitch;
        double knee_pitch;
        std::cerr << "[seq]  load file start--------------------------------------------------------" << std::endl;
        for(int i = 0; i < 91; i++) {
            std::fscanf(fp_angle_vector,"%lf,%lf,%lf", &crotch_yaw, &crotch_pitch, &knee_pitch);
            //std::fscanf(fp,"%*[^123456789]");
            angle_vector[i][0] = (crotch_yaw*3.14159265)/180.0;
            angle_vector[i][1] = (crotch_pitch*3.14159265)/180.0;
            angle_vector[i][2] = (knee_pitch*3.14159265)/180.0;
            std::cerr << angle_vector[i][0] << "  " << angle_vector[i][1] << "  " << angle_vector[i][2] << std::endl;
        }
        std::cerr << "[seq] load file complete------------------------------------------------------" << std::endl;
    }
    fclose(fp_angle_vector);

    std::string log_file_name("/home/leus/sequenceplayer_log.csv");
    if( (fp_log = std::fopen(log_file_name.c_str(), "w")) == NULL) {
        std::cerr << "\x1b[31m" << "[seq]  FILE OPEN ERROR!! ---> " << log_file_name << "\x1b[0m" << std::endl;
    } else {
        std::cerr << "\x1b[31m" << "[seq]  LOG FILE OPEN SUCCEEDED ---> " << log_file_name << "\x1b[0m" << std::endl;
    }
    fprintf(fp_log, "time_usec,qRef[0],qRef[1],qRef[2],qRef[3],qRef[4],qRef[5],qRef[6],qRef[7],qRef[8],qRef[9],qRef[10],qRef[11],zmp[0],zmp[1],zmp[2],emergency_time\n");

    std::cerr << "file open end" << std::endl;
#endif

    // allocate memory for outPorts
    m_qRef.data.length(dof);
    m_tqRef.data.length(dof);
    m_optionalData.data.length(optional_data_dim);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SequencePlayer::onFinalize()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
  */

/*
  RTC::ReturnCode_t SequencePlayer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
  */

RTC::ReturnCode_t SequencePlayer::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "SequencePlayer::onActivated(" << ec_id << ")" << std::endl;

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onDeactivated(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
  */

RTC::ReturnCode_t SequencePlayer::onExecute(RTC::UniqueId ec_id)
{
    static int loop = 0;
#ifdef CALC_VEL_N_ANGVEL
    static int emergency_step_loop = 0;
    static int emergency_step_waiting_loop = 0;
    static int log_file_idx = 0;
    static int knee_speed_test_loop = 0;
    static int jump_test_loop = 0;

    if (loop == 6000) {//wait 12 [sec] for initializing other components
        std::vector<int> indices;
        char group_name[10] = "rleg";
        if(! m_seq->getJointGroup(group_name, indices) ) {
            std::cerr << "Could not find joint group" << group_name << std::endl;
        }
        base_parent_name = m_robot->joint(indices[0])->parent->name;
        target_name = m_robot->joint(indices[indices.size()-1])->name;
        manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(base_parent_name), m_robot->link(target_name), dt));
        manip->setMaxIKError(m_error_pos, m_error_rot);
        manip->setMaxIKIteration(m_iteration);
        std::cerr << "\x1b[31m" << "[seq]  initialization for IK completed" << "\x1b[0m" << std::endl;
    }
#endif
    loop++;
    if ( m_debugLevel > 0 && loop % 1000 == 0) {
        std::cerr << __PRETTY_FUNCTION__ << "(" << ec_id << ")" << std::endl;
    }
    if (m_qInitIn.isNew()) m_qInitIn.read();
    if (m_basePosInitIn.isNew()) m_basePosInitIn.read();
    if (m_baseRpyInitIn.isNew()) m_baseRpyInitIn.read();
    if (m_zmpRefInitIn.isNew()) m_zmpRefInitIn.read();
#ifdef CALC_VEL_N_ANGVEL
    if (m_emergency_step_flagIn.isNew()) m_emergency_step_flagIn.read();
    if (m_debug_showIn.isNew()) m_debug_showIn.read();
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
        root_rpy(0) = m_rpy.data.r - 3.14159265/2.0;
        root_rpy(1) = m_rpy.data.p;
        root_rpy(2) = 0;//m_rpy.data.y;
    }
    if (m_target_speedIn.isNew()) {
        m_target_speedIn.read();
        double tmp = m_target_speed.data * 3.141592 / 180.0;
        if(tmp > 0) {
            target_speed = tmp;
            target_speed_deg = m_target_speed.data;
            //std::cerr << "[seq]target_speed :" << target_speed << std::endl;
        } else {

        }
    }
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
    }
#endif

#ifdef CALC_VEL_N_ANGVEL
    if(rpy_show) {
        if( (loop%250) == 0) {
            std::cerr << root_rpy(0) << "  " << root_rpy(1) << "  " << root_rpy(2) << std::endl;
        }
    }
#endif

    if (m_gname != "" && m_seq->isEmpty(m_gname.c_str())){
        if (m_waitFlag){
            m_gname = "";
            m_waitFlag = false;
            m_waitSem.post();
        }
    }
#ifdef CALC_VEL_N_ANGVEL
    if(m_debug_show.data) {
        hrp::Vector3 p;
        hrp::Matrix33 R;
        switch (m_debug_show.data) {
        case 1: // display p and R of root-link
            p = m_robot->rootLink()->p;
            R = m_robot->rootLink()->R;
            std::cerr << "---root-link---" << std::endl;
            std::cerr << p << std::endl;
            std::cerr << R << std::endl;
            break;
        case 2: // display each joint's p and R (joint-num = number - 2)
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            p = m_robot->link(m_robot->joint((m_debug_show.data)-2)->name)->p;
            R = m_robot->link(m_robot->joint((m_debug_show.data)-2)->name)->R;
            std::cerr << "---" << "\x1b[31m" << m_robot->joint((m_debug_show.data)-2)->name << "\x1b[0m" << "---" << std::endl;
            std::cerr << "----position----" << std::endl;
            std::cerr << p << std::endl;
            std::cerr << "----------------" << std::endl;
            std::cerr << "----rotation----" << std::endl;
            std::cerr << R << std::endl;
            std::cerr << "----------------" << std::endl;
            break;
        case 14: // solve IK and display the solution
            {
                hrp::Vector3 root_p(m_robot->link(base_parent_name)->p);
                hrp::Matrix33 root_R(m_robot->link(base_parent_name)->R);
                hrp::Vector3 foot_p(m_robot->link(target_name)->p);
                hrp::Matrix33 foot_R(m_robot->link(target_name)->R);
                //double l = sqrt( pow(0.1, 2) + pow(0.823285, 2) );//sqrt( pow( (root_p(1)-foot_p(1)), 2) + pow( (root_p(2)-foot_p(2)), 2) );// foot length

                ////-----  display l(foot-length), and default p and R of root-link and foot -----
                //std::cerr << "foot legth :" << l << std::endl;
                /*
                 * std::cerr << "root link = " << base_parent_name << std::endl;
                 * std::cerr << "----root_position----" << std::endl;
                 * std::cerr << root_p << std::endl;
                 * std::cerr << "----------------" << std::endl;
                 * std::cerr << "----root_rotation----" << std::endl;
                 * std::cerr << root_R << std::endl;
                 * std::cerr << "----------------" << std::endl;
                 * std::cerr << "foot link = " << target_name << std::endl;
                 * std::cerr << "----foot_position----" << std::endl;
                 * std::cerr << foot_p << std::endl;
                 * std::cerr << "----------------" << std::endl;
                 * std::cerr << "----foot_rotation----" << std::endl;
                 * std::cerr << foot_R << std::endl;
                 * std::cerr << "----------------" << std::endl;
                 */

                for(int i = 0; i < 101; i++) {
                    double time = 0.01*i;
                    double l = 0.8293360 + 0.015*( 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3) );
                    double yaw_angle = (3.14159265/2)*( 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3) );
                    double theta = calc_theta_hard_coded(i*0.01);
                    double l_xy = -l*sin(theta); // foot length => foot length mapped to xy plane
                    std::cerr << "\x1b[31m" << "time :" << std::setw(5) << (i*0.01) << "  theta :" << std::setw(10) << theta << "  yaw_angle:" << yaw_angle << "\x1b[0m" << endl;
                    double x = l_xy*sin(yaw_angle);
                    double y = -l_xy*cos(yaw_angle);
                    double z = l*cos(theta);
                    ////-----  display x, y, z  -----
                    //std::cerr << "     :     " << "      x :" << std::setw(7) << x << "  y :" << std::setw(7) << y << "  z :" << std::setw(7) << z << "  l:" << l << "  yaw_angle:" << yaw_angle <<  endl;
                    hrp::Vector3 goal_p;
                    goal_p(0) = foot_p(0) + x;
                    goal_p(1) = y;
                    goal_p(2) = 1.0-z;
                    hrp::Matrix33 goal_R = hrp::rotFromRpy(0.0, 0.0, yaw_angle);

                    ////-----  display goal p and R  -----
                    /*
                     * std::cerr << "----goal_position----" << std::endl;
                     * std::cerr << goal_p << std::endl;
                     * std::cerr << "----------------" << std::endl;
                     * std::cerr << "----goal_rotation----" << std::endl;
                     * std::cerr << goal_R << std::endl;
                     * std::cerr << "----------------" << std::endl;
                     */
                    if( manip->calcInverseKinematics2(goal_p, goal_R) ) {
                        std::cerr << "[SequencePlayer] solved av" << std::endl;
                        std::cerr << "                 ";
                        for( int i = 0; i < manip->numJoints(); i++) {
                            std::cerr << ((manip->joint(i)->q)*180/3.14159265) << " , ";
                        }
                        std::cerr << std::endl;
                    } else {
                        std::cerr << "\x1b[33m" << "[SequencePlayer] failed to solve IK" << "\x1b[0m" << endl;
                    }
                }
            }
            break;
        case 15: // switch display rpy angles of root-link or not
            if(rpy_show == false) {
                rpy_show = true;
            } else {
                rpy_show = false;
            }
            break;
        case 16:
            if(emergency_step_loop == 0) {
                if(ankle_test == false) {
                    ankle_test = true;
                } else {
                    ankle_test = false;
                }
            } else {
                std::cerr << "\x1b[31m" << "[seq]  Error!! emergency_step_mode is ON  (try after off the mode)" << "\x1b[0m" << std::endl;
            }
            break;
        case 17:
            if(testes == false) {
                testes = true;
            } else {
                testes = false;
            }
            break;
        case 20:
            if(knee_speed_test == false) {
                knee_speed_test = true;
            }
            break;
        case 21:
            if(jump_test == false) {
                jump_test = true;
            }
            break;
        default:
            break;
        }
        m_debug_show.data = 0;
    }

    if ((m_emergency_step_flag.data != SequencePlayer::OFF) || (once_emergency_step_flag != SequencePlayer::OFF)) {
        if (m_emergency_step_flag.data == SequencePlayer::WAITHING && (once_emergency_step_flag != SequencePlayer::EMERGENCY)) {
            if(emergency_step_waiting_loop == 0) {
                std::cerr << "\x1b[31m" << "start waiting mode" << "\x1b[0m" << std::endl;
                once_emergency_step_flag = SequencePlayer::WAITHING;
            }
            time_offset = m_qInit.tm.sec;
            emergency_step_waiting_loop++;
            Guard guard(m_mutex);

            double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
            double angle_vector[12];
            double time;
            if (emergency_step_waiting_loop < 1000) {
                time = emergency_step_waiting_loop/1000.0;
            } else {
                time = 1.0;
            }
            double time_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);
            double t;
            angle_vector[0] = 0.0;
            angle_vector[1] = 0.0;
            angle_vector[2] = -0.34906585039;
            angle_vector[3] = 0.69813170079;
            angle_vector[4] = -0.34906585039;
            angle_vector[5] = 0.0;
            angle_vector[6] = 0.0;
            angle_vector[7] = 0.0;
            angle_vector[8] = -0.34906585039;
            angle_vector[9] = 0.69813170079;
            angle_vector[10] = -0.34906585039;
            angle_vector[11] = 0.0;
            m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());

            m_qRef.data[0] = m_qRef.data[0] - time_coefficient*(m_qRef.data[0] - angle_vector[0]);
            m_qRef.data[1] = m_qRef.data[1] - time_coefficient*(m_qRef.data[1] - angle_vector[1]);
            m_qRef.data[2] = m_qRef.data[2] - time_coefficient*(m_qRef.data[2] - angle_vector[2]);
            m_qRef.data[3] = m_qRef.data[3] - time_coefficient*(m_qRef.data[3] - angle_vector[3]);
            m_qRef.data[4] = m_qRef.data[4] - time_coefficient*(m_qRef.data[4] - angle_vector[4]);
            m_qRef.data[5] = m_qRef.data[5] - time_coefficient*(m_qRef.data[5] - angle_vector[5]);
            m_qRef.data[6] = m_qRef.data[6] - time_coefficient*(m_qRef.data[6] - angle_vector[6]);
            m_qRef.data[7] = m_qRef.data[7] - time_coefficient*(m_qRef.data[7] - angle_vector[7]);
            m_qRef.data[8] = m_qRef.data[8] - time_coefficient*(m_qRef.data[8] - angle_vector[8]);
            m_qRef.data[9] = m_qRef.data[9] - time_coefficient*(m_qRef.data[9] - angle_vector[9]);
            m_qRef.data[10] = m_qRef.data[10] - time_coefficient*(m_qRef.data[10] - angle_vector[10]);
            m_qRef.data[11] = m_qRef.data[11] - time_coefficient*(m_qRef.data[11] - angle_vector[11]);


            m_zmpRef.data.x = 0.0;
            m_zmpRef.data.y = 0.0;
            m_zmpRef.data.z = -0.933285;
            m_accRef.data.ax = acc[0];
            m_accRef.data.ay = acc[1];
            m_accRef.data.az = acc[2];
            m_basePos.data.x = pos[0];
            m_basePos.data.y = pos[1];
            m_basePos.data.z = pos[2];
            m_baseRpy.data.r = rpy[0];
            m_baseRpy.data.p = rpy[1];
            m_baseRpy.data.y = rpy[2];
            size_t force_i = 0;
            for (size_t i = 0; i < m_wrenches.size(); i++) {
                m_wrenches[i].data[0] = wrenches[force_i++];
                m_wrenches[i].data[1] = wrenches[force_i++];
                m_wrenches[i].data[2] = wrenches[force_i++];
                m_wrenches[i].data[3] = wrenches[force_i++];
                m_wrenches[i].data[4] = wrenches[force_i++];
                m_wrenches[i].data[5] = wrenches[force_i++];
            }
            m_optionalData.data[0] = 1.0;
            m_optionalData.data[1] = 1.0;
            m_optionalData.data[2] = 1.0;
            m_optionalData.data[3] = 1.0;
            m_qRef.tm = m_qInit.tm;
            m_qRefOut.write();
            m_tqRefOut.write();
            m_zmpRefOut.write();
            m_accRefOut.write();
            m_basePosOut.write();
            m_baseRpyOut.write();
            m_optionalDataOut.write();
            for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
                m_wrenchesOut[i]->write();
            }

            if (m_clearFlag){
                m_seq->clear(0.001);
            }
        } else if ( (m_emergency_step_flag.data == SequencePlayer::EMERGENCY) || (once_emergency_step_flag == EMERGENCY)) {

            if (emergency_step_loop == 0) {
                emergency_step_waiting_loop = 0;
                once_emergency_step_flag = SequencePlayer::EMERGENCY;
                std::cerr << "m_qRef.data :";
                for(int i = 0; i < 12; i++) {
                    start_av[i] = m_qRef.data[i];
                    std::cerr << " " << std::setw(7) << std::left << m_qRef.data[i];
                }
                std::cerr << endl;

                root_p = m_robot->link(base_parent_name)->p;
                root_R = m_robot->link(base_parent_name)->R;
                foot_p = m_robot->link(target_name)->p;
                foot_R = m_robot->link(target_name)->R;

                std::cerr << "---- root ----" << std::endl;
                std::cerr << root_p << std::endl << std::endl;
                std::cerr << root_R << std::endl;
                std::cerr << "---- foot ----" << std::endl;
                std::cerr << foot_p << std::endl << std::endl;
                std::cerr << foot_R << std::endl;

                std::cerr << "[time] " << std::setw(5) << std::right << emergency_step_loop/100.0 << "  [angle] "  << std::setw(10) << std::left <<  m_qRef.data[10] << "  " << std::setw(10) << std::left << m_qRef.data[11] << std::endl;

            } //end if (emergency_step_loop == 0)

            emergency_step_loop++;

            Guard guard(m_mutex);

            double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
            double t;
            m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());

            if(ankle_test) {
                if(emergency_step_loop < 1000) {
                    m_qRef.data[10] = start_av[10] - ( emergency_step_loop*root_rpy(1) )/1000.0;// 2 [sec]かけて遷移
                    m_qRef.data[11] = start_av[11] - ( emergency_step_loop*root_rpy(0) )/1000.0;
                } else if(emergency_step_loop < 21000) {
                    m_qRef.data[10] = start_av[10] - root_rpy(1);
                    m_qRef.data[11] = start_av[11] - root_rpy(0);
                } else if(emergency_step_loop < 22000) {
                    m_qRef.data[10] = start_av[10] - ( (22000 - emergency_step_loop)*root_rpy(1) )/1000.0;// 2 [sec]かけて遷移
                    m_qRef.data[11] = start_av[11] - ( (22000 - emergency_step_loop)*root_rpy(0) )/1000.0;
                }
                if((emergency_step_loop%100) == 0) {
                    //            std::cerr << "[time] " << std::setw(5) << std::right << emergency_step_loop/100.0 << "  [angle] "  << std::setw(10) << std::left <<  m_qRef.data[10] << "  " << std::setw(10) << std::left << m_qRef.data[11] << std::endl;
                }
                if( emergency_step_loop > 22000) {
                    emergency_step_loop = 0;
                    once_emergency_step_flag = SequencePlayer::OFF;
                }
            } else {
                time = (1.0*emergency_step_loop)/((double)(CROSS_TIME));
                if(emergency_step_loop < (CROSS_TIME + 1) ) {
                    //time = (1.0*emergency_step_loop)/((double)(CROSS_TIME));
                    l = 0.8293360 + 0.015*( 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3) );
                    yaw_angle = (3.14159265/2)*( 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3) );
                    yaw_angle_deg = (yaw_angle*180.0)/3.14159265;
                    theta = calc_theta_hard_coded(time);
                    l_xy = -l*sin(theta); // foot length => foot length mapped to xy plane
                    pos_diff(0) = l_xy*sin(yaw_angle);
                    pos_diff(1) = -l_xy*cos(yaw_angle);
                    pos_diff(2) = l*cos(theta);
                    goal_p(0) = foot_p(0) + pos_diff(0);
                    goal_p(1) = pos_diff(1);
                    goal_p(2) = root_p(2) - pos_diff(2);
                    goal_R = hrp::rotFromRpy(0.0, 0.0, yaw_angle);
                    if( manip->calcInverseKinematics2(goal_p, goal_R) ) {
                        if(emergency_step_loop < CROSS_TIME) {
                            for(int i = 0; i < 6; i++) {
                                target_q[i] = prev_target[i] = manip->joint(i)->q;
                            }
                        } else {
                            for(int i = 0; i < 6; i++) {
                                target_q[i] = manip->joint(i)->q;
                            }
                        }
                    } else {
                        std::cerr << "\x1b[33m" << "[SequencePlayer] failed to solve IK" << "\x1b[0m" << endl;
                    }
                    if(time == 1) {
                        lleg_av[0] = 0;
                        lleg_av[1] = angle_vector[90][0];
                        lleg_av[2] = angle_vector[90][1];
                        lleg_av[3] = angle_vector[90][2];
                        //target_q[10] = lleg_av[4] = start_av[10] - root_rpy(1)*time;
                        //target_q[11] = lleg_av[5] = start_av[11] - root_rpy(0)*time;
                        target_q[10] = lleg_av[4] = -(angle_vector[90][1] + angle_vector[90][2]) - root_rpy(1)*time;
                        target_q[11] = lleg_av[5] = -angle_vector[90][0] - root_rpy(0)*time;
                    } else {
                        double a = yaw_angle_deg - (int)yaw_angle_deg;
                        double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);
                        lleg_av[0] = 0;
                        lleg_av[1] = (1.0 - a)*angle_vector[(int)yaw_angle_deg][0] + a*angle_vector[(int)yaw_angle_deg+1][0];
                        lleg_av[2] = (1.0 - a)*angle_vector[(int)yaw_angle_deg][1] + a*angle_vector[(int)yaw_angle_deg+1][1];
                        lleg_av[3] = (1.0 - a)*angle_vector[(int)yaw_angle_deg][2] + a*angle_vector[(int)yaw_angle_deg+1][2];
                        //lleg_av[4] = start_av[10] - root_rpy(1)*time;
                        //lleg_av[5] = start_av[11] - root_rpy(0)*time;
                        lleg_av[4] = start_av[10] - transition_coefficient*(start_av[10] + (angle_vector[90][1] + angle_vector[90][2])) - root_rpy(1)*time;
                        lleg_av[5] = start_av[11] - transition_coefficient*(start_av[11] + angle_vector[90][0]) - root_rpy(0)*time;

                    }
                } else {
                    if(emergency_step_loop < (CROSS_TIME + 26)) {
                        for(int i = 0; i < 6; i++) {
                            double tmp = target_q[i];
                            target_q[i] = target_q[i] + (2.0/3.0)*(target_q[i] - prev_target[i]);
                            prev_target[i] = tmp;
                        }
                    } else if(emergency_step_loop < (CROSS_TIME + 251)) {
                        for(int i = 0; i < 6; i++) {
                            double time = (emergency_step_loop - CROSS_TIME - 25)/225.0;
                            target_q[i] = prev_target[i] - (6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3))*(prev_target[i] - manip->joint(i)->q);
                        }
                    }
                    if(emergency_step_loop < (CROSS_TIME + 251)) {
                        double time = ( emergency_step_loop - CROSS_TIME )/250.0;//transition will complete in 2 [sec]
                        double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);
                        lleg_av[4] = target_q[10] - transition_coefficient*(target_q[10] + (angle_vector[90][1] + angle_vector[90][2]));//);start_av[10]);
                        lleg_av[5] = target_q[11] - transition_coefficient*(target_q[11] + angle_vector[90][0]);//);start_av[11]);
                    }
                    //emergency_step_loop = 0;
                    //once_emergency_step_flag = false;
                    //emergency_step_loop = 3000;
                }
            }
            zmp[0] = 0.0;//goal_p(0);
            zmp[1] = 0.0;//goal_p(1);
            zmp[2] = (goal_p(2) - 0.110);
            //update m_qRef
            //m_qRef.data[0] = current_qRef;
            /*
             * m_qRef.data[0] = manip->joint(0)->q;
             * m_qRef.data[1] = manip->joint(1)->q;
             * m_qRef.data[2] = manip->joint(2)->q;
             * m_qRef.data[3] = manip->joint(3)->q;
             * m_qRef.data[4] = manip->joint(4)->q;
             * m_qRef.data[5] = manip->joint(5)->q;
             */
            m_qRef.data[0] = target_q[0];
            m_qRef.data[1] = target_q[1];
            m_qRef.data[2] = target_q[2];
            m_qRef.data[3] = target_q[3];
            m_qRef.data[4] = target_q[4];
            m_qRef.data[5] = target_q[5];
            m_qRef.data[6] = lleg_av[0];
            m_qRef.data[7] = lleg_av[1];
            m_qRef.data[8] = lleg_av[2];
            m_qRef.data[9] = lleg_av[3];
            m_qRef.data[10] = lleg_av[4];
            m_qRef.data[11] = lleg_av[5];

            m_optionalData.data[0] = 1.0;
            m_optionalData.data[2] = 1.0;
            if(time < 1.0) {
                m_optionalData.data[1] = 0.0;
                m_optionalData.data[3] = 1.0 - time;
            } else {
                m_optionalData.data[1] = 1.0;
                m_optionalData.data[3] = 1.0;
            }

            m_zmpRef.data.x = zmp[0];
            m_zmpRef.data.y = zmp[1];
            m_zmpRef.data.z = zmp[2];
            m_accRef.data.ax = acc[0];
            m_accRef.data.ay = acc[1];
            m_accRef.data.az = acc[2];
            m_basePos.data.x = pos[0];
            m_basePos.data.y = pos[1];
            m_basePos.data.z = pos[2];
            m_baseRpy.data.r = rpy[0];
            m_baseRpy.data.p = rpy[1];
            m_baseRpy.data.y = rpy[2];
            size_t force_i = 0;
            for (size_t i = 0; i < m_wrenches.size(); i++) {
                m_wrenches[i].data[0] = wrenches[force_i++];
                m_wrenches[i].data[1] = wrenches[force_i++];
                m_wrenches[i].data[2] = wrenches[force_i++];
                m_wrenches[i].data[3] = wrenches[force_i++];
                m_wrenches[i].data[4] = wrenches[force_i++];
                m_wrenches[i].data[5] = wrenches[force_i++];
            }
            m_qRef.tm = m_qInit.tm;
            m_qRefOut.write();
            m_tqRefOut.write();
            m_zmpRefOut.write();
            m_accRefOut.write();
            m_basePosOut.write();
            m_baseRpyOut.write();
            m_optionalDataOut.write();
            for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
                m_wrenchesOut[i]->write();
            }

            if (m_clearFlag){
                m_seq->clear(0.001);
            }
        }
        //not off
        if(log_file_idx < 7501) {//(emergency_step_loop < CROSS_TIME + 2001) && (log_file_idx < 7501) ) {
            log_data[log_file_idx].time = (m_qInit.tm.sec - time_offset)*1000000 + m_qInit.tm.nsec/1000;
            //log_data[log_file_idx].time[1] = m_qInit.tm.nsec;
            log_data[log_file_idx].angle_vector[0] = m_qRef.data[0];
            log_data[log_file_idx].angle_vector[1] = m_qRef.data[1];
            log_data[log_file_idx].angle_vector[2] = m_qRef.data[2];
            log_data[log_file_idx].angle_vector[3] = m_qRef.data[3];
            log_data[log_file_idx].angle_vector[4] = m_qRef.data[4];
            log_data[log_file_idx].angle_vector[5] = m_qRef.data[5];
            log_data[log_file_idx].angle_vector[6] = m_qRef.data[6];
            log_data[log_file_idx].angle_vector[7] = m_qRef.data[7];
            log_data[log_file_idx].angle_vector[8] = m_qRef.data[8];
            log_data[log_file_idx].angle_vector[9] = m_qRef.data[9];
            log_data[log_file_idx].angle_vector[10] = m_qRef.data[10];
            log_data[log_file_idx].angle_vector[11] = m_qRef.data[11];
            log_data[log_file_idx].zmp[0] = m_zmpRef.data.x;
            log_data[log_file_idx].zmp[1] = m_zmpRef.data.y;
            log_data[log_file_idx].zmp[2] = m_zmpRef.data.z;
            log_data[log_file_idx].emergency_time = time;
            log_file_idx++;
        } else if(log_file_idx == 7501) {// emergency_step_loop == (CROSS_TIME + 2001) || log_file_idx == 7501){
            for(int i = 0; i < log_file_idx; i++) {
                fprintf(fp_log, "%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", log_data[i].time, log_data[i].angle_vector[0], log_data[i].angle_vector[1], log_data[i].angle_vector[2], log_data[i].angle_vector[3], log_data[i].angle_vector[4], log_data[i].angle_vector[5], log_data[i].angle_vector[6], log_data[i].angle_vector[7], log_data[i].angle_vector[8], log_data[i].angle_vector[9], log_data[i].angle_vector[10], log_data[i].angle_vector[11], log_data[i].zmp[0], log_data[i].zmp[1], log_data[i].zmp[2], log_data[i].emergency_time);
            }
            fclose(fp_log);
            log_file_idx = 10000;
            std::cerr << "\x1b[31m" << "[Seq]  save log is completed" << "\x1b[0m" << std::endl;
        }
    } else if(testes) {
        Guard guard(m_mutex);

        double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
        m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());
        m_zmpRef.data.x = zmp[0];
        m_zmpRef.data.y = zmp[1];
        m_zmpRef.data.z = zmp[2];
        m_accRef.data.ax = acc[0];
        m_accRef.data.ay = acc[1];
        m_accRef.data.az = acc[2];
        m_basePos.data.x = pos[0];
        m_basePos.data.y = pos[1];
        m_basePos.data.z = pos[2];
        m_baseRpy.data.r = rpy[0];
        m_baseRpy.data.p = rpy[1];
        m_baseRpy.data.y = rpy[2];

        /*
         * for(int i = 0; i < m_qRef.data.length() ; i++) {
         *     m_qRef.data[i] = 0;
         * }
         * m_qRef.data[1] = loop/10000.0;
         * m_qRef.data[2] = loop/1000.0;
         * m_qRef.data[3] = loop/100.0;
         * m_qRef.data[4] = 1.0;
         * m_qRefOut.write();
         * if( (loop%250) == 0 ) {
         *     std::cerr << "[Seq] testtest" << std::endl;
         * }
         */
        size_t force_i = 0;
        for (size_t i = 0; i < m_wrenches.size(); i++) {
            m_wrenches[i].data[0] = wrenches[force_i++];
            m_wrenches[i].data[1] = wrenches[force_i++];
            m_wrenches[i].data[2] = wrenches[force_i++];
            m_wrenches[i].data[3] = wrenches[force_i++];
            m_wrenches[i].data[4] = wrenches[force_i++];
            m_wrenches[i].data[5] = wrenches[force_i++];
        }
        m_qRef.tm = m_qInit.tm;
        m_qRefOut.write();
        m_tqRefOut.write();
        m_zmpRefOut.write();
        m_accRefOut.write();
        m_basePosOut.write();
        m_baseRpyOut.write();
        m_optionalDataOut.write();
        for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
            m_wrenchesOut[i]->write();
        }

        if (m_clearFlag){
            m_seq->clear(0.001);
        }

    } else if(knee_speed_test){
        Guard guard(m_mutex);

        double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
        m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());
        m_zmpRef.data.x = zmp[0];
        m_zmpRef.data.y = zmp[1];
        m_zmpRef.data.z = zmp[2];
        m_accRef.data.ax = acc[0];
        m_accRef.data.ay = acc[1];
        m_accRef.data.az = acc[2];
        m_basePos.data.x = pos[0];
        m_basePos.data.y = pos[1];
        m_basePos.data.z = pos[2];
        m_baseRpy.data.r = rpy[0];
        m_baseRpy.data.p = rpy[1];
        m_baseRpy.data.y = rpy[2];
        size_t force_i = 0;
        for (size_t i = 0; i < m_wrenches.size(); i++) {
            m_wrenches[i].data[0] = wrenches[force_i++];
            m_wrenches[i].data[1] = wrenches[force_i++];
            m_wrenches[i].data[2] = wrenches[force_i++];
            m_wrenches[i].data[3] = wrenches[force_i++];
            m_wrenches[i].data[4] = wrenches[force_i++];
            m_wrenches[i].data[5] = wrenches[force_i++];
        }

        if(knee_speed_test_loop == 0) {
            start_position = m_qRef.data[2];
            start_offset = -50.0;
            set_offset_count = 2000.0;
            std::cerr << "knee speed test start!!  target_speed :" << target_speed << std::endl;
            std::cerr << "accele:" << (set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed)) << "  plateau:" << (set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed) + (500.0*theta_cons)/target_speed) << "  decele:" << (set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed) << "  return:" << (set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed + 1000) << std::endl;
            time_offset = m_qInit.tm.sec;

            //std::string log_file_name("/home/leus/knee_speed_test_log.csv");
            char file_name[64];
            sprintf(file_name, "/home/leus/knee_speed_test/speed%ld/knee_speed_test_log%ld.csv", target_speed_deg, target_speed_deg);
            //std::string log_file_name = "/home/leus/knee_speed_test_log" +  buf + ".csv";
            if( (knee_speed_test_log = std::fopen(file_name, "w")) == NULL) {
                std::cerr << "\x1b[31m" << "[seq]  FILE OPEN ERROR!! ---> " << file_name << "\x1b[0m" << std::endl;
            } else {
                std::cerr << "\x1b[31m" << "[seq]  LOG FILE OPEN SUCCEEDED ---> " << file_name << "\x1b[0m" << std::endl;
            }
            fprintf(knee_speed_test_log, "time_usec,next_target_angle,current_angle,next_target_speed,current_speed,loop_idx\n");

            std::cerr << "file open end" << std::endl;
        }

        if(target_speed > 0) {
            if(knee_speed_test_loop < set_offset_count) {
                double time = (double)knee_speed_test_loop/set_offset_count;
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);

                target_q[3] = start_position + transition_coefficient*(start_offset*3.141592/180.0 - start_position);

            } else if(knee_speed_test_loop < (set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed))){
                if(knee_speed_test_loop == set_offset_count) {
                    std::cerr << "\x1b[31m" << "acceleration start" << "until :" << (set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed)) << "\x1b[0m" <<   std::endl;
                }
                double time = ((double)knee_speed_test_loop - set_offset_count)*(8.0*target_speed)/(500.0*15.0*theta_ad);
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);

                target_q[3] = start_offset*3.141592/180.0 + transition_coefficient*theta_ad;

            } else if(knee_speed_test_loop < (set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed) + (500.0*theta_cons)/target_speed)) {
                if(knee_speed_test_loop == (int)((set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed)) + 1)) {
                    std::cerr << "\x1b[31m" << "plateau start" <<  "until :" << (set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed) + (500.0*theta_cons)/target_speed) << "\x1b[0m" <<std::endl;
                }
                double time = ((double)knee_speed_test_loop - (set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed)))/500;

                target_q[3] = theta_ad/2.0 + start_offset*3.141592/180.0 + time*target_speed;

            } else if(knee_speed_test_loop < (set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed)) {
                if(knee_speed_test_loop == (int)(((set_offset_count + (500.0*15.0*theta_ad)/(2.0*8.0*target_speed) + (500.0*theta_cons)/target_speed)) + 1)) {
                    std::cerr << "deceleration start" <<  std::endl;
                }
                double time = ((double)knee_speed_test_loop - (set_offset_count + (500.0*theta_cons)/target_speed))*(8.0*target_speed)/(500.0*15.0*theta_ad);
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);

                target_q[3] = start_offset*3.141592/180.0 + theta_cons + transition_coefficient*theta_ad;

            } else if(knee_speed_test_loop < (set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed + 2001)) {
                if(knee_speed_test_loop == (int)(((set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed)) + 1)) {
                    std::cerr << "return to start position start" <<  std::endl;
                }
                double time = ((double)knee_speed_test_loop - (set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed))/2000.0;
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);

                target_q[3] = start_offset*3.141592/180.0 + theta_ad + theta_cons + transition_coefficient*(start_position - (start_offset*3.141592/180.0 + theta_ad + theta_cons));
            } else {
                knee_speed_test = false;
                knee_speed_test_loop = -1;
            }

            //log
            if( (-1 < knee_speed_test_loop) && (knee_speed_test_loop < 7501) ) {
                log_data[knee_speed_test_loop].time = (m_qInit.tm.sec - time_offset)*1000000 + m_qInit.tm.nsec/1000;
                log_data[knee_speed_test_loop].angle_vector[0] = target_q[3];
                log_data[knee_speed_test_loop].angle_vector[1] = m_qCurrent.data[2];
            } else if(knee_speed_test_loop == -1)  {
                long max_idx;
                if( (set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed + 2001) > 7500) {
                    max_idx = 7500;
                } else {
                    max_idx = (int)(set_offset_count + (500.0*15.0*theta_ad)/(8.0*target_speed) + (500.0*theta_cons)/target_speed + 2001);
                }
                for(long idx = 0; idx < max_idx; idx++) {
                    if(idx > 0) {
                        fprintf(knee_speed_test_log, "%ld,%f,%f,%f,%f,%ld\n", log_data[idx].time, log_data[idx].angle_vector[0], log_data[idx].angle_vector[1], (log_data[idx].angle_vector[0] - log_data[idx-1].angle_vector[0])*500, ((log_data[idx].angle_vector[1] - log_data[idx-1].angle_vector[1])/(log_data[idx].time - log_data[idx-1].time))*1000000, idx);
                    } else {
                        fprintf(knee_speed_test_log, "%ld,%f,%f,%f,%f,%ld\n", log_data[idx].time, log_data[idx].angle_vector[0], log_data[idx].angle_vector[1], 0.0, 0.0, idx);
                    }
                }
                fclose(knee_speed_test_log);
                log_file_idx = 10000;
                std::cerr << "\x1b[31m" << "[Seq]  save log is completed" << "\x1b[0m" << std::endl;
            }
        } else {
            std::cerr << "\x1b[33m" << "[SequencePlayer] Error!! target speed is less than ZERO!!" << "\x1b[0m" << std::endl;
            knee_speed_test = false;
            knee_speed_test_loop = -1;
            fclose(knee_speed_test_log);
        }

        m_qRef.data[2] = target_q[3];

        std::cerr <<target_q[3] << knee_speed_test_loop << "," << knee_speed_test_loop  <<  std::endl;

        m_qRef.tm = m_qInit.tm;
        m_qRefOut.write();
        m_tqRefOut.write();
        m_zmpRefOut.write();
        m_accRefOut.write();
        m_basePosOut.write();
        m_baseRpyOut.write();
        m_optionalDataOut.write();

        for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
            m_wrenchesOut[i]->write();
        }

        if (m_clearFlag){
            m_seq->clear(0.001);
        }

        knee_speed_test_loop++;

        //hogehogehoge
    } else if(jump_test) {
        //initial foot p(z) = 0.176715 [mm] at reset-pose
        //highest foot p(z) = 0.624015 [mm] (0.4473 -> 0.441)
        //lowest foot p(z) = 0.135515 [mm] (0.0412 -> 0.039)
        //each position limit is calculated using vertical foot movement from reset-pose
        Guard guard(m_mutex);

        double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
        double t;
        m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());
        m_zmpRef.data.x = zmp[0];
        m_zmpRef.data.y = zmp[1];
        m_zmpRef.data.z = zmp[2];
        m_accRef.data.ax = acc[0];
        m_accRef.data.ay = acc[1];
        m_accRef.data.az = acc[2];
        m_basePos.data.x = pos[0];
        m_basePos.data.y = pos[1];
        m_basePos.data.z = pos[2];
        m_baseRpy.data.r = rpy[0];
        m_baseRpy.data.p = rpy[1];
        m_baseRpy.data.y = rpy[2];
        size_t force_i = 0;
        for (size_t i = 0; i < m_wrenches.size(); i++) {
            m_wrenches[i].data[0] = wrenches[force_i++];
            m_wrenches[i].data[1] = wrenches[force_i++];
            m_wrenches[i].data[2] = wrenches[force_i++];
            m_wrenches[i].data[3] = wrenches[force_i++];
            m_wrenches[i].data[4] = wrenches[force_i++];
            m_wrenches[i].data[5] = wrenches[force_i++];
        }

        if(jump_test_loop == 0) {
            target_root_speed = target_speed_deg/1000.0;
            upward_displacement = 0.441;
            downward_displacement = 0.035;
            bending_count = 160;

            std::cerr << "jump test start!!  target_speed :" << target_root_speed << std::endl;
            std::cerr << "upward displacement: " << upward_displacement << "downward displacement: " << downward_displacement << std::endl;
            time_offset = m_qInit.tm.sec;

            start_p = m_robot->link(target_name)->p;
            start_R = m_robot->link(target_name)->R;
            target_p = start_p;
            target_R = start_R;

            std::cerr << "---- start ----" << std::endl;
            std::cerr << start_p << std::endl << std::endl;
            std::cerr << start_R << std::endl;

            //std::string log_file_name("/home/leus/knee_speed_test_log.csv");
            char file_name[64];
            sprintf(file_name, "/home/leus/jump_test/speed%ld/jump_log%ld.csv", target_speed_deg, target_speed_deg);
            //std::string log_file_name = "/home/leus/knee_speed_test_log" +  buf + ".csv";
            if( (jump_test_log = std::fopen(file_name, "w")) == NULL) {
                std::cerr << "\x1b[31m" << "[seq]  FILE OPEN ERROR!! ---> " << file_name << "\x1b[0m" << std::endl;
            } else {
                std::cerr << "\x1b[31m" << "[seq]  LOG FILE OPEN SUCCEEDED ---> " << file_name << "\x1b[0m" << std::endl;
            }
            fprintf(jump_test_log, "time_usec,target_p(z),next_target_angle(c-y),(c-r),(c-p),(k-p),(a-p),(a-r),current_angle(c-y),(c-r),(c-p),(k-p),(a-p),(a-r),next_target_speed(c-p),current_speed(c-p),loop_idx\n");

            std::cerr << "file open end" << std::endl;
        }

        if(target_root_speed > 0) {
            if(jump_test_loop < 1500.0) {
                double time = (double)jump_test_loop/1500.0;
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);
                target_p(2) = start_p(2) + transition_coefficient*upward_displacement;
                if( manip->calcInverseKinematics2(target_p, target_R) ) {
                    for(int i = 0; i < 6; i++) {
                        target_q[i] = manip->joint(i)->q;
                    }
                } else {
                    std::cerr << "\x1b[33m" << "[SequencePlayer] failed to solve IK" << "\x1b[0m" << endl;
                }
            } else if(jump_test_loop < (1500.0 + 500.0*(15.0/8.0)*(2.0/3.0)*(upward_displacement+downward_displacement)/target_root_speed) ) {
                if(jump_test_loop == 1500.0) {
                    std::cerr << "acceleration start:" << jump_test_loop << std::endl;
                }
                double time = (double)( (jump_test_loop - 1500.0) / (500.0*(15.0/8.0)*2.0*(2.0/3.0)*(upward_displacement+downward_displacement)/target_root_speed) );
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);
                target_p(2) = start_p(2) + upward_displacement - transition_coefficient*(2.0/3.0)*(upward_displacement+downward_displacement)*2;
                if( manip->calcInverseKinematics2(target_p, target_R) ) {
                    for(int i = 0; i < 6; i++) {
                        target_q[i] = manip->joint(i)->q;
                    }
                } else {
                    std::cerr << "\x1b[33m" << "[SequencePlayer] failed to solve IK" << "\x1b[0m" << endl;
                }
            } else if(jump_test_loop < (1500.0 + 500.0*(15.0/8.0)*(upward_displacement+downward_displacement)/target_root_speed) ) {
                if(jump_test_loop == (int)(1500.0 + 500.0*(15.0/8.0)*(2.0/3.0)*(upward_displacement+downward_displacement)/target_root_speed + 1.0) ) {
                    std::cerr << "deceleration start:" << jump_test_loop << std::endl;
                }
                double time = 0.5 + (double)( (jump_test_loop - (1500.0 + 500.0*(15.0/8.0)*(2.0/3.0)*(upward_displacement+downward_displacement)/target_root_speed)) / (500.0*(15.0/8.0)*2.0*(1.0/3.0)*(upward_displacement+downward_displacement)/target_root_speed) );
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);
                target_p(2) = start_p(2) + upward_displacement -  (1.0/3.0)*(upward_displacement+downward_displacement) - transition_coefficient*(2.0/3.0)*(upward_displacement+downward_displacement);
                if( manip->calcInverseKinematics2(target_p, target_R) ) {
                    for(int i = 0; i < 6; i++) {
                        target_q[i] = manip->joint(i)->q;
                    }
                } else {
                    std::cerr << "\x1b[33m" << "[SequencePlayer] failed to solve IK" << "\x1b[0m" << endl;
                }
            } else if(jump_test_loop < (1500.0 + 500.0*(15.0/8.0)*(upward_displacement+downward_displacement)/target_root_speed + bending_count + 1.0) ) {
                if(jump_test_loop == (int)(1500.0 + 500.0*(15.0/8.0)*(upward_displacement+downward_displacement)/target_root_speed + 1.0) ) {
                    std::cerr << "bending start:" << jump_test_loop << std::endl;
                }
                double time = (double)( (jump_test_loop - (1500.0 + 500.0*(15.0/8.0)*(upward_displacement+downward_displacement)/target_root_speed)) / (double)(bending_count) ) ;
                double transition_coefficient = 6*pow(time,5) - 15*pow(time,4) + 10*pow(time,3);
                target_p(2) = start_p(2) - downward_displacement + transition_coefficient*(upward_displacement + downward_displacement);
                if( manip->calcInverseKinematics2(target_p, target_R) ) {
                    for(int i = 0; i < 6; i++) {
                        target_q[i] = manip->joint(i)->q;
                    }
                } else {
                    std::cerr << "\x1b[33m" << "[SequencePlayer] failed to solve IK" << "\x1b[0m" << endl;
                }
            } else {
                jump_test = false;
                jump_test_loop = -1;
                std::cerr << "jump test finished" << std::endl;
            }

            //log
            if( (-1 < jump_test_loop) && (jump_test_loop < 7501) ) {
                log_data[jump_test_loop].time = (m_qInit.tm.sec - time_offset)*1000000 + m_qInit.tm.nsec/1000;
                log_data[jump_test_loop].rest = target_p(2);
                log_data[jump_test_loop].angle_vector[0] = target_q[0];
                log_data[jump_test_loop].angle_vector[1] = target_q[1];
                log_data[jump_test_loop].angle_vector[2] = target_q[2];
                log_data[jump_test_loop].angle_vector[3] = target_q[3];
                log_data[jump_test_loop].angle_vector[4] = target_q[4];
                log_data[jump_test_loop].angle_vector[5] = target_q[5];
                log_data[jump_test_loop].angle_vector[6] = m_qCurrent.data[0];
                log_data[jump_test_loop].angle_vector[7] = m_qCurrent.data[1];
                log_data[jump_test_loop].angle_vector[8] = m_qCurrent.data[2];
                log_data[jump_test_loop].angle_vector[9] = m_qCurrent.data[3];
                log_data[jump_test_loop].angle_vector[10] = m_qCurrent.data[4];
                log_data[jump_test_loop].angle_vector[11] = m_qCurrent.data[5];
            } else if(jump_test_loop == -1)  {
                long max_idx;
                if((1500.0 + 500.0*(15.0/8.0)*(upward_displacement+downward_displacement)/target_root_speed + bending_count + 1)  > 7500) {
                    max_idx = 7500;
                } else {
                    max_idx = (int)(1500.0 + 500.0*(15.0/8.0)*(upward_displacement+downward_displacement)/target_root_speed + bending_count + 1);
                }
                std::cerr << "max_idx: " << max_idx << std::endl;
                for(long idx = 0; idx < max_idx; idx++) {
                    if(idx > 0) {
                        fprintf(jump_test_log, "%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%ld\n", log_data[idx].time, log_data[idx].rest, log_data[idx].angle_vector[0], log_data[idx].angle_vector[1], log_data[idx].angle_vector[2],log_data[idx].angle_vector[3],log_data[idx].angle_vector[4],log_data[idx].angle_vector[5],log_data[idx].angle_vector[6],log_data[idx].angle_vector[7],log_data[idx].angle_vector[8],log_data[idx].angle_vector[9],log_data[idx].angle_vector[10],log_data[idx].angle_vector[11],(log_data[idx].angle_vector[3] - log_data[idx-1].angle_vector[3])*500, ((log_data[idx].angle_vector[9] - log_data[idx-1].angle_vector[9])/(log_data[idx].time - log_data[idx-1].time))*1000000, idx);
                    } else {
                        fprintf(jump_test_log, "%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%ld\n", log_data[idx].time, log_data[idx].rest, log_data[idx].angle_vector[0], log_data[idx].angle_vector[1], log_data[idx].angle_vector[2],log_data[idx].angle_vector[3],log_data[idx].angle_vector[4],log_data[idx].angle_vector[5],log_data[idx].angle_vector[6],log_data[idx].angle_vector[7],log_data[idx].angle_vector[8],log_data[idx].angle_vector[9],log_data[idx].angle_vector[10],log_data[idx].angle_vector[11], 0.0, 0.0, idx);
                    }
                }
                fclose(jump_test_log);
                log_file_idx = 10000;
                std::cerr << "\x1b[31m" << "[Seq]  save log is completed" << "\x1b[0m" << std::endl;
            }
        } else {
            std::cerr << "\x1b[33m" << "[SequencePlayer] Error!! target speed is less than ZERO!!" << "\x1b[0m" << std::endl;
            jump_test = false;
            jump_test_loop = -1;
            fclose(jump_test_log);
        }

        for(int i = 0; i < 6; i++) {
            m_qRef.data[i] = target_q[i];
            m_qRef.data[6+i] = target_q[i];
        }

        m_qRef.tm = m_qInit.tm;
        m_qRefOut.write();
        m_tqRefOut.write();
        m_zmpRefOut.write();
        m_accRefOut.write();
        m_basePosOut.write();
        m_baseRpyOut.write();
        m_optionalDataOut.write();

        for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
            m_wrenchesOut[i]->write();
        }

        if (m_clearFlag){
            m_seq->clear(0.001);
        }

        jump_test_loop++;
    }else {
#endif
        if (m_seq->isEmpty()){
            m_clearFlag = false;
            if (m_waitFlag){
                m_waitFlag = false;
                m_waitSem.post();
            }
            //std::cerr << "m_qRef.data  |" << m_qRef.data[0] << " , " << m_qRef.data[1] << " , " << m_qRef.data[2] << " , " << m_qRef.data[3] << " , " << m_qRef.data[4] << " , " << m_qRef.data[5] << std::endl;
            // std::cerr << "\x1b[0m" << "m_qRef.data  |" << m_qRef.data[0] << " , " << m_qRef.data[1] << " , " << m_qRef.data[2] << " , " << m_qRef.data[3] << " , " << m_qRef.data[4] << " , " << m_qRef.data[5] << "not emergency" << "\x1b[0m"  << std::endl;
        }else{
            Guard guard(m_mutex);

            double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
            m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());
            m_zmpRef.data.x = zmp[0];
            m_zmpRef.data.y = zmp[1];
            m_zmpRef.data.z = zmp[2];
            m_accRef.data.ax = acc[0];
            m_accRef.data.ay = acc[1];
            m_accRef.data.az = acc[2];
            m_basePos.data.x = pos[0];
            m_basePos.data.y = pos[1];
            m_basePos.data.z = pos[2];
            m_baseRpy.data.r = rpy[0];
            m_baseRpy.data.p = rpy[1];
            m_baseRpy.data.y = rpy[2];
            size_t force_i = 0;
            for (size_t i = 0; i < m_wrenches.size(); i++) {
                m_wrenches[i].data[0] = wrenches[force_i++];
                m_wrenches[i].data[1] = wrenches[force_i++];
                m_wrenches[i].data[2] = wrenches[force_i++];
                m_wrenches[i].data[3] = wrenches[force_i++];
                m_wrenches[i].data[4] = wrenches[force_i++];
                m_wrenches[i].data[5] = wrenches[force_i++];
            }
            m_qRef.tm = m_qInit.tm;
            m_qRefOut.write();
            m_tqRefOut.write();
            m_zmpRefOut.write();
            m_accRefOut.write();
            m_basePosOut.write();
            m_baseRpyOut.write();
            m_optionalDataOut.write();

            for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
                m_wrenchesOut[i]->write();
            }

            if (m_clearFlag){
                m_seq->clear(0.001);
            }
            //std::cerr << "\x1b[32m" << "m_qRef.data  |" << m_qRef.data[0] << " , " << m_qRef.data[1] << " , " << m_qRef.data[2] << " , " << m_qRef.data[3] << " , " << m_qRef.data[4] << " , " << m_qRef.data[5] << "\x1b[0m"  <<std::endl;
            //std::cerr << "\x1b[32m" << "              " << m_qRef.data[6] << " , " << m_qRef.data[7] << " , " << m_qRef.data[8] << " , " << m_qRef.data[9] << " , " << m_qRef.data[10] << " , " << m_qRef.data[11] << " --not emergency" << "\x1b[0m"  << std::endl;
        }
#ifdef CALC_VEL_N_ANGVEL
    }//the end of else (corresponding to if (m_emergency_step_flag)
#endif
    return RTC::RTC_OK;
}


double SequencePlayer::calc_theta_hard_coded(double time) {
    const double lf = 0.08;//width of foot
    const double L = sqrt( 0.1*0.1 + 0.823285*0.823285 );//length of foot (inverse pendulum model)
    const double g = 9.80665;//gravity acceleration
    const double root_g_L = sqrt(g/L);//root (g/L)
    const double Plus = exp(root_g_L);//exp(g/L)
    const double Minus = exp(-root_g_L);//exp(-g/L)
    const double C1 =  ( asin(0.310/L) + Minus*atan(0.1/0.823285) - (lf/L)*(1-Minus)) / (Plus - Minus);//general solution coefficient1
    const double C2 = -( asin(0.310/L) + Plus *atan(0.1/0.823285) - (lf/L)*(1-Plus )) / (Plus - Minus);//general solution coefficient2

    return ( C1*exp(root_g_L*time) + C2*exp(-root_g_L*time) + lf/L );
}

double SequencePlayer::calc_theta(double time, double angle) {
    const double lf = 0.08;
    const double foot_half_interval = 0.1*sin(angle);
    const double L = sqrt( pow(foot_half_interval, 2) + pow(0.823285, 2) );
    const double g = 9.80665;
    const double root_g_L = sqrt(g/L);
    const double Plus = exp(root_g_L);
    const double Minus = exp(-root_g_L);
    const double C1 =  ( asin(0.310/L) + Minus*atan(foot_half_interval/0.823285) - (lf/L)*(1-Minus)) / (Plus - Minus);//general solution coefficient1
    const double C2 = -( asin(0.310/L) + Plus *atan(foot_half_interval/0.823285) - (lf/L)*(1-Plus )) / (Plus - Minus);//general solution coefficient2

    return ( C1*exp(root_g_L*time) + C2*exp(-root_g_L*time) + lf/L );
}


/*
  RTC::ReturnCode_t SequencePlayer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

void SequencePlayer::setClearFlag()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_clearFlag = true;
}

void SequencePlayer::waitInterpolation()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_waitFlag = true;
    m_waitSem.wait();
}

bool SequencePlayer::waitInterpolationOfGroup(const char *gname)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_gname = gname;
    m_waitFlag = true;
    m_waitSem.wait();
    return true;
}


bool SequencePlayer::setJointAngle(short id, double angle, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    dvector q(m_robot->numJoints());
    m_seq->getJointAngles(q.data());
    q[id] = angle;
    for (int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = q[i];
    }
    m_robot->calcForwardKinematics();
    hrp::Vector3 absZmp = m_robot->calcCM();
    absZmp[2] = 0;
    hrp::Link *root = m_robot->rootLink();
    hrp::Vector3 relZmp = root->R.transpose()*(absZmp - root->p);
    m_seq->setJointAngles(q.data(), tm);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    for (int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = angles[i];
    }
    m_robot->calcForwardKinematics();
    hrp::Vector3 absZmp = m_robot->calcCM();
    absZmp[2] = 0;
    hrp::Link *root = m_robot->rootLink();
    hrp::Vector3 relZmp = root->R.transpose()*(absZmp - root->p);
    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    v_poss.push_back(angles);
    v_tms.push_back(tm);
    m_seq->setJointAnglesSequence(v_poss, v_tms);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, const bool *mask, 
                                    double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    double pose[m_robot->numJoints()];
    for (int i=0; i<m_robot->numJoints(); i++){
        pose[i] = mask[i] ? angles[i] : m_qInit.data[i];
    }
    m_seq->setJointAngles(pose, tm);
    return true;
}

bool SequencePlayer::setJointAnglesSequence(const OpenHRP::dSequenceSequence angless, const OpenHRP::bSequence& mask, const OpenHRP::dSequence& times)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    bool tmp_mask[robot()->numJoints()];
    if (mask.length() != robot()->numJoints()) {
        for (int i=0; i < robot()->numJoints(); i++) tmp_mask[i] = true;
    }else{
        for (int i=0; i < robot()->numJoints(); i++) tmp_mask[i] = mask.get_buffer()[i];
    }
    int len = angless.length();
    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    for ( int i = 0; i < angless.length(); i++ ) v_poss.push_back(angless[i].get_buffer());
    for ( int i = 0; i <  times.length();  i++ )  v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequence(v_poss, v_tms);
}

bool SequencePlayer::clearJointAngles()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    return m_seq->clearJointAngles();
}

bool SequencePlayer::setJointAnglesSequenceOfGroup(const char *gname, const OpenHRP::dSequenceSequence angless, const OpenHRP::dSequence& times)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    for ( int i = 0; i < angless.length(); i++ ) v_poss.push_back(angless[i].get_buffer());
    for ( int i = 0; i <  times.length();  i++ )  v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequenceOfGroup(gname, v_poss, v_tms, angless.length()>0?angless[0].length():0);
}

bool SequencePlayer::clearJointAnglesOfGroup(const char *gname)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    return m_seq->clearJointAnglesOfGroup(gname);
}

bool SequencePlayer::setJointAnglesSequenceFull(const OpenHRP::dSequenceSequence i_jvss, const OpenHRP::dSequenceSequence i_vels, const OpenHRP::dSequenceSequence i_torques, const OpenHRP::dSequenceSequence i_poss, const OpenHRP::dSequenceSequence i_rpys, const OpenHRP::dSequenceSequence i_accs, const OpenHRP::dSequenceSequence i_zmps, const OpenHRP::dSequenceSequence i_wrenches, const OpenHRP::dSequenceSequence i_optionals, const dSequence i_tms)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    int len = i_jvss.length();
    std::vector<const double*> v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals;
    std::vector<double> v_tms;
    for ( int i = 0; i < i_jvss.length(); i++ ) v_poss.push_back(i_jvss[i].get_buffer());
    for ( int i = 0; i < i_vels.length(); i++ ) v_poss.push_back(i_vels[i].get_buffer());
    for ( int i = 0; i < i_torques.length(); i++ ) v_poss.push_back(i_torques[i].get_buffer());
    for ( int i = 0; i < i_poss.length(); i++ ) v_poss.push_back(i_poss[i].get_buffer());
    for ( int i = 0; i < i_rpys.length(); i++ ) v_poss.push_back(i_rpys[i].get_buffer());
    for ( int i = 0; i < i_accs.length(); i++ ) v_poss.push_back(i_accs[i].get_buffer());
    for ( int i = 0; i < i_zmps.length(); i++ ) v_poss.push_back(i_zmps[i].get_buffer());
    for ( int i = 0; i < i_wrenches.length(); i++ ) v_poss.push_back(i_wrenches[i].get_buffer());
    for ( int i = 0; i < i_optionals.length(); i++ ) v_poss.push_back(i_optionals[i].get_buffer());
    for ( int i = 0; i < i_tms.length();  i++ )  v_tms.push_back(i_tms[i]);
    return m_seq->setJointAnglesSequenceFull(v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals, v_tms);
}

bool SequencePlayer::setBasePos(const double *pos, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setBasePos(pos, tm);
    return true;
}

bool SequencePlayer::setBaseRpy(const double *rpy, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setBaseRpy(rpy, tm);
    return true;
}

bool SequencePlayer::setZmp(const double *zmp, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setZmp(zmp, tm);
    return true;
}

bool SequencePlayer::setWrenches(const double *wrenches, double tm)
{
    Guard guard(m_mutex);
    m_seq->setWrenches(wrenches, tm);
    return true;
}

bool SequencePlayer::setTargetPose(const char* gname, const double *xyz, const double *rpy, double tm, const char* frame_name)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    // setup
    std::vector<int> indices;
    hrp::dvector start_av, end_av;
    std::vector<hrp::dvector> avs;
    if (! m_seq->getJointGroup(gname, indices) ) {
        std::cerr << "[setTargetPose] Could not find joint group " << gname << std::endl;
        return false;
    }
    start_av.resize(indices.size());
    end_av.resize(indices.size());

    //std::cerr << std::endl;
    if ( ! m_robot->joint(indices[0])->parent ) {
        std::cerr << "[setTargetPose] " << m_robot->joint(indices[0])->name << " does not have parent" << std::endl;
        return false;
    }
    string base_parent_name = m_robot->joint(indices[0])->parent->name;
    string target_name = m_robot->joint(indices[indices.size()-1])->name;
    // prepare joint path
    hrp::JointPathExPtr manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(base_parent_name), m_robot->link(target_name), dt));

    // calc fk
    for (int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = m_qRef.data.get_buffer()[i];
    }
    m_robot->calcForwardKinematics();
    for ( int i = 0; i < manip->numJoints(); i++ ){
        start_av[i] = manip->joint(i)->q;
    }

    // xyz and rpy are relateive to root link, where as pos and rotatoin of manip->calcInverseKinematics are relative to base link

    // ik params
    hrp::Vector3 start_p(m_robot->link(target_name)->p);
    hrp::Matrix33 start_R(m_robot->link(target_name)->R);
    hrp::Vector3 end_p(xyz[0], xyz[1], xyz[2]);
    hrp::Matrix33 end_R = m_robot->link(target_name)->calcRfromAttitude(hrp::rotFromRpy(rpy[0], rpy[1], rpy[2]));

    // change start and end must be relative to the frame_name
    if ( (frame_name != NULL) && (! m_robot->link(frame_name) ) ) {
        std::cerr << "[setTargetPose] Could not find frame_name " << frame_name << std::endl;
        return false;
    } else if ( frame_name != NULL ) {
        hrp::Vector3 frame_p(m_robot->link(frame_name)->p);
        hrp::Matrix33 frame_R(m_robot->link(frame_name)->attitude());
        // fix start/end references from root to frame;
        end_p = frame_R * end_p + frame_p;
        end_R = frame_R * end_R;
    }
    manip->setMaxIKError(m_error_pos,m_error_rot);
    manip->setMaxIKIteration(m_iteration);
    std::cerr << "[setTargetPose] Solveing IK with frame" << frame_name << ", Error " << m_error_pos << m_error_rot << ", Iteration " << m_iteration << std::endl;
    std::cerr << "                Start " << start_p << start_R<< std::endl;
    std::cerr << "                End   " << end_p << end_R<< std::endl;

    // interpolate & calc ik
    int len = max(((start_p - end_p).norm() / 0.02 ), // 2cm
                  ((hrp::omegaFromRot(start_R.transpose() * end_R).norm()) / 0.025)); // 2 deg
    len = max(len, 1);

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    v_pos.resize(len);
    v_tm.resize(len);

    // do loop
    for (int i = 0; i < len; i++ ) {
        double a = (1+i)/(double)len;
        hrp::Vector3 p = (1-a)*start_p + a*end_p;
        hrp::Vector3 omega = hrp::omegaFromRot(start_R.transpose() * end_R);
        hrp::Matrix33 R = start_R * rodrigues(omega.isZero()?omega:omega.normalized(), a*omega.norm());
        bool ret = manip->calcInverseKinematics2(p, R);

        if ( m_debugLevel > 0 ) {
            // for debug
            std::cerr << "target pos/rot : " << i << "/" << a << " : "
                      << p[0] << " " << p[1] << " " << p[2] << ","
                      << omega[0] << " " << omega[1] << " " << omega[2] << std::endl;
        }
        if ( ! ret ) {
            std::cerr << "[setTargetPose] IK failed" << std::endl;
            return false;
        }
        v_pos[i] = (const double *)malloc(sizeof(double)*manip->numJoints());
        for ( int j = 0; j < manip->numJoints(); j++ ){
            ((double *)v_pos[i])[j] = manip->joint(j)->q;
        }
        v_tm[i] = tm/len;
    }

    if ( m_debugLevel > 0 ) {
        // for debug
        for(int i = 0; i < len; i++ ) {
            std::cerr << v_tm[i] << ":";
            for(int j = 0; j < start_av.size(); j++ ) {
                std::cerr << v_pos[i][j] << " ";
            }
            std::cerr << std::endl;
        }
    }

    bool ret = m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(), v_pos.size()>0?indices.size():0);

    // clean up memory, need to improve
    for (int i = 0; i < len; i++ ) {
        free((double *)v_pos[i]);
    }

    return ret;
}

void SequencePlayer::loadPattern(const char *basename, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (setInitialState()){
        m_seq->loadPattern(basename, tm);
    }
}

bool SequencePlayer::setInitialState(double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << "m_seq-isEmpty() " << m_seq->isEmpty() << ", m_Init.data.length() " << m_qInit.data.length() << std::endl;
    }
    if (!m_seq->isEmpty()) return true;

    if (m_qInit.data.length() == 0){
        std::cerr << "can't determine initial posture" << std::endl;
        return false;
    }else{
        m_seq->setJointAngles(m_qInit.data.get_buffer(), tm);
        for (int i=0; i<m_robot->numJoints(); i++){
            Link *l = m_robot->joint(i);
            l->q = m_qInit.data[i];
            m_qRef.data[i] = m_qInit.data[i]; // update m_qRef for setTargetPose()
        }

        Link *root = m_robot->rootLink();

        root->p << m_basePosInit.data.x,
            m_basePosInit.data.y,
            m_basePosInit.data.z;
        m_seq->setBasePos(root->p.data(), tm);

        double rpy[] = {m_baseRpyInit.data.r,
                        m_baseRpyInit.data.p,
                        m_baseRpyInit.data.y};
        m_seq->setBaseRpy(rpy, tm);
        calcRotFromRpy(root->R, rpy[0], rpy[1], rpy[2]);

        double zmp[] = {m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z};
        m_seq->setZmp(zmp, tm);
        double zero[] = {0,0,0};
        m_seq->setBaseAcc(zero, tm);
        return true;
    }
}

void SequencePlayer::playPattern(const dSequenceSequence& pos, const dSequenceSequence& rpy, const dSequenceSequence& zmp, const dSequence& tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return;

    std::vector<const double *> v_pos, v_rpy, v_zmp;
    std::vector<double> v_tm;
    for ( int i = 0; i < pos.length(); i++ ) v_pos.push_back(pos[i].get_buffer());
    for ( int i = 0; i < rpy.length(); i++ ) v_rpy.push_back(rpy[i].get_buffer());
    for ( int i = 0; i < zmp.length(); i++ ) v_zmp.push_back(zmp[i].get_buffer());
    for ( int i = 0; i < tm.length() ; i++ ) v_tm.push_back(tm[i]);
    return m_seq->playPattern(v_pos, v_rpy, v_zmp, v_tm, m_qInit.data.get_buffer(), pos.length()>0?pos[0].length():0);
}

bool SequencePlayer::setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    interpolator::interpolation_mode new_mode;
    if (i_mode_ == OpenHRP::SequencePlayerService::LINEAR){
        new_mode = interpolator::LINEAR;
    }else if (i_mode_ == OpenHRP::SequencePlayerService::HOFFARBIB){
        new_mode = interpolator::HOFFARBIB;
    }else{
        return false;
    }
    return m_seq->setInterpolationMode(new_mode);
}

bool SequencePlayer::addJointGroup(const char *gname, const OpenHRP::SequencePlayerService::StrSequence& jnames)
{
    std::cerr << "[addJointGroup] group name = " << gname << std::endl;
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    if (!waitInterpolationOfGroup(gname)) return false;

    Guard guard(m_mutex);
    std::vector<int> indices;
    for (size_t i=0; i<jnames.length(); i++){
        hrp::Link *l = m_robot->link(std::string(jnames[i]));
        if (l){
            indices.push_back(l->jointId);
        }else{
            std::cerr << "[addJointGroup] link name " << jnames[i] << "is not found" << std::endl;
            return false;
        }
    }
    return m_seq->addJointGroup(gname, indices);
}

bool SequencePlayer::removeJointGroup(const char *gname)
{
    std::cerr << "[removeJointGroup] group name = " << gname << std::endl;
    if (!waitInterpolationOfGroup(gname)) return false;
    bool ret;
    {
        Guard guard(m_mutex);
        ret = m_seq->removeJointGroup(gname);
    }
    return ret;
}

bool SequencePlayer::setJointAnglesOfGroup(const char *gname, const dSequence& jvs, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;
    return m_seq->setJointAnglesOfGroup(gname, jvs.get_buffer(), jvs.length(), tm);
}

bool SequencePlayer::playPatternOfGroup(const char *gname, const dSequenceSequence& pos, const dSequence& tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    for ( int i = 0; i < pos.length(); i++ ) v_pos.push_back(pos[i].get_buffer());
    for ( int i = 0; i < tm.length() ; i++ ) v_tm.push_back(tm[i]);
    return m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(), pos.length()>0?pos[0].length():0);
}

void SequencePlayer::setMaxIKError(double pos, double rot){
    m_error_pos = pos;
    m_error_rot = rot;
}

void SequencePlayer::setMaxIKIteration(short iter){
    m_iteration= iter;
}


extern "C"
{

    void SequencePlayerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(sequenceplayer_spec);
        manager->registerFactory(profile,
                                 RTC::Create<SequencePlayer>,
                                 RTC::Delete<SequencePlayer>);
    }

};


