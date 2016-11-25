// -*-C++-*-

#ifndef ROBOTHARDWARESERVICE_IMPL_H
#define ROBOTHARDWARESERVICE_IMPL_H

#include <boost/intrusive_ptr.hpp>
#include "hrpsys/idl/RobotHardwareService.hh"

#include "robot.h"

class RobotHardwareService_impl
    : public virtual POA_OpenHRP::RobotHardwareService,
      public virtual PortableServer::RefCountServantBase
{
public:
    RobotHardwareService_impl();
    virtual ~RobotHardwareService_impl();

    void getStatus(OpenHRP::RobotHardwareService::RobotState_out rs);
    void getStatus2(OpenHRP::RobotHardwareService::RobotState2_out rs);

    CORBA::Boolean power(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss);
    CORBA::Boolean servo(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss);
    void setServoGainPercentage(const char *jname, double percentage);
    void setEachServoGainPercentage(const char *jname, const OpenHRP::RobotHardwareService::DblSequence& percentage);
    void setServoGainPercentagePD(const char *jname, double percentage, CORBA::Boolean p, CORBA::Boolean d);
    void setEachServoGainPercentagePD(const char *jname, const OpenHRP::RobotHardwareService::DblSequence& percentage, CORBA::Boolean p, CORBA::Boolean d);
    void getServoPgainPercentage(::OpenHRP::RobotHardwareService::DblSequence_out _pgain);
    void getServoDgainPercentage(::OpenHRP::RobotHardwareService::DblSequence_out _dgain);
    void setServoErrorLimit(const char *jname, double limit);
    void calibrateInertiaSensor();
    void removeForceSensorOffset();
    void initializeJointAngle(const char* name, const char* option);
    CORBA::Boolean addJointGroup(const char* gname, const OpenHRP::RobotHardwareService::StrSequence& jnames);
    CORBA::Boolean readDigitalInput(::OpenHRP::RobotHardwareService::OctSequence_out din);
    CORBA::Long lengthDigitalInput();
    CORBA::Boolean writeDigitalOutput(const ::OpenHRP::RobotHardwareService::OctSequence& dout);
    CORBA::Boolean writeDigitalOutputWithMask(const ::OpenHRP::RobotHardwareService::OctSequence& dout, const ::OpenHRP::RobotHardwareService::OctSequence& mask);
    CORBA::Long lengthDigitalOutput();
    CORBA::Boolean readDigitalOutput(::OpenHRP::RobotHardwareService::OctSequence_out dout);
    //
    void setRobot(boost::shared_ptr<robot>& i_robot) { m_robot = i_robot; }
private:
    boost::shared_ptr<robot> m_robot;
};
#endif
