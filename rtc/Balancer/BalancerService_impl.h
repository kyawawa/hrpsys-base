// -*- C++ -*-
#ifndef __BALANCER_SERVICE_H__
#define __BALANCER_SERVICE_H__

#include "hrpsys/idl/BalancerService.hh"

using namespace OpenHRP;

class Balancer;

class BalancerService_impl
    : public virtual POA_OpenHRP::BalancerService,
      public virtual PortableServer::RefCountServantBase
{
public:
    /**
       \brief constructor
    */
    BalancerService_impl();

    /**
       \brief destructor
    */
    virtual ~BalancerService_impl();

    CORBA::Boolean setBalancerParam(const OpenHRP::BalancerService::BalancerParam& i_param);
    CORBA::Boolean getBalancerParam(OpenHRP::BalancerService::BalancerParam_out i_param);
    CORBA::Boolean startBalancer();
    CORBA::Boolean stopBalancer(CORBA::Double migration_time);
    CORBA::Boolean startJump(CORBA::Double height, CORBA::Double squat);
    CORBA::Boolean startSquat();
    CORBA::Boolean stopSquat();

    void balancer(Balancer *i_balancer);
private:
    Balancer* m_balancer;
};

#endif // __BALANCER_SERVICE_H__
