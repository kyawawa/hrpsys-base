// -*- C++ -*-
#include <iostream>
#include "BalancerService_impl.h"
#include "Balancer.h"

BalancerService_impl::BalancerService_impl()
{
}

BalancerService_impl::~BalancerService_impl()
{
}

CORBA::Boolean BalancerService_impl::setBalancerParam(const OpenHRP::BalancerService::BalancerParam& i_param)
{
    return m_balancer->setBalancerParam(i_param);
};

CORBA::Boolean BalancerService_impl::getBalancerParam(OpenHRP::BalancerService::BalancerParam_out i_param)
{
    return m_balancer->getBalancerParam(i_param);
};

CORBA::Boolean BalancerService_impl::startBalancer()
{
    return m_balancer->startBalancer();
};

CORBA::Boolean BalancerService_impl::stopBalancer()
{
    return m_balancer->stopBalancer();
};

void BalancerService_impl::balancer(Balancer *i_balancer)
{
    m_balancer = i_balancer;
};
