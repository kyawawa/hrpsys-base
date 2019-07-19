// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "StabilizerService_impl.h"
#include "AutoBalanceStabilizer.h"

StabilizerService_impl::StabilizerService_impl() : m_autobalancestabilizer(NULL)
{
}

StabilizerService_impl::~StabilizerService_impl()
{
}

void StabilizerService_impl::startStabilizer(void)
{
    m_autobalancestabilizer->startStabilizer();
}

void StabilizerService_impl::stopStabilizer(void)
{
    m_autobalancestabilizer->stopStabilizer();
}

void StabilizerService_impl::getParameter(OpenHRP::StabilizerService::stParam_out i_param)
{
    i_param = new OpenHRP::StabilizerService::stParam();
    return m_autobalancestabilizer->getStabilizerParam(*i_param);
};

void StabilizerService_impl::setParameter(const OpenHRP::StabilizerService::stParam& i_stp)
{
    m_autobalancestabilizer->setStabilizerParam(i_stp);
}

bool StabilizerService_impl::dummy()
{
    std::cout << "AutoBalanceStabilizerService: " << std::endl;
}

void StabilizerService_impl::setComponentPointer(AutoBalanceStabilizer *i_autobalancestabilizer)
{
    m_autobalancestabilizer = i_autobalancestabilizer;
}
