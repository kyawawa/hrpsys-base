#include "AutoBalancerService_impl.h"
#include "AutoBalanceStabilizer.h"

AutoBalancerService_impl::AutoBalancerService_impl() : m_autobalancestabilizer(NULL)
{
}

AutoBalancerService_impl::~AutoBalancerService_impl()
{
}

CORBA::Boolean AutoBalancerService_impl::goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th)
{
    return m_autobalancestabilizer->goPos(x, y, th);
};

CORBA::Boolean AutoBalancerService_impl::goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth)
{
    return m_autobalancestabilizer->goVelocity(vx, vy, vth);
};

CORBA::Boolean AutoBalancerService_impl::goStop()
{
    return m_autobalancestabilizer->goStop();
};

CORBA::Boolean AutoBalancerService_impl::emergencyStop()
{
    return m_autobalancestabilizer->emergencyStop();
};

CORBA::Boolean AutoBalancerService_impl::setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx)
{
    return m_autobalancestabilizer->setFootSteps(fss, overwrite_fs_idx);
}

CORBA::Boolean AutoBalancerService_impl::setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx)
{
    return m_autobalancestabilizer->setFootStepsWithParam(fss, spss, overwrite_fs_idx);
}

void AutoBalancerService_impl::waitFootSteps()
{
    return m_autobalancestabilizer->waitFootSteps();
};

void AutoBalancerService_impl::waitFootStepsEarly(CORBA::Double tm)
{
    return m_autobalancestabilizer->waitFootStepsEarly(tm);
};

CORBA::Boolean AutoBalancerService_impl::startAutoBalancer(const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
    return m_autobalancestabilizer->startAutoBalancer(limbs);
};

CORBA::Boolean AutoBalancerService_impl::stopAutoBalancer()
{
    return m_autobalancestabilizer->stopAutoBalancer();
};

CORBA::Boolean AutoBalancerService_impl::setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
    return m_autobalancestabilizer->setGaitGeneratorParam(i_param);
};

CORBA::Boolean AutoBalancerService_impl::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam_out i_param)
{
    i_param = new OpenHRP::AutoBalancerService::GaitGeneratorParam();
    i_param->stride_parameter.length(6);
    i_param->toe_heel_phase_ratio.length(7);
    i_param->zmp_weight_map.length(4);
    return m_autobalancestabilizer->getGaitGeneratorParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
    return m_autobalancestabilizer->setAutoBalancerParam(i_param);
};

CORBA::Boolean AutoBalancerService_impl::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam_out i_param)
{
    i_param = new OpenHRP::AutoBalancerService::AutoBalancerParam();
    return m_autobalancestabilizer->getAutoBalancerParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam_out i_param)
{
    i_param = new OpenHRP::AutoBalancerService::FootstepParam();
    return m_autobalancestabilizer->getFootstepParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep)
{
    return m_autobalancestabilizer->adjustFootSteps(rfootstep, lfootstep);
};

CORBA::Boolean AutoBalancerService_impl::getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx)
{
    return m_autobalancestabilizer->getRemainingFootstepSequence(o_footstep, o_current_fs_idx);
};

CORBA::Boolean AutoBalancerService_impl::getGoPosFootstepsSequence(CORBA::Double x, CORBA::Double y, CORBA::Double th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep)
{
    return m_autobalancestabilizer->getGoPosFootstepsSequence(x, y, th, o_footstep);
};

CORBA::Boolean AutoBalancerService_impl::releaseEmergencyStop()
{
    return m_autobalancestabilizer->releaseEmergencyStop();
};

void AutoBalancerService_impl::setComponentPointer(AutoBalanceStabilizer *i_autobalancestabilizer)
{
    m_autobalancestabilizer = i_autobalancestabilizer;
}
