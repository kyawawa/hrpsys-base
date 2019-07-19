// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef STABILIZER_SERVICE_H
#define STABILIZER_SERVICE_H

#include "hrpsys/idl/StabilizerService.hh"

class AutoBalanceStabilizer;

class StabilizerService_impl
    : public virtual POA_OpenHRP::StabilizerService,
      public virtual PortableServer::RefCountServantBase
{
  public:
    StabilizerService_impl();
    virtual ~StabilizerService_impl();

    void startStabilizer(void);
    void stopStabilizer(void);
    void getParameter(OpenHRP::StabilizerService::stParam_out i_param);
    void setParameter(const OpenHRP::StabilizerService::stParam& i_param);
    void setComponentPointer(AutoBalanceStabilizer *i_autobalancestabilizer);

    bool dummy();
  private:
    AutoBalanceStabilizer *m_autobalancestabilizer;
};

#endif
