#ifndef CVIDEODEVICEMODEL_8081_H
#define CVIDEODEVICEMODEL_8081_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8081 : public CVideoDeviceModel
{
public:
    virtual void SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController);

    virtual int UpdateIR();
    virtual bool IRExtendSupport(){ return false; }
    virtual bool IsIRExtended(){ return false; }
    virtual int ExtendIR(bool bEnable){ return APC_NotSupport; }
    virtual int AdjustZDTableIndex(int &nIndex);
    friend class CVideoDeviceModelFactory;
protected:
    CVideoDeviceModel_8081(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8081_H
