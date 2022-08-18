#ifndef CVIDEODEVICEMODEL_8076_H
#define CVIDEODEVICEMODEL_8076_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8076 : public CVideoDeviceModel
{
public:
    virtual bool InterleaveModeSupport() { return true; }
    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_8076(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_8076_H