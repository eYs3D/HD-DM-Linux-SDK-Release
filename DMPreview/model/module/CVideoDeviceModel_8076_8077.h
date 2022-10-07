#ifndef CVIDEODEVICEMODEL_8076_8077_H
#define CVIDEODEVICEMODEL_8076_8077_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_8076_8077 : public CVideoDeviceModel
{
public:
    virtual bool InterleaveModeSupport() { return true; }
    virtual int GetCurrentTemperature(float &temperature);
    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_8076_8077(DEVSELINFO *pDeviceSelfInfo);
    virtual int TransformDepthDataType(int nDepthDataType, bool bRectifyData);

};

#endif // CVIDEODEVICEMODEL_8076_8077_H