#ifndef CVIDEODEVICEMODEL_Iris_H
#define CVIDEODEVICEMODEL_Iris_H
#include "CVideoDeviceModel.h"

class CVideoDeviceModel_Iris : public CVideoDeviceModel
{
public:
    virtual bool InterleaveModeSupport() { return true; }
    virtual int TransformDepthDataType(int nDepthDataType, bool bRectifyData);
    virtual int AdjustZDTableIndex(int &nIndex);
    friend class CVideoDeviceModelFactory;

protected:
    CVideoDeviceModel_Iris(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVIDEODEVICEMODEL_Iris_H