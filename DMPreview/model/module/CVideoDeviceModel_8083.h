#ifndef CVideoDeviceModel_8083_H
#define CVideoDeviceModel_8083_H
#include<CVideoDeviceModel_8073.h>

class CVideoDeviceModel_8083 : public CVideoDeviceModel_8073
{
public:
    virtual int DefaultVideoMode(){ return 5; }
    virtual bool IsFloodIRSupportToggleMode() override { return false; }
    virtual int InitDeviceSelInfo();
    virtual int AddCameraPropertyModels();
    friend class CVideoDeviceModelFactory;
    std::vector<std::string> GetGainRegisterValueStringList() override;
    virtual std::vector<CIMUModel::INFO> GetIMUInfo() {
        return {
            {0x3438, 0x0166, CIMUModel::IMU_6_AXIS}
        };
    }
    virtual bool IMUSupport(){ return true; }
    int InitDeviceInformation() override;
protected:
    CVideoDeviceModel_8083(DEVSELINFO *pDeviceSelfInfo);
};

#endif // CVideoDeviceModel_8083_H
