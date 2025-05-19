#ifndef CVIDEODEVICEMODEL_IVY3_H
#define CVIDEODEVICEMODEL_IVY3_H
#include<CVideoDeviceModel.h>

class CVideoDeviceModel_IVY3 : public CVideoDeviceModel
{
public:
    virtual bool InterleaveModeSupport(){ return true; }
    friend class CVideoDeviceModelFactory;
    virtual bool IRExtendSupport(){ return false; }
    virtual bool IsIRExtended(){ return false; }
    virtual int ExtendIR(bool bEnable){ return APC_NotSupport; }
    virtual int DefaultVideoMode(){ return 1; }
    virtual bool IsManualGainSupport() { return true; }
    virtual bool IsManualGlobalGainSetSupport() { return false; }
    virtual std::vector<std::string> GetGainRegisterValueStringList() override;
    virtual std::vector<std::string> GetGainRegisterValueStringListByDeviceSelInfo(DEVSELINFO *devSelInfo);

protected:
    static constexpr int kFWRegisterFlag = FG_Address_1Byte | FG_Value_1Byte;
    CVideoDeviceModel_IVY3(DEVSELINFO *pDeviceSelfInfo);
    unsigned short m_nFloodIRMax, m_nFloodIRMin, m_nFloodIRValue;
};

#endif // CVIDEODEVICEMODEL_IVY3_H
