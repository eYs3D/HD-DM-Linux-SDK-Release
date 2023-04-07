#ifndef CVIDEODEVICEMODEL_8073_H
#define CVIDEODEVICEMODEL_8073_H
#include<CVideoDeviceModel.h>

class CVideoDeviceModel_8073 : public CVideoDeviceModel
{
public:
    virtual bool InterleaveModeSupport(){ return true; }
    friend class CVideoDeviceModelFactory;
    int TransformDepthDataType(int nDepthDataType, bool bRectifyData);
    virtual int UpdateIR();
    virtual bool IRExtendSupport(){ return false; }
    virtual bool IsIRExtended(){ return false; }
    virtual int ExtendIR(bool bEnable){ return APC_NotSupport; }
    virtual int StartStreamingTask();
    virtual void SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController);
    virtual int DefaultVideoMode(){ return 12; }
    virtual bool IsManualGainSupport() { return true; }
    virtual bool IsManualGlobalGainSetSupport(){ return false; }
    virtual std::vector<std::string> GetGainRegisterValueStringList() override;
    virtual std::vector<std::string> GetGainRegisterValueStringListByDeviceSelInfo(DEVSELINFO *devSelInfo);
    virtual bool IsFloodIRSupport(){ return true; }
    virtual int SetFloodIRValue(unsigned short nValue) override;
    virtual int UpdateFloodIR() override;
    virtual int GetFloodIRRange(unsigned short &nMin, unsigned short &nMax) override;
    virtual int GetFloodIRValue() override;
    virtual int SetFloodIRToggleMode(int mode) override;
    virtual int GetFloodIRToggleMode() override;
    virtual bool IsFloodIRSupportToggleMode() override { return true; }

protected:
    static constexpr int kFWRegisterFlag = FG_Address_1Byte | FG_Value_1Byte;
    CVideoDeviceModel_8073(DEVSELINFO *pDeviceSelfInfo);
    unsigned short m_nFloodIRMax, m_nFloodIRMin, m_nFloodIRValue;
};

#endif // CVIDEODEVICEMODEL_8073_H
