#include "CVideoDeviceModel_8073.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_8073::CVideoDeviceModel_8073(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}
int CVideoDeviceModel_8073::TransformDepthDataType(int nDepthDataType, bool bRectifyData) {
    // Due to sensor type is customized, so interleave mode did not follow video mode 0.4b
    switch (nDepthDataType){
        case APC_DEPTH_DATA_8_BITS:
        case APC_DEPTH_DATA_8_BITS_RAW:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS : APC_DEPTH_DATA_8_BITS_RAW; break;
        case APC_DEPTH_DATA_8_BITS_x80:
        case APC_DEPTH_DATA_8_BITS_x80_RAW:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS_x80 : APC_DEPTH_DATA_8_BITS_x80_RAW; break;
        case APC_DEPTH_DATA_11_BITS:
        case APC_DEPTH_DATA_11_BITS_RAW:
        case APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_11_BITS : APC_DEPTH_DATA_11_BITS_RAW;  break;
        case APC_DEPTH_DATA_14_BITS:
        case APC_DEPTH_DATA_14_BITS_RAW:
        case APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_14_BITS : APC_DEPTH_DATA_14_BITS_RAW;  break;
        case APC_DEPTH_DATA_OFF_RAW:
        case APC_DEPTH_DATA_OFF_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_OFF_RECTIFY : APC_DEPTH_DATA_OFF_RAW;  break;
        default:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_DEFAULT : APC_DEPTH_DATA_OFF_RECTIFY;  break;
    }
    return nDepthDataType;
}
int CVideoDeviceModel_8073::UpdateIR()
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    int ret;
    ret = APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                            m_deviceSelInfo[0],
                            0xE2, 96,
                            FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret) return ret;

    return CVideoDeviceModel::UpdateIR();
}
// ++++++ PM / FAE requested function for POC version for robotic application

void CVideoDeviceModel_8073::SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController)
{
    CVideoDeviceModel::SetVideoDeviceController(pVideoDeviceController);
    if(m_pVideoDeviceController){
        m_pVideoDeviceController->GetPreviewOptions()->SetIRLevel(20);
    }
}

int CVideoDeviceModel_8073::StartStreamingTask(){
    CVideoDeviceModel::StartStreamingTask();

    if(m_pVideoDeviceController){
        m_pVideoDeviceController->GetPreviewOptions()->SetIRLevel(20);
    }

    return APC_OK;
}

std::vector<std::string> CVideoDeviceModel_8073::GetGainRegisterValueStringListByDeviceSelInfo(DEVSELINFO *devSelInfo) {
    if (!devSelInfo) {
        qDebug() << "GetGainRegisterValueStringListByDeviceSelInfo nullptr \n";
        return std::vector<std::string>();
    }

    SENSORMODE_INFO nSensorMode = SENSOR_BOTH;
    constexpr int kSensorRegisterFlag = FG_Address_2Byte | FG_Value_1Byte;
    constexpr int kSensorSlaveAddress = 0x20;
    unsigned short gainReg;
    unsigned short gainValue;
    std::vector<std::pair<unsigned short, unsigned short>> gainRegisters;
    std::vector<std::string> resultStdStringList;

    gainRegisters.push_back(std::make_pair(0x044D, 0x0));
    gainRegisters.push_back(std::make_pair(0x0450, 0x0));
    gainRegisters.push_back(std::make_pair(0x0451, 0x0));
    gainRegisters.push_back(std::make_pair(0x0452, 0x0));
    gainRegisters.push_back(std::make_pair(0x0453, 0x0));
    gainRegisters.push_back(std::make_pair(0x0454, 0x0));
    gainRegisters.push_back(std::make_pair(0x0455, 0x0));
    gainRegisters.push_back(std::make_pair(0x0456, 0x0));
    gainRegisters.push_back(std::make_pair(0x0457, 0x0));

    for (auto it : gainRegisters) {
        int ret = APC_GetSensorRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), devSelInfo,
                                        kSensorSlaveAddress, it.first, &it.second, kSensorRegisterFlag, nSensorMode);
        char buff[100];
        snprintf(buff, sizeof(buff), "Address 0x%x Value 0x%x", it.first, it.second);
        std::string buffAsStdStr(buff);
        resultStdStringList.push_back(std::move(buffAsStdStr));
    }

    return std::move(resultStdStringList);
}

std::vector<std::string> CVideoDeviceModel_8073::GetGainRegisterValueStringList() {
    return std::move(GetGainRegisterValueStringListByDeviceSelInfo(m_deviceSelInfo[0]));
}

int CVideoDeviceModel_8073::SetFloodIRValue(unsigned short nValue) {
    int ret = APC_DEVICE_NOT_SUPPORT;

    RETRY_APC_API(ret, APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xE6, nValue, kFWRegisterFlag));
    return APC_OK;
}

int CVideoDeviceModel_8073::UpdateFloodIR() {
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    int ret = APC_DEVICE_NOT_SUPPORT;
    unsigned short nValue = 0x0;

    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xE6, &m_nFloodIRValue, kFWRegisterFlag));
    if (APC_OK != ret) return ret;

    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xE7, &m_nFloodIRMin, kFWRegisterFlag));
    if (APC_OK != ret) return ret;

    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xE8, &m_nFloodIRValue, kFWRegisterFlag));
    if (APC_OK != ret) return ret;

    return ret;
}

int CVideoDeviceModel_8073::GetFloodIRRange(unsigned short &nMin, unsigned short &nMax) {
    int ret = APC_DEVICE_NOT_SUPPORT;
    unsigned short nValue = 0x0;

    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xE7, &nValue, kFWRegisterFlag));
    nMin = nValue;

    nValue = 0x0;
    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xE8, &nValue, kFWRegisterFlag));
    nMax = nValue;
    return ret;
}

int CVideoDeviceModel_8073::GetFloodIRValue() {
    int ret = APC_DEVICE_NOT_SUPPORT;
    unsigned short nValue = 0x0;
    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xE6, &nValue, kFWRegisterFlag));
    return ret < 0 ? ret : nValue;
}

int CVideoDeviceModel_8073::GetFloodIRToggleMode() {
    int ret = APC_DEVICE_NOT_SUPPORT;
    unsigned short nValue = 0x0;
    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xEA, &nValue, kFWRegisterFlag));
    return nValue;
}

int CVideoDeviceModel_8073::SetFloodIRToggleMode(int nValue) {
    int ret = APC_DEVICE_NOT_SUPPORT;
    RETRY_APC_API(ret, APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0],
                                         0xEA, (unsigned short) nValue, kFWRegisterFlag));
    return nValue;
}
// ------ PM / FAE requested function for POC version for robotic application
