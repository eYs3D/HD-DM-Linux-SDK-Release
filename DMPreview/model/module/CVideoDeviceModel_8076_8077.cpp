#include "CVideoDeviceModel_8076_8077.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"
#include "RegisterSettings.h"

CVideoDeviceModel_8076_8077::CVideoDeviceModel_8076_8077(DEVSELINFO *pDeviceSelfInfo) : CVideoDeviceModel(pDeviceSelfInfo)
{

}

bool CVideoDeviceModel_8076_8077::isScaleDown() {
    bool isScaleDowned = false;
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);
    std::vector<APC_STREAM_INFO> colorStreamInfo = GetStreamInfoList(STREAM_COLOR);

    if (bDepthStream && !depthStreamInfo.empty() && bColorStream && !colorStreamInfo.empty()) {
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        int nColorIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_COLOR);

        if (colorStreamInfo[nColorIndex].nHeight == 720 && depthStreamInfo[nDepthIndex].nHeight == 360) {
            isScaleDowned = true;
        }
    }

    return isScaleDowned;
}

int CVideoDeviceModel_8076_8077::AdjustZDTableIndex(int& index) {
    if (isScaleDown()) index = 0;
    return APC_OK;
}
bool CVideoDeviceModel_8076_8077::ModuleSyncSupport()
{
    if(USB_PORT_TYPE_3_0 != GetUsbType()) return false;
    if(IsInterleaveMode()) return false;
    return true;
}

int CVideoDeviceModel_8076_8077::ModuleSync()
{
    if(STREAMING != GetState()) return APC_OK;

    int ret = APC_OK;
    if(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSync()){
        ret = RegisterSettings::FrameSync8053_8059_Clock(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                         m_deviceSelInfo[0]);
    }
    return ret;
}

int CVideoDeviceModel_8076_8077::ModuleSyncReset()
{
    int ret = APC_OK;
    if(m_pVideoDeviceController->GetPreviewOptions()->IsModuleSyncMaster()){
        static QMutex mutex;
        QMutexLocker locker(&mutex);
        ret = RegisterSettings::FrameSync8053_8059_Reset(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                         m_deviceSelInfo[0]);
    }
    return ret;
}

int CVideoDeviceModel_8076_8077::ProcessImageCallback(STREAM_TYPE streamType,
                                                      int nImageSize, int nSerialNumber)
{
    if(STREAM_COLOR == streamType && m_pVideoDeviceController->GetPreviewOptions()->IsModuleSyncMaster()){
        if(nSerialNumber > m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(streamType) * 30){
            ModuleSyncReset();
        }
    }

    return CVideoDeviceModel::ProcessImageCallback(streamType, nImageSize, nSerialNumber);
}

int CVideoDeviceModel_8076_8077::GetCurrentTemperature(float& fTemperature)
{
    int ret = APC_OK;
    unsigned short TempeRegAddr = 0x00;
    unsigned short TempeRegVal = 0x00;
    bool is_negtive = false;
    constexpr unsigned short THERMAL_SENSOR_ID = 0x90;

    ret = APC_GetSensorRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0], THERMAL_SENSOR_ID,
                                TempeRegAddr, &TempeRegVal, FG_Address_1Byte | FG_Value_2Byte, SENSOR_BOTH);

    if (ret != APC_OK) return ret;

    qDebug("TempeRegVal = (0x%04x)\n", TempeRegVal);
    TempeRegVal = __bswap_16(TempeRegVal);
    TempeRegVal >>= 5;
    is_negtive = TempeRegVal & (1 << 10);
    if (is_negtive)
        TempeRegVal |= (0x1f << 11);
    fTemperature = (float)TempeRegVal * 0.125;
    qDebug("TempeRegVal = (%f)\n", fTemperature);

    return ret;
}

int CVideoDeviceModel_8076_8077::TransformDepthDataType(int nDepthDataType, bool bRectifyData)
{
    int ret = APC_OK;
    int depthDataType = CVideoDeviceModel::TransformDepthDataType(nDepthDataType, bRectifyData);

    if (isScaleDown()) {
        depthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
    }

    return depthDataType;
}
