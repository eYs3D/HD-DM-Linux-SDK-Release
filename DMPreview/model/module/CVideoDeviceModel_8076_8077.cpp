#include "CVideoDeviceModel_8076_8077.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_8076_8077::CVideoDeviceModel_8076_8077(DEVSELINFO *pDeviceSelfInfo) : CVideoDeviceModel(pDeviceSelfInfo)
{
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
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);

    if (bDepthStream && !depthStreamInfo.empty()) {
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

        bool isScaleDownedHeight = false;
        switch(depthStreamInfo[nDepthIndex].nHeight) {
            case 480:
            case 360:
            case 270:
            case 240:
                fprintf(stderr, "Scale Down \n");
                isScaleDownedHeight = true;
                break;
            default:
                break;
        }

        unsigned short pidBuf = GetDeviceInformation().at(0).deviceInfomation.wPID;
        if (pidBuf == APC_PID_8076 && isScaleDownedHeight) {
            depthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        }
    }

    return depthDataType;
}
