#include "CVideoDeviceModel_8036.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_8036::CVideoDeviceModel_8036(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel_8036_8052(pDeviceSelfInfo),
m_bIsInterleaveSupport(false)
{

}

int CVideoDeviceModel_8036::Init()
{
    int ret = CVideoDeviceModel::Init();

    if (APC_OK != ret) return ret;

    unsigned short value;
    RETRY_APC_API(ret, APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_deviceSelInfo[0],
                                               0xe5, &value,
                                               FG_Address_1Byte | FG_Value_1Byte));

    m_bIsInterleaveSupport = (APC_OK == ret) && (1 == value);

    return ret;
}

int CVideoDeviceModel_8036::GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType)
{
    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);
    std::vector<APC_STREAM_INFO> colorStreamInfo = GetStreamInfoList(STREAM_COLOR);

    if (!depthStreamInfo.empty() && !colorStreamInfo.empty()){
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        int nColorIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_COLOR);
        if (colorStreamInfo[nColorIndex].nHeight / depthStreamInfo[nDepthIndex].nHeight == 2){
            nRectifyLogIndex = 0;
        }
    }

    return CVideoDeviceModel::GetRectifyLogData(nDevIndex, nRectifyLogIndex, pRectifyLogData, depthType);
}

int CVideoDeviceModel_8036::TransformDepthDataType(int nDepthDataType, bool bRectifyData)
{
    int ret = APC_OK;
    int depthDataType = CVideoDeviceModel::TransformDepthDataType(nDepthDataType, bRectifyData);

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);

    if (!depthStreamInfo.empty()) {
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        unsigned short PidBuf;
        unsigned short VidBuf;

        RETRY_APC_API(ret, APC_GetPidVid(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_deviceSelInfo[0],
                                               &PidBuf, &VidBuf));
        if ((PidBuf == APC_PID_8036) && (640 == depthStreamInfo[nDepthIndex].nWidth && 360 == depthStreamInfo[nDepthIndex].nHeight)) {
            char FWVersion[128];
            int nActualLength = 0;
            char *FindFwVer;
            // Apply the same FW to support 360x640 even if USB & MIPI in AMP project
            if (APC_OK == APC_GetFwVersion(CEYSDDeviceManager::GetInstance()->GetEYSD(), m_deviceSelInfo[0], FWVersion, 256, &nActualLength)) {
                FindFwVer = strstr(FWVersion, "MIPITX-SLAVE");
                if (!FindFwVer)
                    depthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
            }
        }
    }

    return depthDataType;
}
