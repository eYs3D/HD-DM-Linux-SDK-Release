#include "CVideoDeviceModel_Iris.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_Iris::CVideoDeviceModel_Iris(DEVSELINFO *pDeviceSelfInfo) : CVideoDeviceModel(pDeviceSelfInfo)
{
}

int CVideoDeviceModel_Iris::TransformDepthDataType(int nDepthDataType, bool bRectifyData)
{
    int depthDataType = CVideoDeviceModel::TransformDepthDataType(nDepthDataType, bRectifyData);

    std::vector<APC_STREAM_INFO> depthStreamInfo = GetStreamInfoList(STREAM_DEPTH);

    if (!depthStreamInfo.empty())
    {
        int nDepthIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        if (640 == depthStreamInfo[nDepthIndex].nWidth && 360 == depthStreamInfo[nDepthIndex].nHeight)
        {
            depthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        }
    }
    return depthDataType;
}

int CVideoDeviceModel_Iris::AdjustZDTableIndex(int &nIndex)
{
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH))
    {
        return APC_OK;
    }

    nIndex = 0;
    
    return APC_OK;
}
