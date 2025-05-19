#include "CVideoDeviceModel_Hypatia2.h"
#include "CVideoDeviceController.h"

CVideoDeviceModel_Hypatia2::CVideoDeviceModel_Hypatia2(DEVSELINFO *pDeviceSelfInfo) : CVideoDeviceModel(pDeviceSelfInfo)
{
}

int CVideoDeviceModel_Hypatia2::TransformDepthDataType(int nDepthDataType, bool bRectifyData) {
    return nDepthDataType == APC_DEPTH_DATA_14_BITS ? m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().videoModeZ14:
                                                      m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().videoModeD11OrColorOnly;
}

int CVideoDeviceModel_Hypatia2::AdjustZDTableIndex(int &nIndex) {
    nIndex = 0;
    return APC_OK; // Currently using scale down
}

int CVideoDeviceModel_Hypatia2::PreparePointCloudInfo() {
    constexpr unsigned short kFileIndexHypatia2And4 = 0;
    GetRectifyLogData(0, kFileIndexHypatia2And4, &m_rectifyLogData);
    GetPointCloudInfo(&m_rectifyLogData, m_pointCloudInfo, GetColorImageData(), GetDepthImageData());
    return APC_OK;
}
