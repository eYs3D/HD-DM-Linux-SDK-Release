#include "CVideoDeviceModel_8083.h"

CVideoDeviceModel_8083::CVideoDeviceModel_8083(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel_8073(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8083::InitDeviceSelInfo() {
    CVideoDeviceModel::InitDeviceSelInfo();

    if(m_deviceSelInfo.empty()) return APC_NullPtr;

    // Adding 2nd device eSP777
    DEVSELINFO *pDevSelfInfo = new DEVSELINFO;
    pDevSelfInfo->index = m_deviceSelInfo[0]->index + 1;
    m_deviceSelInfo.push_back(pDevSelfInfo);

    return APC_OK;
}

int CVideoDeviceModel_8083::InitDeviceInformation() {
    CVideoDeviceModel::InitDeviceInformation();
    // Adding 2nd device eSP777
    m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[1]));
    return APC_OK;
}

int CVideoDeviceModel_8083::AddCameraPropertyModels() {
    CCameraPropertyModel *pNewCameraPropertyModel = new CCameraPropertyModel("Color_eSP777", this, m_deviceSelInfo[1]);
    m_cameraPropertyModel.push_back(std::move(pNewCameraPropertyModel));
    return APC_OK;
}

std::vector<std::string> CVideoDeviceModel_8083::GetGainRegisterValueStringList() {
    return std::move(GetGainRegisterValueStringListByDeviceSelInfo(m_deviceSelInfo[1]));
}
