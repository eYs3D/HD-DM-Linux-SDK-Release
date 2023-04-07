#include "CVideoDeviceModel_8081.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_8081::CVideoDeviceModel_8081(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_8081::UpdateIR()
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

void CVideoDeviceModel_8081::SetVideoDeviceController(CVideoDeviceController *pVideoDeviceController)
{
    CVideoDeviceModel::SetVideoDeviceController(pVideoDeviceController);
    if(m_pVideoDeviceController){
        m_pVideoDeviceController->GetPreviewOptions()->SetIRLevel(60);
    }
}

int CVideoDeviceModel_8081::AdjustZDTableIndex(int &nIndex) {
    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH)){
        return APC_OK;
    }

    nIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);

    return APC_OK;
}

