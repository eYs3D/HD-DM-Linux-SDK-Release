#include "CCameraPropertyController.h"

CCameraPropertyController::CCameraPropertyController(CVideoDeviceModel *pVideoDeviceModel):
m_pVideoDeviceModel(pVideoDeviceModel),
m_nCurrentIndex(0)
{

}

CCameraPropertyController::~CCameraPropertyController()
{

}

void CCameraPropertyController::SelectCurrentCameraProperty(int nIndex)
{
    if((size_t)nIndex >= m_pVideoDeviceModel->GetCameraproperty().size()) return;

    m_nCurrentIndex = nIndex;
}

unsigned short CCameraPropertyController::GetCameraPropertyDeviceCount()
{
    return m_pVideoDeviceModel->GetCameraproperty().size();
}

QString CCameraPropertyController::GetCameraPropertyDeviceName(int nIndex)
{
    if((size_t)nIndex >= m_pVideoDeviceModel->GetCameraproperty().size()) return "";

    return m_pVideoDeviceModel->GetCameraproperty()[nIndex]->GetDeviceName();
}

bool CCameraPropertyController::IsCameraPropertySupport(CCameraPropertyModel::CAMERA_PROPERTY type)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type).bSupport;
}

bool CCameraPropertyController::IsCameraPropertyValid(CCameraPropertyModel::CAMERA_PROPERTY type)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type).bValid;
}

int CCameraPropertyController::GetRange(CCameraPropertyModel::CAMERA_PROPERTY type, int &nMin, int &nMax)
{
    CCameraPropertyModel::CameraPropertyItem item = m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type);

    nMin = item.nMin;
    nMax = item.nMax;

    return APC_OK;
}

int CCameraPropertyController::GetValue(CCameraPropertyModel::CAMERA_PROPERTY type, int &nValue)
{
    nValue = m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetCameraProperty(type).nValue;
    return APC_OK;
}

int CCameraPropertyController::SetValue(CCameraPropertyModel::CAMERA_PROPERTY type, int nValue)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetCameraPropertyValue(type, nValue);
}

int CCameraPropertyController::ResetCameraProperty()
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetDefaultCameraProperty();
}

bool CCameraPropertyController::IsManualExposureValid()
{
    int nValue;
    GetValue(CCameraPropertyModel::AUTO_EXPOSURE, nValue);
    return CVideoDeviceModel::STREAMING == m_pVideoDeviceModel->GetState() &&
           nValue != 1;
}

bool CCameraPropertyController::IsManualGainSupport() {
    return m_pVideoDeviceModel->IsManualGainSupport();
}

bool CCameraPropertyController::IsManualGlobalGainSetSupport() {
    return m_pVideoDeviceModel->IsManualGlobalGainSetSupport();
}

float CCameraPropertyController::GetManuelExposureTimeMs()
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetManuelExposureTimeMs();
}

void CCameraPropertyController::SetManuelExposureTimeMs(float fMs)
{
    if (!IsManualExposureValid()) return;

    m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetManuelExposureTimeMs(fMs);
}

float CCameraPropertyController::GetManuelGlobalGain()
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetManuelGlobalGain();
}

float CCameraPropertyController::GetManualAnalogGain() {
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetManualAnalogGain();
}

void CCameraPropertyController::SetManualAnalogGain(float fAnalogGain) {
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetManualAnalogGain(fAnalogGain);
}

float CCameraPropertyController::GetManualDigitalGain() {
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetManualDigitalGain();
}

void CCameraPropertyController::SetManualDigitalGain(float fDigitalGain) {
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetManualDigitalGain(fDigitalGain);
}

QStringList CCameraPropertyController::GetManualGainRegisterValueStringList() {
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->GetManualGainRegisterValues();
}

void CCameraPropertyController::SetManuelGlobalGain(float fGlobalGain)
{
    if (!IsManualExposureValid()) return;

    m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetManuelGlobalGain(fGlobalGain);
}

float CCameraPropertyController::SetAETarget(int index)
{
    return m_pVideoDeviceModel->GetCameraproperty()[m_nCurrentIndex]->SetAETargetIndex(index);
}
