#ifndef CCAMERAPROPERTYCONTROLLER_H
#define CCAMERAPROPERTYCONTROLLER_H

#include "CVideoDeviceModel.h"

class CCameraPropertyController
{
public:

    CCameraPropertyController(CVideoDeviceModel *pVideoDeviceModel);
    ~CCameraPropertyController();

    int GetCurrentCameraPropertyDeviceIndex(){ return m_nCurrentIndex; }
    void SelectCurrentCameraProperty(int nIndex);

    unsigned short GetCameraPropertyDeviceCount();

    QString GetCameraPropertyDeviceName(int index);
    bool IsCameraPropertySupport(CCameraPropertyModel::CAMERA_PROPERTY type);
    bool IsCameraPropertyValid(CCameraPropertyModel::CAMERA_PROPERTY type);
    int GetRange(CCameraPropertyModel::CAMERA_PROPERTY type, int &nMin, int &nMax);
    int GetValue(CCameraPropertyModel::CAMERA_PROPERTY type, int &nValue);
    int SetValue(CCameraPropertyModel::CAMERA_PROPERTY type, int nValue);
    int ResetCameraProperty();

    /**
     * @brief IsManualExposureValid Manual Exposure Time + Global Analog Digital Gains
     * In sensor data sheet state that AUTO_EXPOSURE should turn off such that developers
     * are allowed to set exposure (Time + Gains)
     * @return if can set manual exposure or not.
     */
    bool IsManualExposureValid();
    bool IsManualGainSupport();
    bool IsManualGlobalGainSetSupport();
    float GetManuelExposureTimeMs();
    void SetManuelExposureTimeMs(float fMs);
    float GetManuelGlobalGain();
    void SetManuelGlobalGain(float fGlobalGain);
    float GetManualAnalogGain();
    void SetManualAnalogGain(float fAnalogGain);
    float GetManualDigitalGain();
    void SetManualDigitalGain(float fDigitalGain);
    QStringList GetManualGainRegisterValueStringList();
    float SetAETarget(int index);

private:
    CVideoDeviceModel *m_pVideoDeviceModel;
    int m_nCurrentIndex;
};

#endif // CCAMERAPROPERTYCONTROLLER_H
