#ifndef CVIDEODEVICECONTROLLER_H
#define CVIDEODEVICECONTROLLER_H
#include <QWidget>
#include "CVideoDeviceModel.h"
#include "CEYSDUIView.h"
#include "PreviewOptions.h"
#include "CRegisterReadWriteController.h"
#include "CCameraPropertyController.h"
#include "CIMUDataController.h"
#include "CDepthAccuracyController.h"
#include "ModeConfigOptions.h"
#include "DepthFilterOptions.h"
#include "CImageProcessController.h"
#include "CSelfcalibrationController.h"
#include "CSparseModeController.h"

class CImageProcessController;
class CDepthAccuracyController;
class CSelfcalibrationController;
class CSparseModeController;
class CVideoDeviceController
{
public:
    CVideoDeviceController(CVideoDeviceModel *pVideoDeviceModel,
                           CEYSDUIView *pView);
    ~CVideoDeviceController();
    void Init();
    void UpdateImageProcessor(size_t,size_t, size_t,size_t,APCImageType::Value);
    CEYSDUIView *GetControlView(){ return m_pControlView; }
    CVideoDeviceModel *GetVideoDeviceModel(){ return m_pVideoDeviceModel; }
    PreviewOptions *GetPreviewOptions(){ return m_pPreviewOptions; }
    ModeConfigOptions *GetModeConfigOptions(){ return m_pModeConfigOptions; }
    CRegisterReadWriteController *GetRegisterReadWriteController(){ return m_pRegisterReadWriteController; }
    CCameraPropertyController *GetCameraPropertyController(){ return m_pCameraPropertyController; }
    CIMUDataController *GetIMUDataController(){ return m_pIMUDataController; }
    CDepthAccuracyController *GetDepthAccuracyController(){ return m_pDepthAccuracyController; }
    std::shared_ptr<CImageProcessController> GetImageProcessController() { return m_pImageProcessController; }
    DepthFilterOptions *GetDepthFilterOptions(){ return m_pDepthFilterOptions; }
    CSelfcalibrationController *GetSelfcalibrationController(){ return m_pSelfcalibrationController; }
    CSparseModeController *GetSparseModeController(){ return m_pSparseModeController; }


    void EnableRectifyData(bool bEnable);
    int SetDepthDataBits(int nDepthDataBits, bool bRectify);
    int SetDepthDataBits(int nDepthDataBits);
    int SetIRLevel(unsigned short nLevel);
    int SetFloodIRLevel(unsigned short nLevel);
    int SetFloodIRToggleMode(int mode);
    int EnableIRExtend(bool bEnable);
    bool IsIRExtend();
    int EnableHWPP(bool bEnable);

    int StartStreaming();
    int StopStreaming();

    int GetRectifyLogData(int nIndex, eSPCtrl_RectLogData *pRectifyLogData);
    int GetSlaveRectifyLogData(int nIndex, eSPCtrl_RectLogData *pRectifyLogData);

    int SelectModeConfigIndex(int nIndex);

    int SetModuleSync(bool bModuleSync);
    int SetModuleSyncMaster(bool bMaster);

    int SetZRange(int nZNear, int nZFar);
    int AdjustZRange();

    int UpdateStreamOptionForCombineMode(int nIndex);

    int DoSnapShot(bool bAsync = true);

    int UpdateSpecificDepthPosition(int x, int y);

    int StartIMUSyncWithFrame();
    int StopIMUSyncWithFrame();

    bool GetAutoReconnectStatus();
    void SetAutoReconnectStatus(bool auto_reconnet);

    inline int GetColorResizeOption();
    void SetColorResizeOption(int index);

    inline int GetDepthResizeOption();
    void SetDepthResizeOption(int index);

    inline bool GetEnableColorResizeOption();
    void SetEnableColorResizeOption(bool isEnable);

    inline bool GetEnableDepthResizeOption();
    void SetEnableDepthResizeOption(bool isEnable);
private:
    int SetDepthDataType(int depthDataType);

    int SaveBitmap(char *pFilePath,
                   BYTE *pBuffer,
                   unsigned short nWidth, unsigned short nHeight,
                   unsigned short nBytePerPixel);
    int SaveYUV(char *pFilePath,
                BYTE *pBuffer,
                unsigned short nWidth, unsigned short nHeight,
                unsigned short nBytePerPixel);
    int SavePly(char *pFilePath, std::vector<CloudPoint> cloudPoints);

private:
    CVideoDeviceModel *m_pVideoDeviceModel;
    CEYSDUIView *m_pControlView;
    PreviewOptions    *m_pPreviewOptions;
    ModeConfigOptions *m_pModeConfigOptions;
    CRegisterReadWriteController *m_pRegisterReadWriteController;
    CCameraPropertyController *m_pCameraPropertyController;
    CIMUDataController *m_pIMUDataController;
    CDepthAccuracyController *m_pDepthAccuracyController;
    std::shared_ptr<CImageProcessController> m_pImageProcessController;
    DepthFilterOptions *m_pDepthFilterOptions;
    CSelfcalibrationController *m_pSelfcalibrationController;
    CSparseModeController *m_pSparseModeController;

};

#endif // CVIDEODEVICECONTROLLER_H
