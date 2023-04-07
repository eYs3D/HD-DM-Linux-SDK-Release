#ifndef CSELFCALIBRATIONCONTROLLER_H
#define CSELFCALIBRATIONCONTROLLER_H

#include "CVideoDeviceModel.h"
#include "CVideoDeviceController.h"
#include "ModuleCalibParWrite.h"
#include "Self_Calibration_API.h"

class CSelfcalibrationController
{

public:
    CSelfcalibrationController(CVideoDeviceController *pVideoDeviceController, CVideoDeviceModel *pVideoDeviceModel);
    ~CSelfcalibrationController();

    bool isLRDMode();
    bool isStreming();
    bool checkLRDMode();
    void getImageParams();
    void getColorImage();

    int executeSelfK();
    int releaseSelfK();
    void disableSelfK();
    int RectLogData2ModuleParameter(eSPCtrl_RectLogData &rectifyData, self_calibration::module_parameters &moduleParameter);
    int ModuleParameter2RectLogData(self_calibration::module_parameters &moduleParameter, eSPCtrl_RectLogData &rectifyData);
    self_calibration::self_calibration_api *GetSelf_calibration() { return p_self_k; }
    self_calibration::self_calibration_issue operator_calculate_score();
    self_calibration::self_calibration_issue operator_do_calibration();
    void operator_apply_before_reg();
    void operator_apply_after_reg();
    void operator_apply_after_flash();
    void operator_reset_all();

private:
    void init();
    void BGR2Gray(unsigned char *src, unsigned char *dst, int width, int height);
    void RGB2Gray(unsigned char *src, unsigned char *dst, int width, int height);
    void saveYUY2(BYTE *pBuffer, int pBuffer_size);
    void saveRGB(BYTE *pBuffer, int pBuffer_size);
    void saveGray(BYTE *pBuffer, int pBuffer_size);
    int initSelfK();
    void show_calibration_issue(self_calibration::self_calibration_issue issue);

private:
    //camera
    CVideoDeviceController *m_pVideoDeviceController;
    CVideoDeviceModel *m_pVideoDeviceModel;
    std::vector<DEVSELINFO *> m_deviceSelInfoList;
    std::vector<CVideoDeviceModel::DeviceInfo> m_deviceInfoList;

    //Self k
    self_calibration::self_calibration_api *p_self_k = new self_calibration::self_calibration_api;
    eSPCtrl_RectLogData *p_rectifyLogData = nullptr;
    BOOL m_IsLRD_Mode;
    int m_nColorResWidth;
    int m_nColorResHeight;
    int m_colorNum = 0;
    BOOL mDumpColorYUY2Frame;
    BOOL mDumpColorRGBFrame;
    BOOL mDumpColorGrayFrame;

    int color_width;
    int color_height;
    int depth_width;
    int depth_height;
    int res_depth_option_id;
    unsigned char *m_color_sbs = nullptr;
    unsigned char *m_gray_sbs = nullptr;
    unsigned char *m_color = nullptr;
    unsigned char *m_depth = nullptr;
};

#endif // CSELFCALIBRATIONCONTROLLER_H
