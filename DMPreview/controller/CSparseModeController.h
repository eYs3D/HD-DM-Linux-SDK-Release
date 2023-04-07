#ifndef CSPARSEMODECONTROLLER_H
#define CSPARSEMODECONTROLLER_H

#include "CVideoDeviceModel.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

#define SPARSE_MODE_CONFIG_DEFAULT_FILENAME "./../settings/APC_SparseModeConfig_Default.ini"
#define SPARSE_MODE_CONFIG_OVERRIDE_FILENAME "./../settings/APC_SparseModeConfig_Override.ini"
#define CFG_SECTION_NAME "SPARSE_MODE"
#define CFG_DEPTH_LRCHECK_DIFF "DEPTH_LRCHECK_DIFF"
#define CFG_SGBM_SAD_THD "SGBM_SAD_THD"
#define CFG_SGBM_SAD_RATIO "SGBM_SAD_RATIO"
#define CFG_TEXT_LMT "TEXT_LMT"
#define CFG_TEXT_PGAIN "TEXT_PGAIN"
#define CFG_TEXT_NGAIN "TEXT_NGAIN"
#define CFG_SPARSE_THD "SPARSE_THD"

class CSparseModeController
{
public:
    static const int DEPTH_LRCHECK_DIFF_ADDRESS = 0xf410;
    static const int SGBM_SAD_THD_ADDRESS = 0xf409;
    static const int SGBM_SAD_RATIO_ADDRESS = 0xf409;
    static const int TEXT_LMT_ADDRESS = 0xf429;
    static const int TEXT_PGAIN_ADDRESS = 0xf42A;
    static const int TEXT_NGAIN_ADDRESS = 0xf42A;
    static const int SPARSE_THD_ADDRESS = 0xf430;
    static const int DEPTH_LRCHECK_BYPASS_ADDRESS = 0xf410;
    static const int DEPTH_SAD_ENB_ADDRESS = 0xf402;
    static const int TEXT_MODE_ADDRESS = 0xf429;
    static const int SPARSE_ENB_ADDRESS = 0xf430;

    enum
    {
        CONFIG_DEPTH_LRCHECK_DIFF,
        CONFIG_SGBM_SAD_THD,
        CONFIG_SGBM_SAD_RATIO,
        CONFIG_TEXT_LMT,
        CONFIG_TEXT_PGAIN,
        CONFIG_TEXT_NGAIN,
        CONFIG_SPARSE_THD
    };

    int Default_DEPTH_LRCHECK_DIFF_Value = 7;
    int Default_SGBM_SAD_THD_Value = 4;
    int Default_SGBM_SAD_RATIO_Value = 5;
    int Default_TEXT_LMT_Value = 127;
    int Default_TEXT_PGAIN_Value = 7;
    int Default_TEXT_NGAIN_Value = 7;
    int Default_SPARSE_THD_Value = 8;

public:
    CSparseModeController(CVideoDeviceController *pVideoDeviceController, CVideoDeviceModel *pVideoDeviceModel);
    ~CSparseModeController();

    void UpdateSparseModeConfig(bool enable);
    void UpdateSparseModeDefault();
    void UpdateSparseModeReg(int SparseMode, int value);
    void ReadConfig();
    void DumpConfig();
    void WriteConfig(int SparseMode, int value);

private:


private:
    //camera
    CVideoDeviceController *m_pVideoDeviceController;
    CVideoDeviceModel *m_pVideoDeviceModel;
    std::vector<DEVSELINFO *> m_deviceSelInfoList;
    std::vector<CVideoDeviceModel::DeviceInfo> m_deviceInfoList;
    void* m_pEYSD;
    DEVSELINFO* m_devSelInfo;
};

#endif // CSPARSEMODECONTROLLER_H
