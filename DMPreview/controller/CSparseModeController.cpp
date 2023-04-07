#include "CSparseModeController.h"
#include <QFile>
#include <QSettings>



CSparseModeController::CSparseModeController(CVideoDeviceController *pVideoDeviceController, CVideoDeviceModel *pVideoDeviceModel):
m_pVideoDeviceController(pVideoDeviceController),
m_pVideoDeviceModel(pVideoDeviceModel)
{
    m_deviceSelInfoList = m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo();
    m_pEYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
    m_devSelInfo = m_deviceSelInfoList[0];
}

CSparseModeController::~CSparseModeController()
{

}

void CSparseModeController::UpdateSparseModeConfig(bool enable)
{
    unsigned short RegValue = 0;
    unsigned short ADDRESS = 0;
    int ret = 0;
    unsigned short RegValueCheck = 0;

    ADDRESS = DEPTH_LRCHECK_BYPASS_ADDRESS;
    qDebug("ADDRESS = DEPTH_LRCHECK_BYPASS_ADDRESS\n");
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    if(enable)
        RegValue = RegValue & ~(1 << 7); //bit[7] off
    else
        RegValue = RegValue | (1 << 7); //bit[7] on
    qDebug("after bit ctrl RegValue: %04x, \n", RegValue);
    ret = APC_SetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("SetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValueCheck: %04x\n", ret, ADDRESS, RegValueCheck);

    ADDRESS = DEPTH_SAD_ENB_ADDRESS;
    qDebug("ADDRESS = DEPTH_SAD_ENB_ADDRESS\n");
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    if (enable)
        RegValue = RegValue | (1 << 2); //bit[2] on
    else
        RegValue = RegValue & ~(1 << 2); //bit[2] off
    qDebug("after bit ctrl RegValue: %04x, \n", RegValue);
    ret = APC_SetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("SetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValueCheck: %04x\n", ret, ADDRESS, RegValueCheck);

    ADDRESS = TEXT_MODE_ADDRESS;
    qDebug("ADDRESS = TEXT_MODE_ADDRESS\n");
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    if (enable)
        RegValue = RegValue | (1 << 7); //bit[7] on
    else
        RegValue = RegValue & ~(1 << 7); //bit[7] off
    qDebug("after bit ctrl RegValue: %04x, \n", RegValue);
    ret = APC_SetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("SetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValueCheck: %04x\n", ret, ADDRESS, RegValueCheck);

    ADDRESS = SPARSE_ENB_ADDRESS;
    qDebug("ADDRESS = SPARSE_ENB_ADDRESS\n");
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    if (enable)
        RegValue = RegValue | (1 << 7); //bit[7] on
    else
        RegValue = RegValue & ~(1 << 7); //bit[7] off
    qDebug("after bit ctrl RegValue: %04x, \n", RegValue);
    ret = APC_SetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("SetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValueCheck: %04x\n", ret, ADDRESS, RegValueCheck);

}

void CSparseModeController::UpdateSparseModeDefault()
{
    UpdateSparseModeReg(CONFIG_DEPTH_LRCHECK_DIFF, Default_DEPTH_LRCHECK_DIFF_Value);
    UpdateSparseModeReg(CONFIG_SGBM_SAD_THD, Default_SGBM_SAD_THD_Value);
    UpdateSparseModeReg(CONFIG_SGBM_SAD_RATIO, Default_SGBM_SAD_RATIO_Value);
    UpdateSparseModeReg(CONFIG_TEXT_LMT, Default_TEXT_LMT_Value);
    UpdateSparseModeReg(CONFIG_TEXT_PGAIN, Default_TEXT_PGAIN_Value);
    UpdateSparseModeReg(CONFIG_TEXT_NGAIN, Default_TEXT_NGAIN_Value);
    UpdateSparseModeReg(CONFIG_SPARSE_THD, Default_SPARSE_THD_Value);
}

void CSparseModeController::UpdateSparseModeReg(int SparseMode, int value)
{
    unsigned short RegValue = 0;
    unsigned short ADDRESS = 0;
    unsigned short NotValidDataRange = 0;
    int ret = 0;

    if (SparseMode == CONFIG_DEPTH_LRCHECK_DIFF)
    {
        ADDRESS = DEPTH_LRCHECK_DIFF_ADDRESS;
        NotValidDataRange = 0xf8; //bit[2:0]

        qDebug("ADDRESS = DEPTH_LRCHECK_DIFF_ADDRESS\n");
        ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
        qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
        RegValue = RegValue & NotValidDataRange;
        qDebug("NotValidDataRange RegValue: %04x, NotValidDataRange:%04x\n", RegValue, NotValidDataRange);
        RegValue |= value;
        qDebug("Data RegValue: %04x\n", RegValue);
    }
    else if ((SparseMode == CONFIG_SGBM_SAD_THD))
    {
        ADDRESS = SGBM_SAD_THD_ADDRESS;
        NotValidDataRange = 0x0f; //bit[7:4]

        qDebug("ADDRESS = SGBM_SAD_THD_ADDRESS\n");
        ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
        qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
        RegValue = RegValue & NotValidDataRange;
        qDebug("NotValidDataRange RegValue: %04x, NotValidDataRange:%04x\n", RegValue, NotValidDataRange);
        value = value & 0x0f;
        value = value << 4;
        RegValue |= value;
        qDebug("Data RegValue: %04x\n", RegValue);
    }
    else if ((SparseMode == CONFIG_SGBM_SAD_RATIO))
    {
        ADDRESS = SGBM_SAD_RATIO_ADDRESS;
        NotValidDataRange = 0xf0; //bit[3:0]

        qDebug("ADDRESS = SGBM_SAD_RATIO_ADDRESS\n");
        ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
        qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
        RegValue = RegValue & NotValidDataRange;
        qDebug("NotValidDataRange RegValue: %04x, NotValidDataRange:%04x\n", RegValue, NotValidDataRange);
        RegValue |= value;
        qDebug("Data RegValue: %04x\n", RegValue);
    }
    else if ((SparseMode == CONFIG_TEXT_LMT))
    {
        ADDRESS = TEXT_LMT_ADDRESS;
        NotValidDataRange = 0xc0; //bit[6:0]

        qDebug("ADDRESS = TEXT_LMT_ADDRESS\n");
        ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
        qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
        RegValue = RegValue & NotValidDataRange;
        qDebug("NotValidDataRange RegValue: %04x, NotValidDataRange:%04x\n", RegValue, NotValidDataRange);
        RegValue |= value;
        qDebug("Data RegValue: %04x\n", RegValue);
    }
    else if ((SparseMode == CONFIG_TEXT_PGAIN))
    {
        ADDRESS = TEXT_PGAIN_ADDRESS;
        NotValidDataRange = 0x8f; //bit[6:4]

        qDebug("ADDRESS = TEXT_PGAIN_ADDRESS\n");
        ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
        qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
        RegValue = RegValue & NotValidDataRange;
        qDebug("NotValidDataRange RegValue: %04x, NotValidDataRange:%04x\n", RegValue, NotValidDataRange);
        value = value & 0x07;
        value = value << 4;
        RegValue |= value;
        qDebug("Data RegValue: %04x\n", RegValue);
    }
    else if ((SparseMode == CONFIG_TEXT_NGAIN))
    {
        ADDRESS = TEXT_NGAIN_ADDRESS;
        NotValidDataRange = 0xf8; //bit[2:0]

        qDebug("ADDRESS = TEXT_NGAIN_ADDRESS\n");
        ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
        qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
        RegValue = RegValue & NotValidDataRange;
        qDebug("NotValidDataRange RegValue: %04x, NotValidDataRange:%04x\n", RegValue, NotValidDataRange);
        RegValue |= value;
        qDebug("Data RegValue: %04x\n", RegValue);
    }
    else if ((SparseMode == CONFIG_SPARSE_THD))
    {
        ADDRESS = SPARSE_THD_ADDRESS;
        NotValidDataRange = 0xe0; //bit[4:0]

        qDebug("ADDRESS = SPARSE_THD_ADDRESS\n");
        ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValue, FG_Address_2Byte | FG_Value_1Byte);
        qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
        RegValue = RegValue & NotValidDataRange;
        qDebug("NotValidDataRange RegValue: %04x, NotValidDataRange:%04x\n", RegValue, NotValidDataRange);
        RegValue |= value;
        qDebug("Data RegValue: %04x\n", RegValue);
    }

    ret = APC_SetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, RegValue, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("SetHWRegister ret:%d, ADDRESS:%04x RegValue: %04x\n", ret, ADDRESS, RegValue);
    unsigned short RegValueCheck = 0;
    ret = APC_GetHWRegister(m_pEYSD, m_devSelInfo, ADDRESS, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte);
    qDebug("GetHWRegister ret:%d, ADDRESS:%04x RegValueCheck: %04x\n", ret, ADDRESS, RegValueCheck);

}

void CSparseModeController::ReadConfig()
{
    QFile file_override_config(SPARSE_MODE_CONFIG_OVERRIDE_FILENAME);
    QFile file_default_config(SPARSE_MODE_CONFIG_DEFAULT_FILENAME);
    QSettings *m_setting;
    if(file_override_config.exists() ) {
        m_setting = new QSettings(SPARSE_MODE_CONFIG_OVERRIDE_FILENAME, QSettings::IniFormat);
    } else if (file_default_config.exists()) {
        m_setting = new QSettings(SPARSE_MODE_CONFIG_DEFAULT_FILENAME, QSettings::IniFormat);
    } else {
        qDebug() << "ReadConfig: config file not exists";
        return;
    }
    m_setting->setIniCodec("UTF8");
    m_setting->beginGroup(CFG_SECTION_NAME);

    Default_DEPTH_LRCHECK_DIFF_Value = m_setting->value(CFG_DEPTH_LRCHECK_DIFF).toInt();
    Default_SGBM_SAD_THD_Value = m_setting->value(CFG_SGBM_SAD_THD).toInt();
    Default_SGBM_SAD_RATIO_Value = m_setting->value(CFG_SGBM_SAD_RATIO).toInt();
    Default_TEXT_LMT_Value = m_setting->value(CFG_TEXT_LMT).toInt();
    Default_TEXT_PGAIN_Value = m_setting->value(CFG_TEXT_PGAIN).toInt();
    Default_TEXT_NGAIN_Value = m_setting->value(CFG_TEXT_NGAIN).toInt();
    Default_SPARSE_THD_Value = m_setting->value(CFG_SPARSE_THD).toInt();
    m_setting->endGroup();
    qDebug() << "ReadConfig: config file ready";
}

void CSparseModeController::DumpConfig()
{
    qDebug() << "DumpConfig Default_DEPTH_LRCHECK_DIFF_Value:" << Default_DEPTH_LRCHECK_DIFF_Value;
    qDebug() << "DumpConfig Default_SGBM_SAD_THD_Value:" << Default_SGBM_SAD_THD_Value;
    qDebug() << "DumpConfig Default_SGBM_SAD_RATIO_Value:" << Default_SGBM_SAD_RATIO_Value;
    qDebug() << "DumpConfig Default_TEXT_LMT_Value:" << Default_TEXT_LMT_Value;
    qDebug() << "DumpConfig Default_TEXT_PGAIN_Value:" << Default_TEXT_PGAIN_Value;
    qDebug() << "DumpConfig Default_TEXT_NGAIN_Value:" << Default_TEXT_NGAIN_Value;
    qDebug() << "DumpConfig Default_SPARSE_THD_Value:" << Default_SPARSE_THD_Value;
}

void CSparseModeController::WriteConfig(int SparseMode, int value)
{
    QSettings *m_setting = new QSettings(SPARSE_MODE_CONFIG_OVERRIDE_FILENAME, QSettings::IniFormat);
    m_setting->setIniCodec("UTF8");
    m_setting->beginGroup(CFG_SECTION_NAME);
    QString key;
    if (SparseMode == CONFIG_DEPTH_LRCHECK_DIFF)
    {
        key = CFG_DEPTH_LRCHECK_DIFF;
    }
    else if ((SparseMode == CONFIG_SGBM_SAD_THD))
    {
        key = CFG_SGBM_SAD_THD;
    }
    else if ((SparseMode == CONFIG_SGBM_SAD_RATIO))
    {
        key = CFG_SGBM_SAD_RATIO;
    }
    else if ((SparseMode == CONFIG_TEXT_LMT))
    {
        key = CFG_TEXT_LMT;
    }
    else if ((SparseMode == CONFIG_TEXT_PGAIN))
    {
        key = CFG_TEXT_PGAIN;
    }
    else if ((SparseMode == CONFIG_TEXT_NGAIN))
    {
        key = CFG_TEXT_NGAIN;
    }
    else if ((SparseMode == CONFIG_SPARSE_THD))
    {
        key = CFG_SPARSE_THD;
    }
    m_setting->setValue(key, value);
    m_setting->endGroup();
    m_setting->sync();
    qDebug() << "WriteConfig: key:" << key << ", value:" << value << endl;
}


