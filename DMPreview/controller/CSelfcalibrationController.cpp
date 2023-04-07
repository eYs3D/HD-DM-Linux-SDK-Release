#include "CSelfcalibrationController.h"
#include "CVideoDeviceModel.h"
#include "CImageDataModel.h"
#include "CEYSDDeviceManager.h"


CSelfcalibrationController::CSelfcalibrationController(CVideoDeviceController *pVideoDeviceController, CVideoDeviceModel *pVideoDeviceModel):
m_pVideoDeviceController(pVideoDeviceController),
m_pVideoDeviceModel(pVideoDeviceModel)
{
    m_nColorResWidth = 0;
    m_nColorResHeight = 0;
    m_IsLRD_Mode = false;
    mDumpColorYUY2Frame = false;
    mDumpColorRGBFrame = false;
    mDumpColorGrayFrame = false;
    init();
}

CSelfcalibrationController::~CSelfcalibrationController()
{
    releaseSelfK();
}

void CSelfcalibrationController::init()
{
    m_deviceSelInfoList = m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo();

    auto EYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
    auto devSelInfo = m_deviceSelInfoList[0];
    unsigned short value;
    APC_GetFWRegister(EYSD, devSelInfo, 0xf6, &value, FG_Address_1Byte | FG_Value_1Byte);
    res_depth_option_id = int(value);
    qDebug() << "CSelfcalibrationController init res_depth_option_id" << res_depth_option_id;

}

bool CSelfcalibrationController::isLRDMode()
{
    QString csModeDesc = m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().csModeDesc;
    qDebug() << "isLRDMode csModeDesc" << csModeDesc;
    if (csModeDesc.isEmpty() || csModeDesc.isNull()) {
        int pid = m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceInformation()[0].deviceInfomation.wPID;
        CVideoDeviceModel::ImageData color_imageData = m_pVideoDeviceController->GetVideoDeviceModel()->GetColorImageData();
        CVideoDeviceModel::ImageData depth_imageData = m_pVideoDeviceController->GetVideoDeviceModel()->GetDepthImageData();
        m_IsLRD_Mode = ( ( 2560 == color_imageData.nWidth && 1280 == depth_imageData.nWidth ) || ( 2560 == color_imageData.nWidth && 640 == depth_imageData.nWidth )
            || ( 1280 == color_imageData.nWidth && 640 == depth_imageData.nWidth ) || ( 640 == color_imageData.nWidth && 320 == depth_imageData.nWidth )
            || (2208 == color_imageData.nWidth && 1104 == depth_imageData.nWidth ) );
    } else {
        QString stringLRDr = "L'+R'+D";
        QString stringLRD = "L+R+D";
        m_IsLRD_Mode = ( (QString::compare(csModeDesc, stringLRDr) == 0) || (QString::compare(csModeDesc, stringLRD) == 0) );
    }
    qDebug() << "isLRDMode m_IsLRD_Mode" << m_IsLRD_Mode;
    return m_IsLRD_Mode;
}

bool CSelfcalibrationController::isStreming()
{
    bool isStreming = false;
    isStreming = ( CVideoDeviceModel::STREAMING == m_pVideoDeviceController->GetVideoDeviceModel()->GetState());
    qDebug() << "isStreming isStreming" << isStreming;
    return isStreming;
}

bool CSelfcalibrationController::checkLRDMode()
{
    if (!isStreming() || !isLRDMode()) {
        return false;
    }
    return true;
}

void CSelfcalibrationController::getImageParams()
{
    qDebug() << "getImageParams";
    CVideoDeviceModel::ImageData color_imageData = m_pVideoDeviceController->GetVideoDeviceModel()->GetColorImageData();
    CVideoDeviceModel::ImageData depth_imageData = m_pVideoDeviceController->GetVideoDeviceModel()->GetDepthImageData();
    m_nColorResWidth = color_imageData.nWidth;
    m_nColorResHeight = color_imageData.nHeight;
    color_width = color_imageData.nWidth;
    color_height = color_imageData.nHeight;
    depth_width = depth_imageData.nWidth;
    depth_height = depth_imageData.nHeight;

    //Gray color
    if(!m_color) {
        qDebug() << "init m_color";
        m_color = (unsigned char *)malloc(color_width * color_height * sizeof(unsigned char));
        memset(m_color, 0, color_width * color_height * sizeof(unsigned char));
    }

    if(!m_depth) {
        m_depth = (unsigned char *)malloc(depth_height * depth_width * sizeof(unsigned char));
    }
    initSelfK();
}


void CSelfcalibrationController::BGR2Gray(unsigned char *src, unsigned char *dst, int width, int height)
{
    if (src == NULL) {
        qDebug() << "src is null";
        return;
    }

    if (dst == NULL) {
        qDebug() << "dst is null";
        return;
    }

    for (int i = 0; i < width * height; i++) {
        int r = (int)src[3 * i + 2];
        int g = (int)src[3 * i + 1];
        int b = (int)src[3 * i + 0];
        dst[i] = (r * 38 + g * 75 + b * 15) >> 7;
    }
}

void CSelfcalibrationController::RGB2Gray(unsigned char *src, unsigned char *dst, int width, int height)
{
    if (src == NULL) {
        qDebug() << "src is null";
        return;
    }

    if (dst == NULL) {
        qDebug() << "dst is null";
        return;
    }

    for (int i = 0; i < width * height; i++) {
        int r = (int)src[3 * i + 0];
        int g = (int)src[3 * i + 1];
        int b = (int)src[3 * i + 2];
        dst[i] = (r * 38 + g * 75 + b * 15) >> 7;
    }
}

void CSelfcalibrationController::saveYUY2(BYTE *pBuffer, int pBuffer_size)
{
    QDate currentDate = QDate::currentDate();
    QTime currentTime = QTime::currentTime();
    char timeStamp[64];
    sprintf(timeStamp, "%04d%02d%02d_%02d%02d",
            currentDate.year(), currentDate.month(), currentDate.day(),
            currentTime.hour(), currentTime.minute(), currentTime.second());
    char pFilePath[256] = {0};
    sprintf(pFilePath, "self_k_test_%s.yuy2", timeStamp);
    FILE *pFile = fopen(pFilePath, "wb");
    if (pFile) {
        fseek(pFile, 0, SEEK_SET);
        fwrite(pBuffer, sizeof(unsigned char), pBuffer_size, pFile);
        fclose(pFile);
    } else {
        qDebug() << "saveYUY2 failed";
    }
}

void CSelfcalibrationController::saveRGB(BYTE *pBuffer, int pBuffer_size)
{
    QDate currentDate = QDate::currentDate();
    QTime currentTime = QTime::currentTime();
    char timeStamp[64];
    sprintf(timeStamp, "%04d%02d%02d_%02d%02d",
            currentDate.year(), currentDate.month(), currentDate.day(),
            currentTime.hour(), currentTime.minute(), currentTime.second());
    char pFilePath[256] = {0};
    sprintf(pFilePath, "self_k_test_%s.rgb24", timeStamp);
    FILE *pFile = fopen(pFilePath, "wb");
    if (pFile) {
        fseek(pFile, 0, SEEK_SET);
        fwrite(pBuffer, sizeof(unsigned char), pBuffer_size, pFile);
        fclose(pFile);
    } else {
        qDebug() << "saveRGB failed";
    }
}

void CSelfcalibrationController::saveGray(BYTE *pBuffer, int pBuffer_size)
{
    QDate currentDate = QDate::currentDate();
    QTime currentTime = QTime::currentTime();
    char timeStamp[64];
    sprintf(timeStamp, "%04d%02d%02d_%02d%02d",
            currentDate.year(), currentDate.month(), currentDate.day(),
            currentTime.hour(), currentTime.minute(), currentTime.second());
    char pFilePath[256] = {0};
    sprintf(pFilePath, "self_k_test_%s.gray", timeStamp);
    FILE *pFile = fopen(pFilePath, "wb");
    if (pFile) {
        fseek(pFile, 0, SEEK_SET);
        fwrite(pBuffer, sizeof(unsigned char), pBuffer_size, pFile);
        fclose(pFile);
    } else {
        qDebug() << "saveGray failed";
    }
}

void CSelfcalibrationController::show_calibration_issue(self_calibration::self_calibration_issue issue)
{
    switch (issue)
    {
    case self_calibration::self_calibration_issue::Self_Calibration_Success:
        qDebug()  << "issue event is success" << endl;
        break;
    case self_calibration::self_calibration_issue::Self_Calibration_Fail:
        qDebug()  << "issue event is fail" << endl;
        break;
    case self_calibration::self_calibration_issue::Self_Calibration_ParameterError:
        qDebug()  << "issue event is parameter error" << endl;
        break;
    case self_calibration::self_calibration_issue::Self_Calibration_FrameError:
        qDebug()  << "issue event is frame error" << endl;
        break;
    case self_calibration::self_calibration_issue::Self_Calibration_TareDepthError:
        qDebug()  << "issue event is tare depth error" << endl;
        break;
    case self_calibration::self_calibration_issue::Self_Calibration_PlaneError:
        qDebug()  << "issue event is plane error" << endl;
        break;
    case self_calibration::self_calibration_issue::Self_Calibration_QualityBad:
        qDebug()  << "issue event is quality bad" << endl;
        break;
    case self_calibration::self_calibration_issue::Self_Calibration_NotReady:
        qDebug()  << "issue event is parameter not readt" << endl;
        break;

    default:
        break;
    }
}

void CSelfcalibrationController::getColorImage()
{
    int ret, mCount = 0;
    unsigned char *gColorImgBuf = NULL;
    unsigned char *gColorRGBImgBuf = NULL;
    auto EYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
    auto devSelInfo = m_deviceSelInfoList[0];
    bool color_MJPG = false;

    if (gColorImgBuf == NULL) {
        gColorImgBuf = (unsigned char *)calloc(2 * color_width * color_height, sizeof(unsigned char));
    }

    if (gColorRGBImgBuf == NULL) {
        gColorRGBImgBuf = (unsigned char *)calloc(3 * color_width * color_height, sizeof(unsigned char));
    }

    while (mCount < 1) {
        unsigned long gColorImgSize;
        int gColorSerial;
        ret = APC_GetColorImage(EYSD, devSelInfo, (BYTE *)gColorImgBuf, &gColorImgSize, &gColorSerial);
        if (ret == APC_OK && gColorSerial > 0) {
            mCount++;
            if (!mDumpColorYUY2Frame) {
                    saveYUY2(gColorImgBuf, color_width * color_height * 2);
                    mDumpColorYUY2Frame = true;
            }

            ret = APC_ColorFormat_to_BGR24(EYSD, devSelInfo, gColorRGBImgBuf, gColorImgBuf, gColorImgSize,
                                           color_width, color_height,
                                           color_MJPG ? APCImageType::Value::COLOR_MJPG : APCImageType::Value::COLOR_YUY2);

            int color_size = color_width * color_height * 3;
            if (!mDumpColorRGBFrame) {
                    saveRGB(gColorRGBImgBuf, color_width * color_height * 3);
                    mDumpColorRGBFrame = true;
            }

            RGB2Gray(gColorRGBImgBuf, m_color, color_width, color_height);
            if (!mDumpColorGrayFrame) {
                    saveGray(m_color, color_width * color_height);
                    mDumpColorGrayFrame = true;
            }
        }
    }

    if (gColorImgBuf != NULL) {
        free(gColorImgBuf);
        gColorImgBuf = NULL;
    }

    if (gColorRGBImgBuf != NULL) {
        free(gColorRGBImgBuf);
        gColorRGBImgBuf = NULL;
    }
}

int CSelfcalibrationController::initSelfK()
{
    self_calibration::module_parameters pars;
    self_calibration::calibration_mode mode = self_calibration::calibration_mode::Fix_extrinsic;


    if (!p_rectifyLogData)
        p_rectifyLogData = (eSPCtrl_RectLogData *)malloc(sizeof(eSPCtrl_RectLogData));

    auto EYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
    auto devSelInfo = m_deviceSelInfoList[0];
    int nRet = APC_GetRectifyMatLogData(EYSD, devSelInfo, p_rectifyLogData, res_depth_option_id);
    RectLogData2ModuleParameter(*p_rectifyLogData, pars);


    int kk = p_self_k->init(pars, m_color, mode, 200);
    QString csModeDesc = m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo().csModeDesc;
    QString stringLRDr = "L'+R'+D";
    QString stringLRD = "L+R+D";
    if (QString::compare(csModeDesc, stringLRDr) == 0) {
        p_self_k->set_input_image_type(self_calibration::input_image_type::rectified);
        qDebug() << "Init self calibration input_image_type rectified.\n";
    } else if (QString::compare(csModeDesc, stringLRD) == 0) {
        p_self_k->set_input_image_type(self_calibration::input_image_type::raw);
        qDebug() << "Init self calibration input_image_type raw.\n";
    } else {
        qDebug() << "Init self calibration input_image_type error\n";
        return 0;
    }

    if (!kk) {
        qDebug() << "Init self calibration success.\n";
    } else {
        qDebug() << "Init self calibration fail.\n";
        return 0;
    }

/*
    p_self_k->initflow();
    p_self_k->runflow();

    qDebug() << "show output pars" << endl;
    self_calibration::module_parameters new_pars;
    p_self_k->getCurrentParameters(&new_pars);
*/

    return 0;
}

int CSelfcalibrationController::executeSelfK()
{

    getColorImage();

    self_calibration::parameter_state par_state;
    self_calibration::module_parameters new_pars;
    self_calibration::calibration_mode mode = self_calibration::calibration_mode::Fix_extrinsic;
    self_calibration::self_calibration_issue selfK_issue;

    float now_score=0,target_score = 0;
    p_self_k->get_low_quality_threshold_to_trigger_warning(&target_score);
    qDebug() << "object score is "<<target_score;
    p_self_k->doCalculatingForCurrentHealth(now_score);
    qDebug() << "now score is "<<now_score;
    if (now_score > target_score){
        return 0;
    }

    p_self_k->set_recovery_mode(mode);
    // p_self_k->setControlParameters(self_calibration::calibration_controller_parameters::recovery_mode, &mode);
    selfK_issue = p_self_k->doParameterEstimationForRecovery();
    show_calibration_issue(selfK_issue);
    p_self_k->getStateOfEstimatedPars(&par_state);
    float score;
    p_self_k->getScroeOfEstimatedPars(&score);
    qDebug() << "executeSelfK score:" << score;

    if (par_state == self_calibration::parameter_state::Good) {
        auto EYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
        auto devSelInfo = m_deviceSelInfoList[0];
        p_self_k->getEstimatedParameters(&new_pars);
        eSPCtrl_RectLogData *tmp_rectifyLogData = nullptr;
        tmp_rectifyLogData = (eSPCtrl_RectLogData *)malloc(sizeof(eSPCtrl_RectLogData));
        memset(tmp_rectifyLogData, 0, sizeof(eSPCtrl_RectLogData));
        APC_GetRectifyMatLogData(EYSD, devSelInfo, tmp_rectifyLogData, res_depth_option_id);
        ModuleParameter2RectLogData(new_pars, *tmp_rectifyLogData);
        WriteRectLogData_temp(*tmp_rectifyLogData, EYSD, devSelInfo);
        free(tmp_rectifyLogData);
        p_self_k->updateCurrentParameters();
    }

    return 0;
}


int CSelfcalibrationController::releaseSelfK()
{
    if (p_rectifyLogData)
    {
        free(p_rectifyLogData);
        p_rectifyLogData = nullptr;
    }

    if(m_color) {
        free(m_color);
        m_color = nullptr;
    }

    if(m_depth) {
        free(m_depth);
        m_depth = nullptr;
    }

    return true;
}

void CSelfcalibrationController::disableSelfK()
{

}

int CSelfcalibrationController::RectLogData2ModuleParameter(eSPCtrl_RectLogData &rectifyData, self_calibration::module_parameters &moduleParameter)
{
    moduleParameter.height = rectifyData.InImgHeight;
    moduleParameter.width = rectifyData.InImgWidth / 2;
    moduleParameter.fixed_focus = rectifyData.NewCamMat1[0];

    for (int i = 0; i < 9; i++)
    {
        moduleParameter.left_intrinsic[i] = rectifyData.CamMat1[i];
        moduleParameter.right_intrinsic[i] = rectifyData.CamMat2[i];
        moduleParameter.left_rotation[i] = rectifyData.LRotaMat[i];
        moduleParameter.right_rotation[i] = rectifyData.RRotaMat[i];
        moduleParameter.extrinsic_rotation[i] = rectifyData.RotaMat[i];
    }

    for (int i = 0; i < 8; i++)
    {
        moduleParameter.left_distortion[i] = rectifyData.CamDist1[i];
        moduleParameter.right_distortion[i] = rectifyData.CamDist2[i];
    }

    for (int i = 0; i < 3; i++)
    {
        moduleParameter.extrinsic_translation[i] = rectifyData.TranMat[i];
    }

    for (int i = 0; i < 16; i++)
    {
        moduleParameter.reprojection[i] = rectifyData.ReProjectMat[i];
    }

    for (int i = 0; i < 12; i++)
    {
        moduleParameter.left_projection[i] = rectifyData.NewCamMat1[i];
        moduleParameter.right_projection[i] = rectifyData.NewCamMat2[i];
    }

    return 0;
}

int CSelfcalibrationController::ModuleParameter2RectLogData(self_calibration::module_parameters &moduleParameter, eSPCtrl_RectLogData &rectifyData)
{
    for (int i = 0; i < 9; i++)
    {
        rectifyData.CamMat1[i] = (float)moduleParameter.left_intrinsic[i];
        rectifyData.CamMat2[i] = (float)moduleParameter.right_intrinsic[i];
        rectifyData.LRotaMat[i] = (float)moduleParameter.left_rotation[i];
        rectifyData.RRotaMat[i] = (float)moduleParameter.right_rotation[i];
        rectifyData.RotaMat[i] = (float)moduleParameter.extrinsic_rotation[i];
    }

    for (int i = 0; i < 8; i++)
    {
        rectifyData.CamDist1[i] = (float)moduleParameter.left_distortion[i];
        rectifyData.CamDist2[i] = (float)moduleParameter.right_distortion[i];
    }

    for (int i = 0; i < 3; i++)
    {
        rectifyData.TranMat[i] = (float)moduleParameter.extrinsic_translation[i];
    }

    for (int i = 0; i < 16; i++)
    {
        rectifyData.ReProjectMat[i] = (float)moduleParameter.reprojection[i];
    }

    for (int i = 0; i < 12; i++)
    {
        rectifyData.NewCamMat1[i] = (float)moduleParameter.left_projection[i];
        rectifyData.NewCamMat2[i] = (float)moduleParameter.right_projection[i];
    }

    return 0;
}

self_calibration::self_calibration_issue CSelfcalibrationController::operator_calculate_score()
{
    self_calibration::self_calibration_issue ret = self_calibration::self_calibration_issue::Self_Calibration_Fail;
    if (checkLRDMode() && p_self_k) {
        float score = 0;
        getColorImage();
        ret = p_self_k->doCalculatingForCurrentHealth(score);
    }
    return ret;
}

self_calibration::self_calibration_issue CSelfcalibrationController::operator_do_calibration()
{
    self_calibration::self_calibration_issue ret = self_calibration::self_calibration_issue::Self_Calibration_Fail;
    if (checkLRDMode() && p_self_k) {
        getColorImage();
        ret = p_self_k->doParameterEstimationForRecovery();
    }
    return ret;
}

void CSelfcalibrationController::operator_apply_before_reg()
{
    if (checkLRDMode() && p_self_k) {
        self_calibration::module_parameters new_pars;
        p_self_k->getCurrentParameters(&new_pars);
        eSPCtrl_RectLogData *tmp_rectifyLogData = nullptr;
        tmp_rectifyLogData = (eSPCtrl_RectLogData *)malloc(sizeof(eSPCtrl_RectLogData));
        memset(tmp_rectifyLogData, 0, sizeof(eSPCtrl_RectLogData));
        auto EYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
        auto devSelInfo = m_deviceSelInfoList[0];
        APC_GetRectifyMatLogData(EYSD, devSelInfo, tmp_rectifyLogData, res_depth_option_id);
        ModuleParameter2RectLogData(new_pars, *tmp_rectifyLogData);
        WriteRectLogData_temp(*tmp_rectifyLogData, EYSD, devSelInfo);
        free(tmp_rectifyLogData);
    }
}

void CSelfcalibrationController::operator_apply_after_reg()
{
    if (checkLRDMode() && p_self_k) {
        self_calibration::module_parameters new_pars;
        p_self_k->getEstimatedParameters(&new_pars);
        eSPCtrl_RectLogData *tmp_rectifyLogData = nullptr;
        tmp_rectifyLogData = (eSPCtrl_RectLogData *)malloc(sizeof(eSPCtrl_RectLogData));
        memset(tmp_rectifyLogData, 0, sizeof(eSPCtrl_RectLogData));
        auto EYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
        auto devSelInfo = m_deviceSelInfoList[0];
        APC_GetRectifyMatLogData(EYSD, devSelInfo, tmp_rectifyLogData, res_depth_option_id);
        ModuleParameter2RectLogData(new_pars, *tmp_rectifyLogData);
        WriteRectLogData_temp(*tmp_rectifyLogData, EYSD, devSelInfo);
        free(tmp_rectifyLogData);
    }
}

void CSelfcalibrationController::operator_apply_after_flash()
{
    if (checkLRDMode() && p_self_k) {
        self_calibration::module_parameters new_pars;
        p_self_k->getEstimatedParameters(&new_pars);
        eSPCtrl_RectLogData *tmp_rectifyLogData = nullptr;
        tmp_rectifyLogData = (eSPCtrl_RectLogData *)malloc(sizeof(eSPCtrl_RectLogData));
        memset(tmp_rectifyLogData, 0, sizeof(eSPCtrl_RectLogData));
        auto EYSD = CEYSDDeviceManager::GetInstance()->GetEYSD();
        auto devSelInfo = m_deviceSelInfoList[0];
        APC_GetRectifyMatLogData(EYSD, devSelInfo, tmp_rectifyLogData, res_depth_option_id);
        ModuleParameter2RectLogData(new_pars, *tmp_rectifyLogData);
        WriteRectification(*tmp_rectifyLogData, EYSD, devSelInfo, res_depth_option_id);
        WriteRectifyLogData(*tmp_rectifyLogData, EYSD, devSelInfo, res_depth_option_id);
        free(tmp_rectifyLogData);
    }
}

void CSelfcalibrationController::operator_reset_all()
{
    if (checkLRDMode() && p_self_k) {
        p_self_k->resetCurrentAndEstimatedParametersToDefault();
    }
}

