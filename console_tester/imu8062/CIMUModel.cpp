#include "CIMUModel.h"
#include "CIMUDeviceManager.h"

#include <unistd.h>
#include <vector>
#include <map>
#include <climits>
#include <cstring>

unsigned char CIMUModel::GenerateIMUNumber()
{
    static unsigned char nIMUNumberCounter = 1;
    nIMUNumberCounter = (nIMUNumberCounter + 1) % UCHAR_MAX;
    return nIMUNumberCounter;
}

unsigned char CIMUModel::GetIMUNumber(std::string serialNumber)
{
    static std::map<std::string, unsigned char> imuNumberMap;

    if (0 == imuNumberMap.count(serialNumber)){
        imuNumberMap[serialNumber] = GenerateIMUNumber();
    }
    return imuNumberMap[serialNumber];
}

CIMUModel::CIMUModel(INFO info, void* cameraHandle):
m_imuType(info.axis),
m_pHandle(nullptr),
m_nCurrentIMUFormat(RAW_DATA_WITHOUT_OFFSET),
m_pLogFile(nullptr),
m_pCameraHandle(cameraHandle),
m_bIsSyncWithCamera(false),
m_nSyncIndex(0)
{
    if (!m_pCameraHandle) {
        fprintf(stderr, "Check ctor not provide camera handle\n");
        return;
    }
    Init(info);
}

CIMUModel::~CIMUModel()
{
    stopCallbackThreadAndJoin();
}

std::string CIMUModel::GetCameraSerialNumber(void *cameraHandle, DEVSELINFO devSelInfo)
{
    unsigned char pSerialNumber[256] = {0};
    int nSerialNumberLength;
    int ret = APC_Init_Fail;
    std::string sSerialNumber;

    RETRY_APC_API(ret, APC_GetSerialNumber(cameraHandle, &devSelInfo, pSerialNumber, 256, &nSerialNumberLength));
    if (APC_OK == ret) {
        if (nSerialNumberLength<0)
            nSerialNumberLength = 0;
        sSerialNumber.resize( nSerialNumberLength / 2 );
        for (int i = 0 ; i < nSerialNumberLength / 2 ; ++i)
            sSerialNumber[i] = pSerialNumber[i * 2 + 1] * 256 + pSerialNumber[i * 2];
    }

    return std::move(sSerialNumber);
}

int CIMUModel::Init(INFO info)
{
    if (m_pHandle){
        m_pHandle = nullptr;
    }

    m_bIsSyncWithCamera = false;

    std::vector<hid_device *> hidDeviceList;

    auto EnumerateIMU = [&](std::vector<hid_device *> &deviceList) -> int {
        deviceList = CIMUDeviceManager::GetInstance()->GetDeviceList(info.nVID, info.nPID);
        fprintf(stderr, "IMU EnumerateIMU size: %zu \n", deviceList.size());
        return deviceList.size();
    };

    auto IsIMUConntectdWithCamera = [&](hid_device *device) -> bool{

        m_pHandle = device;

        bool IsConnected = false;

        switch (m_imuType){
            case IMU_6_AXIS:
            {
                std::string sFWSerialNumber = GetCameraSerialNumber(m_pCameraHandle, m_devSelInfo);
                std::string sIMUSerialNumber = GetSerialNumber();
                IsConnected = sFWSerialNumber == sIMUSerialNumber;
                break;
            }
            case IMU_9_AXIS:
            {
                mModuleID = GetModuleID();
                if (0 == mModuleID) break;
                IMUData data;
                hid_set_nonblocking(m_pHandle, true);
                ReadIMUData(data, false);
                hid_set_nonblocking(m_pHandle, false);
                IsConnected = (data._moduleID == mModuleID);
                break;
            }
            default: break;
        }

        m_pHandle = nullptr;

        return IsConnected;
    };

    auto TryConnectIMUWithCamera = [&](){

        if ((size_t)m_nSyncIndex >= hidDeviceList.size()){
            m_nSyncIndex = m_nSyncIndex % hidDeviceList.size();
        }

        m_pHandle = hidDeviceList[m_nSyncIndex];
        if (IMU_6_AXIS == m_imuType) {
            std::string sFWSerialNumber = GetCameraSerialNumber(m_pCameraHandle, m_devSelInfo);
            SetSerialNumber(sFWSerialNumber);
            m_bIsSyncWithCamera = true;
        }
        return;
    };

    auto InitializeIMUWithCamera = [&](){

        for (hid_device *device : hidDeviceList){
            if (IsIMUConntectdWithCamera(device)) {
                m_pHandle = device;
                m_bIsSyncWithCamera = true;
                break;
            }
        }

        if (!m_pHandle){
            TryConnectIMUWithCamera();
        }
    };

    if(IMU_UNKNOWN == m_imuType){
        return APC_NotSupport;
    }

    if (IMU_9_AXIS == m_imuType) {
        SetModuleID((unsigned char)GenerateIMUNumber());
    }

    int nDeviceCount = EnumerateIMU(hidDeviceList);
    if (0 == nDeviceCount) return APC_NoDevice;

    InitializeIMUWithCamera();

    ReadDataOutputFormat();

    return APC_OK;
}

int CIMUModel::ReadDataOutputFormat()
{
    SetFeatureDATA_Item setFeatureData = { &READ_OUTPUT_FORMAT[0], (sizeof(READ_OUTPUT_FORMAT) / sizeof(READ_OUTPUT_FORMAT[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    //memcpy(pImuDataOutoutFormat, &status[0], setFeatureData.nDataLength);
    m_nCurrentIMUFormat = status[0] != 0 ? (DATA_FORMAT)status[0] : RAW_DATA_WITHOUT_OFFSET;
    return (int) m_nCurrentIMUFormat;
}

void CIMUModel::EnableDataOutput(bool bIsEnbale)
{
    SetFeatureDATA_Item setFeatureData;

    if (bIsEnbale) {
        setFeatureData = { &ENABLE_OUTPUT[0], (sizeof(ENABLE_OUTPUT) / sizeof(ENABLE_OUTPUT[0])) };
    }
    else {
        setFeatureData = { &DISABLE_OUTPUT[0], (sizeof(DISABLE_OUTPUT) / sizeof(DISABLE_OUTPUT[0])) };
    }

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
}

int CIMUModel::SelectDataFormat(DATA_FORMAT format)
{
    SetFeatureDATA_Item setFeatureData;

    switch (format)
    {
    case RAW_DATA_WITHOUT_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_1[0], (sizeof(SET_OUTPUT_FORMAT_1) / sizeof(SET_OUTPUT_FORMAT_1[0])) };
        break;

    case RAW_DATA_WITH_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_2[0], (sizeof(SET_OUTPUT_FORMAT_2) / sizeof(SET_OUTPUT_FORMAT_2[0])) };
        break;

    case OFFSET_DATA:
        setFeatureData = { &SET_OUTPUT_FORMAT_3[0], (sizeof(SET_OUTPUT_FORMAT_3) / sizeof(SET_OUTPUT_FORMAT_3[0])) };
        break;

    case DMP_DATA_WITHOT_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_4[0], (sizeof(SET_OUTPUT_FORMAT_4) / sizeof(SET_OUTPUT_FORMAT_4[0])) };
        break;

    case DMP_DATA_WITH_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_5[0], (sizeof(SET_OUTPUT_FORMAT_5) / sizeof(SET_OUTPUT_FORMAT_5[0])) };
        break;

    default:
        return APC_NotSupport;
    }

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    m_nCurrentIMUFormat = format;
    return APC_OK;
}

void CIMUModel::GetFeatureReport(char* pData, size_t data_lenght)
{
    if (m_pHandle) {
        unsigned char *pBuf = nullptr;
        pBuf = (unsigned char *)calloc(data_lenght + 1, sizeof(unsigned char));
        pBuf[0] = { 0x0 };
        hid_get_feature_report(m_pHandle, pBuf, data_lenght + 1);
        memcpy(pData, pBuf + 1, data_lenght);
        free( pBuf );
    }
}

void CIMUModel::SendFeatureReport(const char* pData, size_t data_lenght)
{
    if (m_pHandle) {
        unsigned char *pBuf = nullptr;
        pBuf = (unsigned char *)calloc(data_lenght + 1, sizeof(unsigned char));
        pBuf[0] = { 0x0 };
        memcpy(pBuf + 1, pData, data_lenght);
        hid_send_feature_report(m_pHandle, pBuf, data_lenght + 1);
        free( pBuf );
    }
}

std::string CIMUModel::GetFWVersion()
{
    SetFeatureDATA_Item setFeatureData[8] = {
        { &GET_FW_VERSION_0[0], (sizeof(GET_FW_VERSION_0) / sizeof(GET_FW_VERSION_0[0])) },
        { &GET_FW_VERSION_1[0], (sizeof(GET_FW_VERSION_1) / sizeof(GET_FW_VERSION_1[0])) },
        { &GET_FW_VERSION_2[0], (sizeof(GET_FW_VERSION_2) / sizeof(GET_FW_VERSION_2[0])) },
        { &GET_FW_VERSION_3[0], (sizeof(GET_FW_VERSION_3) / sizeof(GET_FW_VERSION_3[0])) },
        { &GET_FW_VERSION_4[0], (sizeof(GET_FW_VERSION_4) / sizeof(GET_FW_VERSION_4[0])) },
        { &GET_FW_VERSION_5[0], (sizeof(GET_FW_VERSION_5) / sizeof(GET_FW_VERSION_5[0])) },
        { &GET_FW_VERSION_6[0], (sizeof(GET_FW_VERSION_6) / sizeof(GET_FW_VERSION_6[0])) },
        { &GET_FW_VERSION_7[0], (sizeof(GET_FW_VERSION_7) / sizeof(GET_FW_VERSION_7[0])) }
    };

    char fwVersion[256] = { 0 };
    char* pBuf = &fwVersion[0];
    WORD count = 0;

    for (int i = 0; i < 8; i++) {
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
        GetFeatureReport(pBuf, 8);
        pBuf += 8;
        count += 8;
    }

    std::string sFWVersion(fwVersion);
    fprintf(stderr, "CIMUModel::GetFWVersion %s \n",sFWVersion.c_str());
    return std::move(sFWVersion);
}

std::string CIMUModel::GetModuleName()
{
    SetFeatureDATA_Item setFeatureData[4] = {
        { &GET_MODULE_NAME_0[0], (sizeof(GET_MODULE_NAME_0) / sizeof(GET_MODULE_NAME_0[0])) },
        { &GET_MODULE_NAME_1[0], (sizeof(GET_MODULE_NAME_1) / sizeof(GET_MODULE_NAME_1[0])) },
        { &GET_MODULE_NAME_2[0], (sizeof(GET_MODULE_NAME_2) / sizeof(GET_MODULE_NAME_2[0])) },
        { &GET_MODULE_NAME_3[0], (sizeof(GET_MODULE_NAME_3) / sizeof(GET_MODULE_NAME_3[0])) }
    };

    char moduleName[256] = { 0 };
    WORD count = 0;

    for (int i = 0; i < 4; i++) {
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
        GetFeatureReport(&moduleName[count], 8);
        count += 8;
    }

    std::string sModuleName(moduleName);
    return std::move(sModuleName);
}

std::string CIMUModel::GetStatus()
{
    SetFeatureDATA_Item setFeatureData = { &READ_OUTPUT_STATUS[0], (sizeof(READ_OUTPUT_STATUS) / sizeof(READ_OUTPUT_STATUS[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    return (0 == status[0]) ? "Disable" : "Enable";
}

std::string CIMUModel::GetSerialNumber()
{
    if (IMU_6_AXIS != m_imuType) return "";

    SetFeatureDATA_Item setFeatureData[5] = {
        { &GET_SERIAL_NUMBER_0[0], (sizeof(GET_SERIAL_NUMBER_0) / sizeof(GET_SERIAL_NUMBER_0[0])) },
        { &GET_SERIAL_NUMBER_1[0], (sizeof(GET_SERIAL_NUMBER_1) / sizeof(GET_SERIAL_NUMBER_1[0])) },
        { &GET_SERIAL_NUMBER_2[0], (sizeof(GET_SERIAL_NUMBER_2) / sizeof(GET_SERIAL_NUMBER_2[0])) },
        { &GET_SERIAL_NUMBER_3[0], (sizeof(GET_SERIAL_NUMBER_3) / sizeof(GET_SERIAL_NUMBER_3[0])) },
        { &GET_SERIAL_NUMBER_4[0], (sizeof(GET_SERIAL_NUMBER_4) / sizeof(GET_SERIAL_NUMBER_4[0])) }
    };

    char serialNumber[256] = { 0 };
    WORD count = 0;
    std::string sSerialNumber;
    for (int i = 0; i < 5; i++) {
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
        GetFeatureReport(&serialNumber[count], 8);
        int j = (0 == i) ? 2 : 0;
        for (; j < 8 ; j+=2){
            char unicode = (serialNumber[count + j] | (serialNumber[count + j + 1]) << 8);
            if ('\0' == unicode) break;
            sSerialNumber += unicode;
        }

        count += 8;
    }

    fprintf(stderr, "CIMUModel::GetSerialNumber %s\n", sSerialNumber.c_str());
    return std::move(sSerialNumber);
}

void CIMUModel::SetSerialNumber(const std::string& sSerialNumber)
{
    if (IMU_6_AXIS != m_imuType) return;

    char SET_SERIAL_NUMBER[6][8] = { {0x00, 0x15, 0x24, 0x03, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }};


    SetFeatureDATA_Item setFeatureData[6] = {
        { &SET_SERIAL_NUMBER[0][0], (sizeof(SET_SERIAL_NUMBER[0]) / sizeof(SET_SERIAL_NUMBER[0][0])) },
        { &SET_SERIAL_NUMBER[1][0], (sizeof(SET_SERIAL_NUMBER[1]) / sizeof(SET_SERIAL_NUMBER[1][0])) },
        { &SET_SERIAL_NUMBER[2][0], (sizeof(SET_SERIAL_NUMBER[2]) / sizeof(SET_SERIAL_NUMBER[2][0])) },
        { &SET_SERIAL_NUMBER[3][0], (sizeof(SET_SERIAL_NUMBER[3]) / sizeof(SET_SERIAL_NUMBER[3][0])) },
        { &SET_SERIAL_NUMBER[4][0], (sizeof(SET_SERIAL_NUMBER[4]) / sizeof(SET_SERIAL_NUMBER[4][0])) },
        { &SET_SERIAL_NUMBER[5][0], (sizeof(SET_SERIAL_NUMBER[5]) / sizeof(SET_SERIAL_NUMBER[5][0])) }
    };

    const char *pSerialNumber = sSerialNumber.data();
    for (int i = 0; i < 6; i++) {
        int j = (0 == i) ? 4 : 2;
        for(; j < 8; j += 2){
            if (*pSerialNumber != '\0') {
                unsigned short unicode = *pSerialNumber;
                SET_SERIAL_NUMBER[i][j] = unicode & 0xff;
                SET_SERIAL_NUMBER[i][j + 1] = (unicode & 0xff00) >> 8;
                pSerialNumber++;
            }
        }
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
    }

    SetFeatureDATA_Item checkFlashWritingStatue = { &CHECK_FLASH_WRITING_STATUS[0], (sizeof(CHECK_FLASH_WRITING_STATUS) / sizeof(CHECK_FLASH_WRITING_STATUS[0])) };
    SendFeatureReport(checkFlashWritingStatue.pData, checkFlashWritingStatue.nDataLength);
    char status[8] = { 0 };
    GetFeatureReport(&status[0], checkFlashWritingStatue.nDataLength);

    char nRetryCount = 10;
    while (0 != status[0] && --nRetryCount >= 0) {
        usleep(100 * 1000);
        SendFeatureReport(checkFlashWritingStatue.pData, checkFlashWritingStatue.nDataLength);
        GetFeatureReport(&status[0], checkFlashWritingStatue.nDataLength);
    }

    SetFeatureDATA_Item writeSerialToFlash = { &START_WRITE_FLASH[0], (sizeof(START_WRITE_FLASH) / sizeof(START_WRITE_FLASH[0])) };
    SendFeatureReport(writeSerialToFlash.pData, writeSerialToFlash.nDataLength);
}

int CIMUModel::GetModuleID()
{
    if (IMU_9_AXIS != m_imuType || !m_pCameraHandle) return -1;

    unsigned short value;

    int ret = APC_GetHWRegister(m_pCameraHandle,
                                &m_devSelInfo,
                                    0xf306, &value,
                                    FG_Address_2Byte | FG_Value_1Byte);

    if (APC_OK != ret) return -1;

    return value;
}

void CIMUModel::SetModuleID(unsigned char nID)
{
    if (IMU_9_AXIS != m_imuType) return;

    APC_SetHWRegister(m_pCameraHandle, &m_devSelInfo,0xf306, nID, FG_Address_2Byte | FG_Value_1Byte);
}

int CIMUModel::ReadIMUData(IMUData &imuData, bool bSync)
{
    if(!m_pHandle){
        return APC_NullPtr;
    }

    unsigned char imuRawData[256] = {0};
    int ret = hid_read(m_pHandle, imuRawData, sizeof(imuRawData));
    if (ret > 0){
        int nIMUDataByte = GetIMUDataOutputByte(m_nCurrentIMUFormat);

        if(m_pLogFile){
            for (int i = 0; i < nIMUDataByte; i++) {
                fprintf(m_pLogFile, "%02x ", imuRawData[i]);
            }
            fprintf(m_pLogFile, "\n");
            fflush(m_pLogFile);
        }
    }

    return ret;
}

int CIMUModel::GetIMUDataOutputByte(DATA_FORMAT format)
{
    switch(format){
        case RAW_DATA_WITHOUT_OFFSET:
        case RAW_DATA_WITH_OFFSET:
        case OFFSET_DATA:
            return 27;
        case DMP_DATA_WITHOT_OFFSET:
        case DMP_DATA_WITH_OFFSET:
            return 58;
        default: return 0;
    }
}

std::vector<CIMUModel::DATA_FORMAT> CIMUModel::GetSupportDataFormat()
{
    switch (m_imuType){
        case IMU_6_AXIS:
        return {
            RAW_DATA_WITHOUT_OFFSET,
            RAW_DATA_WITH_OFFSET,
            OFFSET_DATA
        };
        case IMU_9_AXIS:
        default:
        return {};
    }
}

void CIMUModel::CheckCalibratingStatus(char *pCalibratingStatus)
{
    SetFeatureDATA_Item setFeatureData = { &CHECK_CALIBRATING_STATUS[0], (sizeof(CHECK_CALIBRATING_STATUS) / sizeof(CHECK_CALIBRATING_STATUS[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    *pCalibratingStatus = status[0];
}

void CIMUModel::StartCalibration()
{
    SetFeatureDATA_Item setFeatureData = { &START_CALIBRATION[0], (sizeof(START_CALIBRATION) / sizeof(START_CALIBRATION[0])) };
    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
}

void CIMUModel::ReadCalibrated(char *pCalibrated)
{
    SetFeatureDATA_Item setFeatureData = { &READ_CALIBRATED[0], (sizeof(READ_CALIBRATED) / sizeof(READ_CALIBRATED[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    *pCalibrated = status[0];
}

bool CIMUModel::ReadRegister(uint8_t bank, uint8_t address, uint8_t* value)
{
    if (!value) {
        fprintf(stderr, "ReadRegister give a null pointer.\n ");
        return -1;
    }

    char READ_REGISTER[8] = { 0x01, 0x20, 0x01, static_cast<char>(bank), static_cast<char>(address), 0x00, 0x00, 0x00 };

    SetFeatureDATA_Item setFeatureData = { &READ_REGISTER[0], sizeof(READ_REGISTER) / sizeof(READ_REGISTER[0]) };
    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);

    char status[8];
    constexpr short kRetryMax = 10;
    short tryCount = 0;
    while (tryCount < kRetryMax) {
        memset(status, 0x0, 8);
        GetFeatureReport(&status[0], sizeof(status) / sizeof(status[0]));
        switch (status[1]) {
            case RW_REG_STATUS_CMD_FINISHED:
                *value = status[0];
                return status[1] == RW_REG_STATUS_CMD_FINISHED;
            case RW_REG_STATUS_CMD_PROCESSING:
                tryCount++;
                continue;
            case RW_REG_STATUS_CMD_FAILED:
            default:
                setFeatureData = { &READ_REGISTER[0], sizeof(READ_REGISTER) / sizeof(READ_REGISTER[0]) };
                SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
                tryCount++;
                continue;
        }
    }

    return status[1] == RW_REG_STATUS_CMD_FINISHED;
}

bool CIMUModel::WriteRegister(uint8_t bank, uint8_t address, uint8_t value)
{
    char WRITE_REGISTER[8] = { 0x01, 0x21, 0x01, static_cast<char>(bank), static_cast<char>(address),
                               static_cast<char>(value), 0x00, 0x00 };

    SetFeatureDATA_Item setFeatureData = {&WRITE_REGISTER[0], sizeof(WRITE_REGISTER) / sizeof(WRITE_REGISTER[0]) };
    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);

    char status[8];
    constexpr short kRetryMax = 10;
    short tryCount = 0;
    while (tryCount < kRetryMax) {
        memset(status, 0x0, 8);
        GetFeatureReport(&status[0], sizeof(status) / sizeof(status[0]));

        switch (status[1]) {
            case RW_REG_STATUS_CMD_FINISHED:
                return status[1] == RW_REG_STATUS_CMD_FINISHED && status[0] == 0x1;
            case RW_REG_STATUS_CMD_PROCESSING:
                tryCount++;
                continue;
            case RW_REG_STATUS_CMD_FAILED:
            default:
                setFeatureData = { &WRITE_REGISTER[0], sizeof(WRITE_REGISTER) / sizeof(WRITE_REGISTER[0]) };
                SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
                tryCount++;
                continue;
        }
    }

    return status[1] == RW_REG_STATUS_CMD_FINISHED;
}

void CIMUModel::EnableDataCallback(CIMUModel::Callback cb) {
    std::string imuStatus = GetStatus();
    EnableDataOutput(true);
    fprintf(stderr, "EnableDataCallback++ %s\n", imuStatus.c_str());
    startCallbackThread(std::move(cb));
}

void CIMUModel::DisableDataCallback() {
    stopCallbackThreadAndJoin();
    EnableDataOutput(false);
}

CIMUModel::TimeSync CIMUModel::ReadTimeSync() {
    unsigned char status[8] = { 0 };
    SendFeatureReport(&Read_Time_Sync[0], sizeof(Read_Time_Sync) / sizeof(Read_Time_Sync[0]));
    GetFeatureReport((char*) &status[0], sizeof(status) / sizeof(status[0]));
    return {
        .sn = status[0],
        .time1 = status[1],
        .time2 = status[2],
        .time3 = status[3],
        .time4 = status[4],
        .diff1 = status[5],
        .diff2 = status[6],
        .diff3 = status[7]
    };
}

void CIMUModel::SetTimeSync(uint8_t sn) {
    unsigned char status[8] = { 0 };
    const unsigned char Set_Time_Sync[8] = { 0x00, 0x1F, 0x01, sn, 0x00, 0x00, 0x00, 0x00};
    SendFeatureReport((const char*) &Set_Time_Sync[0], sizeof(Set_Time_Sync) / sizeof(Set_Time_Sync[0]));
}

CIMUModel::RTC CIMUModel::ReadRTC() {
    unsigned char status[8] = { 0 };
    SendFeatureReport(&READ_RTC[0], sizeof(READ_RTC) / sizeof(READ_RTC[0]));
    GetFeatureReport((char*) &status[0], sizeof(status) / sizeof(status[0]));
    return {
        .hour = status[0],
        .min = status[1],
        .sec = status[2],
        .subSecond = (uint16_t) (status[3] + (status[4] << 8))
    };
}

void CIMUModel::WriteRTC(CIMUModel::RTC rtc) {
    unsigned char status[8] = { 0 };
    const unsigned char WRITE_RTC[8] = { 0x00, 0x1D, 0x01, rtc.hour, rtc.min, rtc.sec, 0x00, 0x00};
    SendFeatureReport((const char*) &WRITE_RTC[0], sizeof(WRITE_RTC) / sizeof(WRITE_RTC[0]));
    GetFeatureReport((char*) &status[0], sizeof(status) / sizeof(status[0]));
}

uint8_t CIMUModel::ReadAccFS() {
    unsigned char status[8] = { 0 };
    SendFeatureReport(&Read_ACC_FS[0], sizeof(Read_ACC_FS) / sizeof(Read_ACC_FS[0]));
    GetFeatureReport((char*) &status[0], sizeof(status) / sizeof(status[0]));
    return status[0];
}

void CIMUModel::WriteAccFS(uint8_t value) {
    unsigned char status[8] = { 0 };
    const unsigned char WRITE_ACC_FW[8] = { 0x01, 0x23, 0x01, value, 0x0, 0x0, 0x0, 0x0 };
    SendFeatureReport((const char*) &WRITE_ACC_FW[0], sizeof(WRITE_ACC_FW) / sizeof(WRITE_ACC_FW[0]));
    GetFeatureReport((char*) &status[0], sizeof(status) / sizeof(status[0]));
}

uint8_t CIMUModel::ReadGyroFS() {
    unsigned char status[8] = { 0 };
    SendFeatureReport(&Read_GYRO_FS[0], sizeof(Read_GYRO_FS) / sizeof(Read_GYRO_FS[0]));
    GetFeatureReport((char*) &status[0], sizeof(status) / sizeof(status[0]));
    return status[0];
}

void CIMUModel::WriteGyroFS(uint8_t value) {
    unsigned char status[8] = { 0 };
    const unsigned char WRITE_GYRO_FW[8] = { 0x01, 0x25, 0x01, value, 0x0, 0x0, 0x0, 0x0 };
    SendFeatureReport((const char*) &WRITE_GYRO_FW[0], sizeof(WRITE_GYRO_FW) / sizeof(WRITE_GYRO_FW[0]));
    GetFeatureReport((char*) &status[0], sizeof(status) / sizeof(status[0]));
}

void CIMUModel::RebootBootloader() {
    SendFeatureReport((const char*) &Reboot_Bootloader[0], sizeof(Reboot_Bootloader) / sizeof(Reboot_Bootloader[0]));
}