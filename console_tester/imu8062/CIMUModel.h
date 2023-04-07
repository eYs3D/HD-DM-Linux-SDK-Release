#ifndef CIMUMODEL_H
#define CIMUMODEL_H

#include "IMUData.h"
#include <vector>
#include "../eSPDI/eSPDI_def.h"
#include "../eSPDI/eSPDI.h"

#include "../hidapi.h"
#include <functional>
#include <thread>
#include <cstring>

#ifndef RETRY_APC_API
#define RETRY_COUNT (5)
#define RETRY_APC_API(ret, func) \
    do{ \
        int retryCount = RETRY_COUNT; \
        while (retryCount > 0){ \
            ret = func; \
            if (APC_OK == ret) break; \
            --retryCount; \
        } \
    }while (false) \

#endif

class CIMUModel
{
private:

    struct SetFeatureDATA_Item {
        const char* pData;
        int nDataLength;
    };

    static unsigned char GenerateIMUNumber();
    bool isCallbackStart;
    std::thread mCallbackLooper;

public:
    CIMUModel(CIMUModel &&) = default;
    using Callback = std::function<bool(const IMUData* data)>;
    inline bool isCallbackThreadStart() { return isCallbackStart; }
    CIMUModel::Callback mCallback;
    IMUData mCallbackData;
    void startCallbackThread(Callback cb) {
        if (!isCallbackThreadStart()) {
            mCallback = std::move(cb);
            mCallbackLooper = std::thread( [&] {
                fprintf(stderr, "Thread start++ \n");
                isCallbackStart = true;
                unsigned char imuRawData[256];
                while (isCallbackThreadStart()) {
                    memset(imuRawData, 0x0, sizeof(imuRawData) / sizeof(imuRawData[0]));

                    if(!m_pHandle) return APC_NullPtr;

                    int ret = hid_read(m_pHandle, imuRawData, sizeof(imuRawData));
                    if (ret > 0 && IMU_9_AXIS == GetType()){
                        mCallbackData.parsePacket_Quaternion(imuRawData);
                        mCallback(&mCallbackData);
                    }
                }
                fprintf(stderr, "Thread start-- \n");
                return 0;
            });
        } else {
            fprintf(stderr, "EnableDataCallback opened nop...\n");
        }
    }

    void stopCallbackThreadAndJoin(){
        if (isCallbackThreadStart()) {
            isCallbackStart = false;
            mCallbackLooper.join();
        }
    }

    const char GET_MODULE_NAME_0[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_MODULE_NAME_1[8] = { 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_MODULE_NAME_2[8] = { 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_MODULE_NAME_3[8] = { 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const char GET_FW_VERSION_0[8] = { 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_FW_VERSION_1[8] = { 0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_FW_VERSION_2[8] = { 0x00, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_FW_VERSION_3[8] = { 0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_FW_VERSION_4[8] = { 0x00, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_FW_VERSION_5[8] = { 0x00, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_FW_VERSION_6[8] = { 0x00, 0x0A, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_FW_VERSION_7[8] = { 0x00, 0x0B, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const char READ_OUTPUT_STATUS[8] = { 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char DISABLE_OUTPUT[8] = { 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char ENABLE_OUTPUT[8] = { 0x00, 0x11, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };

    const char READ_OUTPUT_FORMAT[8] = { 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const char SET_OUTPUT_FORMAT_1[8] = { 0x00, 0x12, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };
    const char SET_OUTPUT_FORMAT_2[8] = { 0x00, 0x12, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00 };
    const char SET_OUTPUT_FORMAT_3[8] = { 0x00, 0x12, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
    const char SET_OUTPUT_FORMAT_4[8] = { 0x00, 0x12, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00 };
    const char SET_OUTPUT_FORMAT_5[8] = { 0x00, 0x12, 0x02, 0x05, 0x00, 0x00, 0x00, 0x00 };

    const char CHECK_CALIBRATING_STATUS[8] = { 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char START_CALIBRATION[8] = { 0x00, 0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char READ_CALIBRATED[8] = { 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const char GET_SERIAL_NUMBER_0[8] = { 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_SERIAL_NUMBER_1[8] = { 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_SERIAL_NUMBER_2[8] = { 0x00, 0x14, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_SERIAL_NUMBER_3[8] = { 0x00, 0x14, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const char GET_SERIAL_NUMBER_4[8] = { 0x00, 0x14, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const char CHECK_FLASH_WRITING_STATUS[8] = { 0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const char START_WRITE_FLASH[8] = { 0x00, 0x1B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

    static unsigned char GetIMUNumber(std::string serialNumber);
    static const uint8_t RW_REG_STATUS_CMD_FINISHED = 0x00;
    static const uint8_t RW_REG_STATUS_CMD_PROCESSING = 0x01;
    static const uint8_t RW_REG_STATUS_CMD_FAILED = 0xA5;

    enum TYPE{
        IMU_6_AXIS,
        IMU_9_AXIS,
        IMU_UNKNOWN
    };

    enum DATA_FORMAT{
        RAW_DATA_WITHOUT_OFFSET = 1,
        RAW_DATA_WITH_OFFSET,
        OFFSET_DATA,
        DMP_DATA_WITHOT_OFFSET,
        DMP_DATA_WITH_OFFSET
    };

    struct INFO {
        unsigned short nVID;
        unsigned short nPID;
        TYPE axis;
    };

    CIMUModel(INFO info, void* pCameraHandle);
    ~CIMUModel();

    bool IsValid(){ return m_pHandle != nullptr; }
    int Init(INFO info);
    int ReadDataOutputFormat();

    void EnableDataOutput(bool bIsEnbale);
    void EnableDataCallback(CIMUModel::Callback cb);
    void DisableDataCallback();
    int SelectDataFormat(DATA_FORMAT format);
    DATA_FORMAT GetDataFormat(){ return m_nCurrentIMUFormat; }

    int ReadIMUData(IMUData &imuData, bool bSync = true);
    int GetIMUDataOutputByte(DATA_FORMAT format);
    std::vector<DATA_FORMAT> GetSupportDataFormat();

    void CheckCalibratingStatus(char *pCalibratingStatus);
    void StartCalibration();
    void ReadCalibrated(char *pCalibrated);

    std::string GetFWVersion();
    std::string GetModuleName();
    std::string GetStatus();
    TYPE GetType(){ return m_imuType; }
    std::string GetCameraSerialNumber(void *cameraHandle, DEVSELINFO devSelInfo);

    std::string GetSerialNumber();
    void SetSerialNumber(const std::string& sSerialNumber);

    int GetModuleID();
    void SetModuleID(unsigned char nID);

    void SetLogFile(FILE *pLogFile){ m_pLogFile = pLogFile; }

    bool IsSyncWithCamera(){ return m_bIsSyncWithCamera; }

    /**
     *
     * @param bank Refer to ICM-20948 data sheet
     * @param address Refer to ICM-20948 data sheet
     * @param value Refer to ICM-20948 data sheet
     * @return true for successfully read.
     */
    bool ReadRegister(uint8_t bank, uint8_t address, uint8_t *value);
    bool WriteRegister(uint8_t bank, uint8_t address, uint8_t value);
private:
    void GetFeatureReport(char* pData, size_t data_lenght);
    void SendFeatureReport(const char* pData, size_t data_lenght);

private:

    void *m_pCameraHandle;
    DEVSELINFO m_devSelInfo { .index = 0 };

    hid_device *m_pHandle;
    TYPE m_imuType;

    DATA_FORMAT m_nCurrentIMUFormat;
    FILE *m_pLogFile;

    bool m_bIsSyncWithCamera;
    unsigned short m_nSyncIndex;
    int mModuleID;
};

#endif // CIMUMODEL_H
