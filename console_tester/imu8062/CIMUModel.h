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
    struct RTC {
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint16_t subSecond;
    };

    struct TimeSync {
        uint8_t sn;
        uint8_t time1;
        uint8_t time2;
        uint8_t time3;
        uint8_t time4;
        uint8_t diff1;
        uint8_t diff2;
        uint8_t diff3;
    };

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
                    if (ret >= 21 /* Hard-code here due to IVY HID document not locked down yet */){
                        fprintf(stderr, "Raw hex Data++");
                        for (int i = 0; i < 21; ++i) {
                            fprintf(stderr, "%02x ", imuRawData[i]);
                        }
                        fprintf(stderr,"\n--\n");
                        mCallbackData.parsePacket_STM_IMU(imuRawData);
                        mCallback(&mCallbackData);
                    } else if (ret > 0 && IMU_9_AXIS == GetType()) {
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
    const char READ_RTC[8] = { 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const char Read_ACC_FS[8] = { 0x01, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const char Read_GYRO_FS[8] = { 0x01, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const char Reboot_Bootloader[8] = { 0x00, 0x1E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    const char Read_Time_Sync[8] = { 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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

    /**
     * Only could be used in the YX8083 device.
     * @return RRC/SDT 8byte
     */
    TimeSync ReadTimeSync();

    /**
     * Only could be used in the YX8083 device. Set sn
     * @return
     */
    void SetTimeSync(uint8_t sn);


    /**
     * Only could be used in the YX8083 device.
     * @return RTC
     */
    RTC ReadRTC();

    /**
     * Only could be used in the YX8083 device. Set hour, min, and sec.
     * @return
     */
    void WriteRTC(RTC rtc);

    /**
     * Only could be used in the YX8083 device.
     * @return Read IMU Accel Full Scale Byte 0.  Range:[0, 3] (0=2G, 1=4G, 2=8G, 3=16G)
     */
    uint8_t ReadAccFS();

    /**
     *
     * @param IMU Accel Full Scale Byte 0.  Range:[0, 3] (0=2G, 1=4G, 2=8G, 3=16G)
     */
    void WriteAccFS(uint8_t value);

     /**
     * Only could be used in the YX8083 device.
     * @return Read IMU Gyro Full Scale Byte 0 [0, 5] (0=125dps, 1=250dps, 2=500dps, 3=1000dps, 4=2000dps, 5=4000dps)
     */
    uint8_t ReadGyroFS();

    /**
     * @param value
     * IMU Gyro Full Scale Byte 0 [0, 5] (0=125dps, 1=250dps, 2=500dps, 3=1000dps, 4=2000dps, 5=4000dps)
     */
    void WriteGyroFS(uint8_t value);

    /**
     * Allow user enter bootloader mode without short the JTAG pins on PCB board.
     */
    void RebootBootloader();

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
