#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <vector>
#include <math.h>
#include <thread>
#include <cstring>
#include <linux/input.h>
#include <sys/stat.h>
#include <glob.h>
#include <fstream>
#include <cstdio>

#include "PlyWriter.h"
#include "eSPDI.h"

#include "ColorPaletteGenerator.h"
#include "RegisterSettings.h"
#include "hidapi.h"
#include "imu8062/CIMUModel.h"
#include "json/nlohmann/json.hpp"
#include "meta/MetaDataPayloadParser.h"
#include "meta/JsonFileSerializer.h"

#define CT_DEBUG_CT_DEBUG(format, ...) \
    printf("[CT][%s][%d]" format, __func__, __LINE__, ##__VA_ARGS__)

#define CT_DEBUG_ENABLE 1
#ifdef CT_DEBUG_ENABLE
#define CT_DEBUG CT_DEBUG_CT_DEBUG
#else
#define CT_DEBUG(fmt, args...) do {} while (0)
#endif

#define USE_PTHREAD_TO_GET_COLOR_AND_DEPTH_IMAGE    1

#define DEFAULT_DEVICE_INDEX                        (0)
#define DEFAULT_COLOR_IMG_FORMAT                    PIX_FMT_MJPEG
#define DEFAULT_COLOR_IMG_WIDTH                     (1280)
#define DEFAULT_COLOR_IMG_HEIGHT                    (720)
#define DEFAULT_DEPTH_IMG_WIDTH                     (640)
#define DEFAULT_DEPTH_IMG_HEIGHT                    (360)
#define UNUSED(x)                                   (void)(x)

#define APC_DEPTH_DATA_OFF_RAW                      0 /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_DEFAULT                      0 /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_8_BITS                       1 /* rectify, 1 byte per pixel */
#define APC_DEPTH_DATA_14_BITS                      2 /* rectify, 2 byte per pixel */
#define APC_DEPTH_DATA_8_BITS_x80                   3 /* rectify, 2 byte per pixel but using 1 byte only */
#define APC_DEPTH_DATA_11_BITS                      4 /* rectify, 2 byte per pixel but using 11 bit only */
#define APC_DEPTH_DATA_OFF_RECTIFY                  5 /* rectify (depth off, only rectify color) */
#define APC_DEPTH_DATA_8_BITS_RAW                   6 /* raw */
#define APC_DEPTH_DATA_14_BITS_RAW                  7 /* raw */
#define APC_DEPTH_DATA_8_BITS_x80_RAW               8 /* raw */
#define APC_DEPTH_DATA_11_BITS_RAW                  9 /* raw */
#define APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY     13// multi-baseline

/* Common usage. Other reference eSPDI_def.h and PIF document.*/
#define APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET       32
#define APC_DEPTH_DATA_SCALE_DOWN_8_BITS            (APC_DEPTH_DATA_8_BITS + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)/* rectify, 1 byte per pixel */\
#define APC_DEPTH_DATA_SCALE_DOWN_11_BITS           (APC_DEPTH_DATA_11_BITS + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)/* rectify, 2 byte per pixel but using 11 bit only */
#define APC_DEPTH_DATA_SCALE_DOWN_14_BITS           (APC_DEPTH_DATA_14_BITS + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* rectify, 2 byte per pixel */

#define TEST_RUN_NUMBER                             20
#define TEST_FRAME_COUNTER                          100
#define ABORT_FRAME_COUNTER                         5
#define TEST_RUN_NUMBER_MIPI                        TEST_FRAME_COUNTER + ABORT_FRAME_COUNTER_MIPI
#define TEST_FRAME_COUNTER_MIPI                     1
#define ABORT_FRAME_COUNTER_MIPI                    50
#define V4L2_BUFFER_QUQUE_SIZE                      32
#define V4L2_BUFFER_QUQUE_SIZE_FOR_INTERLEAVE       2
#define SAVE_FILE_PATH                              "./out_img/"

//s:[eys3D] 20200615 implement ZD table
#define COLOR_PALETTE_MAX_COUNT                     16384

#define COLOR_STR                                   "COLOR"
#define DEPTH_STR                                   "DEPTH"

#define GetDevSelectIndexPtr() ({ \
    if (gsDevSelInfo.index < SIMPLE_DEV_START_IDX) \
        gsDevSelInfo.index = APC_GetCompositeDevSelectIndex(EYSD, gsDevSelInfo.index); \
    else \
        gsDevSelInfo.index = APC_GetSimpleDevSelectIndex(EYSD, gsDevSelInfo.index); \
    &gsDevSelInfo; \
})

static int g_v4l2_buffer_quque_size = V4L2_BUFFER_QUQUE_SIZE;

bool DEBUG_LOG = true;

DEPTH_TRANSFER_CTRL g_depth_output = DEPTH_IMG_COLORFUL_TRANSFER;
BYTE g_pzdTable[APC_ZD_TABLE_FILE_SIZE_11_BITS];
unsigned short g_distance_table[APC_ZD_TABLE_FILE_SIZE_11_BITS / 2];

RGBQUAD *g_ColorPaletteZ14 = nullptr;
RGBQUAD *g_GrayPaletteZ14 = nullptr;

unsigned short g_maxFar;
unsigned short g_maxNear;
int g_zdTableInfo_index = 0;

static DEVINFORMATION *g_pDevInfo = nullptr;
static DEVSELINFO gsDevSelInfo;
//e:[eys3D] 20200615 implement ZD table

void* EYSD = NULL;
//pthread_mutex_t save_file_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t gcolor_thread_id = (pthread_t) - 1;
static pthread_t gdepth_thread_id = (pthread_t) - 1;

static int gColorFormat     = 1; // 0: YUYV, 1: MJPEG
static int gColorWidth      = 1280;
static int gColorHeight     = 720;
JsonFileSerializer* gJsonFileSerializer = nullptr;
static bool snapShot_color  = true;
static bool snapShot_raw    = true;
static bool snapShot_depth  = true;
static bool snapShot_rgb    = true;
static bool snapShot_mipi   = true;
static bool bTesting_color  = true;
static bool bTesting_depth  = true;
static bool bTesting_mipi   = true;
static bool bTestEnd_color  = false;
static bool bTestEnd_depth  = false;
static bool bTestEnd_mipi   = false;
static bool gInterleave     = false;
static bool gSplitImage     = false;
static bool gRectifyData    = true;
static bool gContinueMode   = false;
static bool gIgnoreDataTime = false;
static bool bKeepStreaming  = false;

typedef enum {
    ERROR_NONE                  =  0, /**< Successful */
    ERROR_NO_SUCH_DEVICE        = -1, /**< No such device or address */
    ERROR_NOT_SUPPORTED         = -2, /**< Not supported in this device */
    ERROR_NOT_PERMITTED         = -3, /**< Operation not permitted */
    ERROR_PERMISSION_DENIED     = -4, /**< Permission denied */
    ERROR_RESOURCE_BUSY         = -5, /**< Device or resource busy */
    ERROR_ALREADY_IN_PROGRESS   = -6, /**< Operation already in progress */
    ERROR_OUT_OF_MEMORY         = -7, /**< Out of memory */
    ERROR_INVALID_PARAMETER     = -8, /**< Invalid parameter */
    ERROR_INVALID_OPERATION     = -9, /**< Invalid Operation */
    ERROR_IO_ERROR              = -10,/**< IO ERROR */
    ERROR_TIMED_OUT             = -11,/**< Time out */
    ERROR_UNKNOWN               = -12,/**< Unknown */
} error_e;

typedef struct {
    int i2c_bus;
    uint16_t i2c_slave_addr;
    int vc_id;
} MipiInfo;

MipiInfo *g_pMipiInfo = nullptr;

DEPTH_TRANSFER_CTRL gDepth_Transfer_ctrl = DEPTH_IMG_NON_TRANSFER;

static int gDepthWidth                  = 640; // Depth is only YUYV format
static int gDepthHeight                 = 720;
PAPC_STREAM_INFO gpsStreamColorInfo     = NULL; 
PAPC_STREAM_INFO gpsStreamDepthInfo     = NULL;

static int gActualFps                   = 30;
static unsigned char *gColorImgBuf      = NULL;
static unsigned char *gDepthImgBuf      = NULL;

//s:[eys3D] 20200610 implement to save raw data to RGB format
static unsigned char *gTempImgBuf       = NULL;
static unsigned char *gColorRGBImgBuf   = NULL;
static unsigned char *gDepthRGBImgBuf   = NULL;
//e:[eys3D] 20200610 implement to save raw data to RGB format
static unsigned long int gColorImgSize  = 0;
static unsigned long int gDepthImgSize  = 0;
static int gDepthDataType               = APC_DEPTH_DATA_8_BITS;
static int gColorSerial                 = 0;
static int gDepthSerial                 = 0;
static unsigned short gSetupIRValue     = 0xff;

#define CT_CTRL_SEL_NUM                 18
#define PU_CTRL_SEL_NUM                 19
char *gCtCtrlSelectors[CT_CTRL_SEL_NUM] = {
    "CT_CONTROL_UNDEFINED",
    "CT_SCANNING_MODE_CONTROL",
    "CT_AE_MODE_CONTROL",
    "CT_AE_PRIORITY_CONTROL",
    "CT_EXPOSURE_TIME_ABSOLUTE_CONTROL",
    "CT_EXPOSURE_TIME_RELATIVE_CONTROL",
    "CT_FOCUS_ABSOLUTE_CONTROL",
    "CT_FOCUS_RELATIVE_CONTROL",
    "CT_FOCUS_AUTO_CONTROL",
    "CT_IRIS_ABSOLUTE_CONTROL",
    "CT_IRIS_RELATIVE_CONTROL",
    "CT_ZOOM_ABSOLUTE_CONTROL",
    "CT_ZOOM_RELATIVE_CONTROL",
    "CT_PANTILT_ABSOLUTE_CONTROL",
    "CT_PANTILT_RELATIVE_CONTROL",
    "CT_ROLL_ABSOLUTE_CONTROL",
    "CT_ROLL_RELATIVE_CONTROL",
    "CT_PRIVACY_CONTROL"
};
char *gPuCtrlSelectors[PU_CTRL_SEL_NUM] = {
    "PU_CONTROL_UNDEFINED",
    "PU_BACKLIGHT_COMPENSATION_CONTROL",
    "PU_BRIGHTNESS_CONTROL",
    "PU_CONTRAST_CONTROL",
    "PU_GAIN_CONTROL",
    "PU_POWER_LINE_FREQUENCY_CONTROL",
    "PU_HUE_CONTROL",
    "PU_SATURATION_CONTROL",
    "PU_SHARPNESS_CONTROL",
    "PU_GAMMA_CONTROL",
    "PU_WHITE_BALANCE_TEMPERATURE_CONTROL",
    "PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL",
    "PU_WHITE_BALANCE_COMPONENT_CONTROL",
    "PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL",
    "PU_DIGITAL_MULTIPLIER_CONTROL",
    "PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL",
    "PU_HUE_AUTO_CONTROL",
    "PU_ANALOG_VIDEO_STANDARD_CONTROL",
    "PU_ANALOG_LOCK_STATUS_CONTROL"
};

static constexpr int APC_USER_SETTING_OFFSET = 5;
static void write_calibration_data(int index);
static void copy_file_to_g2(int index);
int load_file_data_to_buffer(uint8_t* p_buffer, int size, const char* filename);
int saveRawFile(const char *pFilePath, BYTE *pBuffer, unsigned int sizeByte);

//s:[eys3D] 20200610 definition functions
int convert_yuv_to_rgb_pixel(int y, int u, int v);
int convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height, bool isMIPI);
int save_file(unsigned char *buf, int size, int width, int height,int type, bool isRGBNamed);
int get_product_name(char *path, char *out);
void print_APC_error(int error);
static int error_msg(int error);
static void setupDepth(void);
static void *pfunc_thread_close(void *arg);
static long long calcByGetTimeOfDay() ;

int tjpeg2yuv(unsigned char* jpeg_buffer, int jpeg_size, unsigned char** yuv_buffer, int* yuv_size, int* yuv_type);
int tyuv2rgb(unsigned char* yuv_buffer, int yuv_size, int width, int height, int subsample, unsigned char** rgb_buffer, int* rgb_size);
int getZDtable(DEVINFORMATION *pDevInfo, DEVSELINFO DevSelInfo, int depthHeight,int colorHeight);


int getPointCloud(void *pHandleEYSD, DEVSELINFO *pDevSelInfo, unsigned char *ImgColor, int CW, int CH,
                  unsigned char *ImgDepth, int DW, int DH, int depthDataType,
                  unsigned char *pPointCloudRGB,
                  float *pPointCloudXYZ,
                  float fNear,
                  float fFar);

int saveDepth2rgb(unsigned char *m_pDepthImgBuf, unsigned char *m_pRGBBuf, unsigned int m_nImageWidth, unsigned int m_nImageHeight);
void setHypatiaVideoMode(int mode);
int setupIR(unsigned short value);
void UpdateD8bitsDisplayImage_DIB24(RGBQUAD *pColorPalette, unsigned char *pDepth, unsigned char *pRGB, int width, int height);
void UpdateD11DisplayImage_DIB24(const RGBQUAD* pColorPalette, const unsigned char *pDepth, unsigned char *pRGB, int width, int height);
void UpdateZ14DisplayImage_DIB24(RGBQUAD *pColorPaletteZ14, unsigned char *pDepthZ14, unsigned char *pDepthDIB24, int cx, int cy);
char* PidToModuleName(unsigned short pid);

static void *get_exposure_time_test_func(void *arg);
static void *set_exposure_time_test_func(void *arg);
static void *get_global_gain_test_func(void *arg);
static void *set_global_gain_test_func(void *arg);
static void *get_AE_target_func(void *arg);
static void *set_AE_target_func(void *arg);
static void *get_property_bar_info_func(void *arg);
static void *property_bar_test_func(void *arg);
static void *mipi_point_cloud_test_func(void *arg);
static void *get_thermal_sensor_2075_temperature_test_func(void *arg);
static void *imu_test_func_for_YX8062(void *arg);

static int init_device(bool is_select_dev);
static void *test_color_time_stamp(void *arg);
static void *test_depth_time_stamp(void *arg);
static void *test_color_depth_time_stamp(void *arg);
static void *pfunc_thread_color(void *arg);
static void *pfunc_thread_depth(void *arg);
static void *pfunc_thread_mipi(void *arg);
static int open_device_default(bool two_open, int colorWidth, int colorHeight, int depthWidth, int depthHeight,int fps, WORD videoMode, bool blkMode);
static int open_device(void);
static void get_color_image(void);
static void get_depth_image(void);
static void get_mipi_image(void);
static void get_point_cloud(void);
static void get_point_cloud_8063(void);
static int getPointCloudInfo(void *pHandleEYSD, DEVSELINFO *pDevSelInfo, PointCloudInfo *pointCloudInfo, int depthDataType, int depthHeight);
static void close_device(void);
static void release_device(void);
static void SetSnapShotFlag(void);
static void setupFWRegister_EX8038(void);
static void setupFWRegister(void);
static void setupFWRegister(void);
static void readFWRegister(void);
static void SetCounterMode(void);
static void GetCounterMode(void);
static void setV4L2buffer(void);
static void setIRValue(void);
static void Read3X(void);
static void Write3X(void);
static void Read4X(void);
static void Write4X(void);
static void Read5X(void);
static void Write5X(void);
static void Read24X(void);
static void Write24X(void);
static void ReadAllCalibrationData(void);
static void PointCloudFPS(void);
static void ResetUNPData(void);
static void ReadPlugIn(void);
//e:[eys3D] 20200610 definition functions
static void test_file_saving(APCImageType::Value type);
static int TransformDepthDataType(int *nDepthDataType, bool bRectifyData);
void* ppPostProcessHandle = nullptr;
void* ppDecimationFilterHandle = nullptr;
static unsigned char *alloc_single_depth(void *arg);
static int release_single_depth(void *arg);
//kbhit
static int keycode_of_key_being_pressed();
static void *pfunc_thread_keyboard_handler(void *arg);
//Test Pause/Resume
static void test_pause_resume_streaming(void);
static void pause_streaming();
static void resume_streaming();
static int high_low_exchange(int data);
static void *set_analog_gain_test_func(void *arg);
static void *set_digital_gain_test_func(void *arg);
static unsigned int gCameraPID = 0xffff;

#define _ENABLE_INTERACTIVE_UI_ 1
//#define _ENABLE_FILESAVING_DEMO_UI_ 1
//#define _ENABLE_FILESAVING_DEMO_POINT_CLOUD_UI_ 1
#define DEFAULT_SAVING_FRAME_COUNT 150

#include <thread>
#include <functional>

namespace libeys3dsample {
class Frame {
public:
    Frame(size_t dataBufferSize) {
        dataVec.resize(dataBufferSize);
    }
    // Move constructor + assignment
    Frame(Frame&& f) = default;
    Frame& operator=(Frame&& f) = default;
    // Copy ctor + assignment disallow
    Frame (const Frame&) = delete;
    Frame& operator=(Frame& f) = delete;
    int32_t width;
    int32_t height;
    uint64_t actualImageSize;
    int sn = 0;
    std::vector<u_int8_t> dataVec;
    int status = 0;
    int64_t ts_s = 0;
    int64_t ts_us = 0;
};

/***
 * FrameCallbackHelper a simple class to implement get image raw data. (Color|Depth)CallbackHelper inherit basic
 * behaviors except runner. Provided that APC_SetupBlock already set to true, the APC_Get(Color|Depth)Image will
 * get an image when image is ready.
 */

class FrameCallbackHelper {
private:
    bool mIsStart = false;
    std::thread mCallbackLooper;
protected:
    void* mDeviceHandle = nullptr;
    DEVSELINFO mDevSelInfo;
public:
    Frame* mFrame = nullptr;
    using Callback = std::function<bool(const Frame* frame)>;
    Callback mCallback;
    explicit FrameCallbackHelper(void* handle, DEVSELINFO devSelInfo, size_t w, size_t h, size_t bpp, Callback& cb) {
        mDeviceHandle = handle;
        mDevSelInfo.index = devSelInfo.index;
        mCallback = cb;
        mFrame = new libeys3dsample::Frame(w * h * bpp);
        mFrame->width = w;
        mFrame->height = h;
    }

    inline bool isStart() { return mIsStart; }

    ~FrameCallbackHelper() {
        stopThreadAndJoin();
        delete mFrame;
    }

    // Delete copy assignment and ctor
    FrameCallbackHelper (const FrameCallbackHelper&) = delete;
    FrameCallbackHelper& operator=(FrameCallbackHelper& helper) = delete;

    void stopThreadAndJoin(){
        if (isStart()) {
            mIsStart = false;
            mCallbackLooper.join();
        }
    }

    virtual void runner(Frame* f) = 0;

    void start() {
        if (!isStart()) {
            mIsStart = true;
            mCallbackLooper = std::thread( [this] { runner(mFrame); } );
        }
    }

    void stop() {
        stopThreadAndJoin();
    }

};

class ColorCallbackHelper : public FrameCallbackHelper {
public:
    explicit ColorCallbackHelper(void* handle, DEVSELINFO info, size_t w, size_t h, size_t bpp, Callback& cb) :
                FrameCallbackHelper(handle, info, w, h, bpp, cb) {};
    ColorCallbackHelper (const ColorCallbackHelper&) = delete;
    ColorCallbackHelper& operator=(ColorCallbackHelper& helper) = delete;

    virtual void runner(Frame *f) override {
        if (!f->width || !f->height) {
            return;
        }
        while (isStart()) {
            f->status = APC_GetColorImageWithTimestamp(mDeviceHandle, &mDevSelInfo, f->dataVec.data(),
                                                       &f->actualImageSize, &f->sn, 0, &f->ts_s, &f->ts_us);
            mCallback(f);
        }
    }
};

class DepthCallbackHelper : public FrameCallbackHelper {
public:
    explicit DepthCallbackHelper(void* handle, DEVSELINFO info, size_t w, size_t h, size_t bpp, Callback& cb) :
                FrameCallbackHelper(handle, info, w, h, bpp, cb) {};
    DepthCallbackHelper (const DepthCallbackHelper&) = delete;
    DepthCallbackHelper& operator=(DepthCallbackHelper& helper) = delete;

    virtual void runner(Frame *f) override {
        if (!f->width || !f->height) {
            return;
        }
        while (isStart()) {
            f->status = APC_GetDepthImageWithTimestamp(mDeviceHandle, &mDevSelInfo, f->dataVec.data(),
                                                       &f->actualImageSize, &f->sn, 0, &f->ts_s, &f->ts_us);
            mCallback(f);
        }
    }
};

}; // namespace libeys3d

void SimpleColorDepthCallback(APCImageType::Value imgType, int imgId, unsigned char *imgBuf, int imgSizeByte,
                              int width, int height, int serialNumber, long long timestamp, void *pParameter) {
    fprintf(stderr, "APC_OpenDeviceCallback reason type:%d id:%d buf:%p sz:%d w:%d h:%d sn:%d ts:%lld param:%p \n",
            imgType, imgId, imgBuf, imgSizeByte, width, height, serialNumber, timestamp, pParameter);
}

void SelectDevInx(int index) {
    if (g_pDevInfo[index].nDevType != OTHERS && g_pDevInfo[index].nDevType != UNKNOWN_DEVICE_TYPE) {
        gsDevSelInfo.index = index;
        gCameraPID = g_pDevInfo[gsDevSelInfo.index].wPID;
        CT_DEBUG("Selected Deivce [Index, Type] is [%d, %d]\n",
            gsDevSelInfo.index, g_pDevInfo[gsDevSelInfo.index].nDevType);
    }
}

int parse_cmd_config(const char *cmd, char *param[], int param_count, const char  *delim, uint32_t *count) {
    char *found = NULL;
    int i = 0;
    char *cmd_tmp = NULL;
    char **param_tmp = NULL;

    cmd_tmp = (char *)cmd;

    while( (found = strsep(&cmd_tmp,delim)) != NULL ) {
        if (i >= param_count) {
            break;
        }
        param_tmp =  &param[i];
        *param_tmp = found;
        i++;
    }

    *count = i;

    return APC_OK;
}

int GetDateTime(char * psDateTime) {
    time_t timep; 
    struct tm *p; 

    time(&timep); 
    p=localtime(&timep); 

    sprintf(psDateTime, "%04d%02d%02d_%02d%02d%02d", 
        (1900+p->tm_year),
        (1+p->tm_mon),
        p->tm_mday,
        p->tm_hour,
        p->tm_min,
        p->tm_sec);
    return 0;
}

static void SetSaveFileDefault(void) {
    snapShot_color = true;
    snapShot_raw = true;
    snapShot_depth = true;
    snapShot_rgb = true;
}

static void NeedToEnableDebugLog(void) {
    char input[64];

    printf("Need to enable Debug Log? (Yes/No)\n");
    scanf("%s", input);
    
    if (input[0] == 'Y' || input[0] == 'y') {
        DEBUG_LOG = true;
    } else {
        DEBUG_LOG = false;
    }
}

static void NeedToSaveFile(void) {
    char input[64];

    printf("Need to save as files? (Yes/No)\n");
    scanf("%s", input);
    
    if (input[0] == 'Y' || input[0] == 'y') {
        snapShot_color = true;
        snapShot_depth = true;
        snapShot_mipi  = true;
    } else {
        snapShot_color = false;
        snapShot_depth = false;
        snapShot_mipi  = false;
    }
    snapShot_raw = false;
    snapShot_rgb = false;
}

static void NeedToEnableInterleaveMode(void) {
    char input[64];

    printf("Need to enable Interleave Mode? (Yes/No)\n");
    scanf("%s", input);
    
    if (input[0] == 'Y' || input[0] == 'y') {
        gInterleave = true;
    } else {
        gInterleave = false;
    }
}

static void NeedToSetupContinueMode(void) {
    char input[64];

    printf("Need to enable MIPI-TX as Continue Mode? (Yes/No)\n");
    scanf("%s", input);
    
    if (input[0] == 'Y' || input[0] == 'y') {
        gContinueMode = true;
    } else {
        gContinueMode = false;
    }
}

static void NeedToSplitImage(void) {
    char input[64];

    printf("Need to split Color Image and Depth Image? (Yes/No)\n");
    scanf("%s", input);
    
    if (input[0] == 'Y' || input[0] == 'y') {
        gSplitImage = true;
    } else {
        gSplitImage = false;
    }
}

static void NeedToRectifyData(void) {
    char input[64];

    printf("Need to enable Rectify Data? (Yes/No)\n");
    scanf("%s", input);
    
    if (input[0] == 'Y' || input[0] == 'y') {
        gRectifyData = true;
    } else {
        gRectifyData = false;
    }
}

static int SetContinueMode(void) {
    int ret = APC_OK;
    bool bContinueMode = false;

    ret = APC_GetFWContinueMode(EYSD, GetDevSelectIndexPtr(), &bContinueMode);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetFWContinueMode()(%d)\n", ret);
    } else {
        if (DEBUG_LOG) CT_DEBUG("Call APC_GetFWContinueMode() (%d, %s)\n", ret, bContinueMode ? "ENABLE" : "DISABLE");
    }

    ret = APC_SetFWContinueMode(EYSD, GetDevSelectIndexPtr(), gContinueMode);
    if (ret == APC_OK) {
        if (DEBUG_LOG) CT_DEBUG("Call APC_SetFWContinueMode() (%d, %s)\n", ret, gContinueMode ? "ENABLE" : "DISABLE");
        ret = APC_GetFWContinueMode(EYSD, GetDevSelectIndexPtr(), &bContinueMode);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetFWContinueMode()(%d)\n", ret);
        } else {
            CT_DEBUG("Call APC_GetFWContinueMode() (%d, %s)\n", ret, bContinueMode ? "ENABLE" : "DISABLE");
        }
    } else {
        CT_DEBUG("Failed to call APC_SetFWContinueMode()(%d)\n", ret);
    }

    return ret;
}

static int SetVirtualChannel(void) {
    int ret = APC_OK;
    int nVirtualChannelID0 = 0, nVirtualChannelID1 = 0;

    ret = APC_GetHWVirtualChannel(EYSD, GetDevSelectIndexPtr(), &nVirtualChannelID0, &nVirtualChannelID1);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetHWVirtualChannel()(%d)\n", ret);
    } else {
        if (DEBUG_LOG) CT_DEBUG("Call APC_GetHWVirtualChannel() (%d, %d, %d)\n", ret, nVirtualChannelID0, nVirtualChannelID1);
    }

    ret = APC_SetHWVirtualChannel(EYSD, GetDevSelectIndexPtr(), g_pMipiInfo[gsDevSelInfo.index].vc_id);
    if (ret == APC_OK) {
        if (DEBUG_LOG) CT_DEBUG("Call APC_SetHWVirtualChannel() (%d, %d)\n", ret, g_pMipiInfo[gsDevSelInfo.index].vc_id);
        ret = APC_GetHWVirtualChannel(EYSD, GetDevSelectIndexPtr(), &nVirtualChannelID0, &nVirtualChannelID1);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetHWVirtualChannel()(%d)\n", ret);
        } else {
            CT_DEBUG("Call APC_GetHWVirtualChannel() (%d, %d, %d)\n", ret, nVirtualChannelID0, nVirtualChannelID1);
        }
    } else {
        CT_DEBUG("Failed to call APC_SetHWVirtualChannel()(%d)\n", ret);
    }

    return ret;
}

static int GetMipiInfo(void) {
    int ret = APC_OK;
    char pBusInfo[16] = {0};
    int nBusInfoLength;

    if (APC_OK == APC_GetBusInfo(EYSD, GetDevSelectIndexPtr(), pBusInfo, &nBusInfoLength)) {
        char *param_strs[3] = {0};
        uint32_t count = 0;
        unsigned long lval = 0;
        ret = parse_cmd_config(pBusInfo, param_strs, 3, "-", &count);
        if (ret == APC_OK) {
            if (!param_strs[0] || !param_strs[1] || !param_strs[2]) {
                CT_DEBUG("Failed to parse [%s]\n", pBusInfo);
                return APC_Init_Fail;
            }
            lval = strtoul (param_strs[0], NULL, 10);
            g_pMipiInfo[gsDevSelInfo.index].i2c_bus = (int)lval;
            lval = strtoul (param_strs[1], NULL, 16);
            g_pMipiInfo[gsDevSelInfo.index].i2c_slave_addr  = (uint16_t)(lval & 0xff);
            lval = strtoul (param_strs[2], NULL, 10);
            g_pMipiInfo[gsDevSelInfo.index].vc_id = (int)lval;
            if (DEBUG_LOG) CT_DEBUG("Dev#%d: [bus, address, id] = [%d, 0x%02x, %d]\n",
                gsDevSelInfo.index,
                g_pMipiInfo[gsDevSelInfo.index].i2c_bus,
                g_pMipiInfo[gsDevSelInfo.index].i2c_slave_addr,
                g_pMipiInfo[gsDevSelInfo.index].vc_id);
        }
    } else {
        CT_DEBUG("APC_GetBusInfo() Fail! (%d)\n", ret);
    }

    return ret;
}

int main(void)
{
    int input = 0, input_idx = 0;
    do {
        printf("\n-----------------------------------------\n");
        printf("Software version: %s\n", APC_VERSION);
        printf("Please choose fllowing steps:\n");
#if defined(_ENABLE_INTERACTIVE_UI_)        
        printf(" 0. Init device\n");
        printf(" 1. Open device\n");
        printf(" 2. Get Color Image\n");
        printf(" 3. Get Depth Image\n");
        printf(" 4. Get Color and Depth Image\n");
        printf(" 5. Get Point Cloud\n");
        printf(" 6. Get MIPI Image\n");
        printf(" 8. Close Device\n");
        printf(" 9. Release Device\n");
        printf("10. SnapShot\n");
        printf("11. Set IR Value\n");
        printf("20. FW Reg Write\n");
        printf("21. FW Reg Read\n");
        printf("22. Camera intrinsic/extrinsic Read3X Y Offset\n");
        //printf("23. Camera intrinsic/extrinsic Write3X\n");
        printf("24. Camera intrinsic/extrinsic Read4X Rectify\n");
        //printf("25. Camera intrinsic/extrinsic Write4X\n");
        printf("26. Camera intrinsic/extrinsic Read5X ZD table\n");
        //printf("27. Camera intrinsic/extrinsic Write5X\n");
        printf("28. Camera intrinsic/extrinsic Read24X Log\n");
        //printf("29. Camera intrinsic/extrinsic Write24X\n");
        printf("30. Reset UNPData\n");
        printf("31. Point Cloud FPS Demo\n");
        printf("33. ReadPlugIn\n");
#endif
#if defined(_ENABLE_FILESAVING_DEMO_UI_)
        printf("50. save file demo   (1280X720@30, APC_DEPTH_DATA_14_BITS)\n");
#endif
#if defined(_ENABLE_FILESAVING_DEMO_POINT_CLOUD_UI_)
        printf("51. point cloud Demo (1280X720@30, APC_DEPTH_DATA_14_BITS)\n");
#endif
        printf("81. Force overwrite calibration data G1 section to G2, 30 40 50 240. (Use at your own risk.)\n");
        printf("85. IVY Camera (2248X1364@30, APC_DEPTH_DATA_OFF_RAW, USB)\n");
        printf("86. IVY2 / IVY4 Camera Image Callback Helper (1104x1104@30, APC_DEPTH_DATA_11_BITS, USB)\n");
        printf("92. IVY2 Only Camera IMU Callback Helper HIDAPI with libusb \n");
        printf("93. IVY2 / IVY4 Camera Exposure: Analog/Digital Gain, Exposure Time\n");
        printf("94. IVY2 / IVY4 Camera requested case. Devices info. Save files. Demo callbacks. IVY4 without IMU.\n");
        printf("95. IVY2 IMU Enter FW download mode.\n");
        printf("96. Use Windows API open camera.\n");
        printf("97. IVY2 + IVY3 / IVY4 Camera streaming. IVY2 IMU test\n");
        printf("98. IVY4 Tool1 item Disable eSP876 ISP related function\n");
        printf("99. IVY4 Metadata implementation.\n");
        printf("101. IVY2 / IVY4 Batch ASIC register value.\n");
        printf("102. IVY2 / IVY4 Batch ASIC & Sensor & FW register value.\n");
        printf("----------Read/Write calibration data case----------\n");
        printf("103. IVY2 / IVY4 Read all calibration data from the camera, save as read.bin file\n");
        printf("     read.bin file will save at calibrationdata/read folder\n");
        printf("104. IVY2 / IVY4 Write the output write.bin file of the calibration tool to the camera.\n");
        printf("     Put write.bin file into calibrationdata/write folder\n");
        printf("105. General Streaming Loop Test.\n");
        printf("106. Profile YUY2 decode Loop Test.\n");
        printf("255. EXIT)\n");
        scanf("%d", &input);
        switch (input) {
#if defined(_ENABLE_INTERACTIVE_UI_)
        case 0:
            init_device(true);
            break;
        case 1:
            open_device();
            break;
        case 2:
            get_color_image();
            if (gcolor_thread_id != (pthread_t) - 1)
                pthread_join(gcolor_thread_id, NULL);
            break;
        case 3:
            get_depth_image();
            if (gdepth_thread_id != (pthread_t) - 1)
                pthread_join(gdepth_thread_id, NULL);
            break;
        case 4:{
            int delay_time = 0;
#if defined(qcom)
            /*
            NOTE: This is workaround for QCS610 development board.
                  Because, this board needs more time for the UVC setup time.
                  So, We add the delay time with the V4L2 UVC setup process.
            */
            delay_time = 6;
            printf("input wait time between get color and get depth, we test QCS610 need 6 sec\n");
            scanf("%d", &delay_time);            
#endif
            get_color_image();
            usleep(delay_time * 1000 * 1000);
            get_depth_image();}
            if (gdepth_thread_id != (pthread_t) - 1)
                pthread_join(gdepth_thread_id, NULL);
            if (gcolor_thread_id != (pthread_t) - 1)
                pthread_join(gcolor_thread_id, NULL);
            break;
        case 5:
            get_point_cloud();
            break;
        case 6:
            get_mipi_image();
            if (gdepth_thread_id != (pthread_t) - 1)
                pthread_join(gdepth_thread_id, NULL);
            break;
        case 8:
            close_device();
            break;
        case 9:
            release_device();
            break;
        case 10:
            SetSnapShotFlag();
            break;
        case 11:
            setIRValue();
            break;
#endif
        case 20:
            setupFWRegister();
            break;
        case 21:
            readFWRegister();
            break;
        case 22:
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++)
                CT_DEBUG("Device Count: %d, Device Index List: %d\n", APC_GetDeviceNumber(EYSD), i);
            for (int i = 0; i < APC_GetSimpleDeviceNumber(EYSD); i++)
                CT_DEBUG("Simple Device Count: %d, Simple Device Index List: %d\n", APC_GetSimpleDeviceNumber(EYSD), SIMPLE_DEV_START_IDX + i);
            printf("Select Device Index (0...%d): ",
                (APC_GetSimpleDeviceNumber(EYSD) > 0) ? (SIMPLE_DEV_START_IDX + APC_GetSimpleDeviceNumber(EYSD) - 1) : (APC_GetDeviceNumber(EYSD) - 1));
            scanf("%d", &input_idx);
            printf("Selected Deivce Index is = [%d]\n", input_idx);
            SelectDevInx(input_idx);
            Read3X();
            break;
        case 23:
            Write3X();
            break;
        case 24:
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++)
                CT_DEBUG("Device Count: %d, Device Index List: %d\n", APC_GetDeviceNumber(EYSD), i);
            for (int i = 0; i < APC_GetSimpleDeviceNumber(EYSD); i++)
                CT_DEBUG("Simple Device Count: %d, Simple Device Index List: %d\n", APC_GetSimpleDeviceNumber(EYSD), SIMPLE_DEV_START_IDX + i);
            printf("Select Device Index (0...%d): ",
                (APC_GetSimpleDeviceNumber(EYSD) > 0) ? (SIMPLE_DEV_START_IDX + APC_GetSimpleDeviceNumber(EYSD) - 1) : (APC_GetDeviceNumber(EYSD) - 1));
            scanf("%d", &input_idx);
            printf("Selected Deivce Index is = [%d]\n", input_idx);
            SelectDevInx(input_idx);
            Read4X();
            break;
        case 25:
            Write4X();
            break;
        case 26:
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++)
                CT_DEBUG("Device Count: %d, Device Index List: %d\n", APC_GetDeviceNumber(EYSD), i);
            for (int i = 0; i < APC_GetSimpleDeviceNumber(EYSD); i++)
                CT_DEBUG("Simple Device Count: %d, Simple Device Index List: %d\n", APC_GetSimpleDeviceNumber(EYSD), SIMPLE_DEV_START_IDX + i);
            printf("Select Device Index (0...%d): ",
                (APC_GetSimpleDeviceNumber(EYSD) > 0) ? (SIMPLE_DEV_START_IDX + APC_GetSimpleDeviceNumber(EYSD) - 1) : (APC_GetDeviceNumber(EYSD) - 1));
            scanf("%d", &input_idx);
            printf("Selected Deivce Index is = [%d]\n", input_idx);
            SelectDevInx(input_idx);
            Read5X();
            break;
        case 27:
            Write5X();
            break;
        case 28:
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++)
                CT_DEBUG("Device Count: %d, Device Index List: %d\n", APC_GetDeviceNumber(EYSD), i);
            for (int i = 0; i < APC_GetSimpleDeviceNumber(EYSD); i++)
                CT_DEBUG("Simple Device Count: %d, Simple Device Index List: %d\n", APC_GetSimpleDeviceNumber(EYSD), SIMPLE_DEV_START_IDX + i);
            printf("Select Device Index (0...%d): ",
                (APC_GetSimpleDeviceNumber(EYSD) > 0) ? (SIMPLE_DEV_START_IDX + APC_GetSimpleDeviceNumber(EYSD) - 1) : (APC_GetDeviceNumber(EYSD) - 1));
            scanf("%d", &input_idx);
            printf("Selected Deivce Index is = [%d]\n", input_idx);
            SelectDevInx(input_idx);
            Read24X();
            break;
        case 29:
            Write24X();
            break;
        case 30:
            ResetUNPData();
            break;
        case 31:
            PointCloudFPS();
            break;
        case 33:
            ReadPlugIn();
            break;
        case 34:
            init_device(false);
            get_point_cloud_8063();
            release_device();
            break;
        case 35: {
            init_device(false);
            int cW = 1280, cH = 720;
            int dW = 1280, dH = 720;
            int fps = 30;
            open_device_default(true, cW, cH, dW, dH, fps, APC_DEPTH_DATA_11_BITS, false);

            // Adjust these parameters to get the best result.
            POST_PROCESS_PARAMS p;
            p.spatial_filter_kernel_size = 15;
            p.spatial_filter_outlier_threshold = 1;

            // Adjust these decimation filter parameters to get the best result.
            DECIMATION_PARAMS decimationParams;
            decimationParams.decimation_sub_sample_factor = 2;
            unsigned int decimationOutW = 0u;
            unsigned int decimationOutH = 0u;

            APC_InitPostProcessCustomParameter(&ppPostProcessHandle, dW, dH, APCImageType::DEPTH_11BITS, p);
            APC_InitDecimationFilter(&ppDecimationFilterHandle, dW, dH, &decimationOutW, &decimationOutH,
                                     APCImageType::DEPTH_11BITS, decimationParams);

            unsigned char *depthFrame = alloc_single_depth(nullptr);

            auto ImgColor = new unsigned char[cW * cH * 3];
            auto pPointCloudRGB = new unsigned char[cW * cH * 3];
            auto pPointCloudXYZ = new float[dW * dH * 3];
            memset(ImgColor, 0xff, cW * cH * 3); /* White */
            memset(pPointCloudRGB, 0x0, cW * cH * 3);
            memset(pPointCloudXYZ, 0x0, dW * dH * 3 * sizeof(float));

            PointCloudInfo pointCloudInfo;
            getPointCloudInfo(EYSD, &gsDevSelInfo, &pointCloudInfo, gDepthDataType, gDepthWidth);
            getPointCloud(EYSD, &gsDevSelInfo, ImgColor, cW, cH, depthFrame, dW, dH, APC_DEPTH_DATA_11_BITS,
                          pPointCloudRGB, pPointCloudXYZ, 0, 1000);

            unsigned char *filteredFrame = new unsigned char[dW * dH * 3];
            APC_PostProcess(ppPostProcessHandle, depthFrame, filteredFrame, APCImageType::DEPTH_11BITS);

            memset(ImgColor, 0x00, cW * cH * 3); /* Black */
            getPointCloud(EYSD, &gsDevSelInfo, ImgColor, cW, cH, filteredFrame, dW, dH, APC_DEPTH_DATA_11_BITS,
                          pPointCloudRGB, pPointCloudXYZ, 0, 1000);

            memset(ImgColor, 0x33, cW * cH * 3); /* Gray */
            memset(filteredFrame, 0x0, dW * dH * 3); /* Gray */
            APC_DecimationFilter(ppDecimationFilterHandle, depthFrame, filteredFrame, APCImageType::DEPTH_11BITS);
            getPointCloud(EYSD, &gsDevSelInfo, ImgColor, decimationOutW, decimationOutH, filteredFrame,
                          decimationOutW, decimationOutH, APC_DEPTH_DATA_11_BITS,
                          pPointCloudRGB, pPointCloudXYZ, 0, 1000);

            delete[] ImgColor;
            delete[] pPointCloudRGB;
            delete[] pPointCloudXYZ;
            delete[] filteredFrame;
            release_single_depth(nullptr);
            APC_ReleasePostProcess(ppPostProcessHandle);
            APC_ReleaseDecimationFilter(ppDecimationFilterHandle);

            close_device();
            release_device();
            break;
        }
#if defined(_ENABLE_FILESAVING_DEMO_UI_)
        case 50:
            init_device(false);
            open_device_default(true, 1280, 720, 1280, 720, 30, APC_DEPTH_DATA_14_BITS /* Distance mm*/, false);
            test_file_saving(APCImageType::Value::COLOR_YUY2);
            close_device();
            release_device();
            printf("Sync the filesytem! Please wait a minute!\n");
            sync();
        break;
#endif
#if defined(_ENABLE_FILESAVING_DEMO_POINT_CLOUD_UI_)
        case 51:
            init_device(false);
            open_device_default(true, 30, APC_DEPTH_DATA_14_BITS /* Distance mm*/, false);
            get_point_cloud();
            close_device();
            release_device();
        break;
#endif
        case 80:
            NeedToEnableDebugLog();
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++) {
                CT_DEBUG("######################################################################\n");
                CT_DEBUG("Device Count: %d, Select Device Index: %d\n", APC_GetDeviceNumber(EYSD), i);
                SelectDevInx(i);
                get_thermal_sensor_2075_temperature_test_func(NULL);
                get_global_gain_test_func(NULL);
                get_exposure_time_test_func(NULL);
            }
            CT_DEBUG("######################################################################\n");
            break;
        case 81: {
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++) {
                CT_DEBUG("######################################################################\n");
                CT_DEBUG("Device Count: %d, Select Device Index: %d\n", APC_GetDeviceNumber(EYSD), i);
                SelectDevInx(i);
            }

            for (int index = 0; index < APC_USER_SETTING_OFFSET; ++index) {
                copy_file_to_g2(index);
            }
            CT_DEBUG("######################################################################\n");
            break;
        }
        case 82: {
            init_device(false);
            open_device_default(false, 1280, 720, 1280, 720, 30, APC_DEPTH_DATA_14_BITS, true);
            void* handle = nullptr;
            size_t outW, outH;
            float factor = 0.5f;
            APC_InitResizeProcess(&handle, 1280, 720, &outW, &outH, APCImageType::COLOR_YUY2, factor);

            std::vector<uint8_t> colorImageVec(gColorWidth * gColorHeight * 2);
            int ret = APC_GetColorImage(EYSD, GetDevSelectIndexPtr(), (BYTE*)colorImageVec.data(), &gColorImgSize, &gColorSerial);

            std::vector<uint8_t> colorImageResizeVec(gColorWidth * gColorHeight * ((size_t) 2 * factor));
            ret = APC_ResizeProcess(handle, colorImageVec, colorImageResizeVec, APCImageType::COLOR_YUY2);
            CT_DEBUG("APC_ResizeProcess YUYV to YUYV %d\n", ret);

            std::vector<uint8_t> colorImageRGBResizeVec(gColorWidth * gColorHeight * 3 * factor /*Resolution will be even */);
            ret = APC_ResizeProcess(handle, colorImageVec, colorImageRGBResizeVec, APCImageType::COLOR_RGB24);
            CT_DEBUG("APC_ResizeProcess YUYV to RGB %d\n", ret);

            APC_ReleaseResizeProcess(handle);
            break;
        }
        case 83:
            NeedToEnableDebugLog();
            NeedToSplitImage();
            init_device(false);
            for (int i = 0; i < APC_GetSimpleDeviceNumber(EYSD); i++) {
                CT_DEBUG("######################################################################\n");
                CT_DEBUG("Simple Device Count: %d, Select Simple Device Index: %d\n", APC_GetSimpleDeviceNumber(EYSD), SIMPLE_DEV_START_IDX + i);
                SelectDevInx(SIMPLE_DEV_START_IDX + i);
                open_device_default(false, 1280, 720, 1280, 720, 30, APC_DEPTH_DATA_11_BITS, true);
                mipi_point_cloud_test_func(NULL);
                close_device();
            }
            CT_DEBUG("######################################################################\n");
            release_device();
            snapShot_mipi = true;
            break;
        case 84:
            NeedToEnableDebugLog();
            NeedToSplitImage();
            init_device(false);
            for (int i = 0; i < APC_GetSimpleDeviceNumber(EYSD); i++) {
                CT_DEBUG("######################################################################\n");
                CT_DEBUG("Simple Device Count: %d, Select Simple Device Index: %d\n", APC_GetSimpleDeviceNumber(EYSD), SIMPLE_DEV_START_IDX + i);
                SelectDevInx(SIMPLE_DEV_START_IDX + i);
                open_device_default(false, 1280, 720, 1280, 720, 30, APC_DEPTH_DATA_14_BITS, true);
                mipi_point_cloud_test_func(NULL);
                close_device();
            }
            CT_DEBUG("######################################################################\n");
            release_device();
            snapShot_mipi = true;
            break;
        case 85:
            NeedToSaveFile();
            CT_DEBUG("snapShot_color: %s, snapShot_depth: %s\n",
                     snapShot_color ? "TRUE" : "FALSE", snapShot_depth ? "TRUE" : "FALSE");
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++)
                CT_DEBUG("Device Count: %d, Device Index List: %d\n", APC_GetDeviceNumber(EYSD), i);
            printf("Select Device Index (0...%d): ", APC_GetDeviceNumber(EYSD) - 1);
            scanf("%d", &input_idx);
            printf("Selected Deivce Index is = [%d]\n", input_idx);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++) {
                if (input_idx != -1 && input_idx != i)
                    continue;
                CT_DEBUG("######################################################################\n");
                CT_DEBUG("Device Count: %d, Select Device Index: %d\n", APC_GetDeviceNumber(EYSD), i);
                SelectDevInx(i);
                open_device_default(true, 2248, 1364, 0, 0, 30, APC_DEPTH_DATA_OFF_RAW, true);
                gSetupIRValue = 60;
                get_color_image();

                // Workaround for IVY if open color only. Enable this block.
                if (false) {
                    RegisterSettings::DM_Quality_Register_Setting(EYSD, GetDevSelectIndexPtr(),
                                                                  g_pDevInfo[gsDevSelInfo.index].wPID);
                }

                set_digital_gain_test_func(nullptr);
                set_analog_gain_test_func(nullptr);
                if (gcolor_thread_id != (pthread_t) - 1)
                    pthread_join(gcolor_thread_id, NULL);
                close_device();
            }
            CT_DEBUG("######################################################################\n");
            release_device();
            SetSaveFileDefault();
            break;
        case 86: {
            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++)
                CT_DEBUG("Device Count: %d, Device Index List: %d\n", APC_GetDeviceNumber(EYSD), i);
            printf("Select Device Index (0...%d): ", APC_GetDeviceNumber(EYSD) - 1);
            scanf("%d", &input_idx);
            printf("Selected Deivce Index is = [%d]\n", input_idx);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++) {
                if (input_idx != -1 && input_idx != i)
                    continue;
                CT_DEBUG("######################################################################\n");
                CT_DEBUG("Device Count: %d, Select Device Index: %d\n", APC_GetDeviceNumber(EYSD), i);
                SelectDevInx(i);

                int cw = 1104;
                int ch = 1104;
                int dw = 1104;
                int dh = 1104;
                int f0 = APC_DEPTH_DATA_11_BITS;
                int fps = 30;
                printf("Choose YUYV:0 MJPEG:1 \n");
                scanf("%d", &gColorFormat);
                printf("Choose color width ");
                scanf("%d", &cw);
                printf("Choose color height ");
                scanf("%d", &ch);
                printf("Choose depth width ");
                scanf("%d", &dw);
                printf("Choose depth height ");
                scanf("%d", &dh);
                printf("Choose FPS \n");
                scanf("%d", &fps);
                printf("Choose Depth data type\nD(11Bits)=4\nZ(14Bits)=2\nColor Only L'+R'=5\nColor Only L+R=0\n");
                scanf("%d", &f0);

                open_device_default(true, cw, ch, dw, dh, fps, f0, true);
                // For IVY1DVT
                setupIR(20);

                auto grabColorImages = std::thread([=] () {
                    if (!cw || !ch) return ;

                    libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([] (const libeys3dsample::Frame *f) {
                        static int64_t lastTimeUs = 0;
                        int64_t curTimeUs = f->ts_s * 1000000 + f->ts_us;
                        if (lastTimeUs > 0) {
                            fprintf(stderr, "Inside color callback frame.sn=%d status=%d diffMs=%f\n", f->sn, f->status,
                                    (curTimeUs - lastTimeUs) / 1000.0f);
                        }
                        lastTimeUs = curTimeUs;
                        return f->status == APC_OK;
                    });

                    libeys3dsample::ColorCallbackHelper cbColorHelper(EYSD, *GetDevSelectIndexPtr(), cw, ch, 2,
                                                                      colorCallbackFn);

                    cbColorHelper.start();
                    sleep(20);
                    cbColorHelper.stop();
                });

                // Workaround for QCS610 IVY1DVT
                sleep(6);

                auto grabDepthImages = std::thread([=] () {
                    if (!dw || !dh) return ;

                    libeys3dsample::FrameCallbackHelper::Callback depthCallbackFn([] (const libeys3dsample::Frame *f) {
                        static int64_t lastTimeUs = 0;
                        int64_t curTimeUs = f->ts_s * 1000000 + f->ts_us;
                        if (lastTimeUs > 0) {
                            fprintf(stderr, "Inside depth callback frame.sn=%d status=%d diffMs=%f\n", f->sn, f->status,
                                    (curTimeUs - lastTimeUs) / 1000.0f);
                        }
                        lastTimeUs = curTimeUs;
                        return f->status == APC_OK;
                    });

                    libeys3dsample::DepthCallbackHelper cbDepthHelper(EYSD, *GetDevSelectIndexPtr(), dw, dh, 2,
                                                                      depthCallbackFn);

                    cbDepthHelper.start();
                    sleep(20);
                    cbDepthHelper.stop();
                });

                grabDepthImages.join();
                grabColorImages.join();

                close_device();
                CT_DEBUG("######################################################################\n");
            }

            release_device();
            break;
        }
        case 90:
            CT_DEBUG("######################################################################\n");
            imu_test_func_for_YX8062(NULL);
            CT_DEBUG("######################################################################\n");
            break;
        case 91: {
            // Only test for YX8062 other module might need to modify the flow !!
            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;
            int fps = 30;
            int cw = 1280; int ch = 720;
            int dw = 1280; int dh = 720;
            int depthDataType = 4;
            int isMjpeg = 0;

            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            printf("Color width:\n");
            scanf("%d", &cw);
            printf("Color height:\n");
            scanf("%d", &ch);
            printf("Depth width:\n");
            scanf("%d", &dw);
            printf("Depth height:\n");
            scanf("%d", &dh);
            printf("Choose FPS:\n");
            scanf("%d", &fps);
            printf("Choose Depth Data Type\n");
            scanf("%d", &depthDataType);

            APC_Init(&cameraHandle, false);

            APC_SetDepthDataType(cameraHandle, &devSelInfo, depthDataType);

            APC_OpenDevice2(cameraHandle, &devSelInfo, cw, ch, isMjpeg, dw, dh, DEPTH_IMG_NON_TRANSFER, false, nullptr,
                            &fps);

            libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([] (const libeys3dsample::Frame *f) {
                static bool everEnter = false;
                if (!everEnter) {
                    fprintf(stderr, "Inside color callback frame.sn=%d status=%d\n", f->sn, f->status);
                    everEnter = true;
                }
                return f->status;
            });

            libeys3dsample::FrameCallbackHelper::Callback depthCallbackFn([] (const libeys3dsample::Frame *f) {
                static bool everEnter = false;
                if (!everEnter) {
                    fprintf(stderr, "Inside depth callback frame.sn=%d status=%d\n", f->sn, f->status);
                    everEnter = true;
                }
                return f->status;
            });

            libeys3dsample::ColorCallbackHelper colorCallback(cameraHandle, devSelInfo, cw, ch, 2, colorCallbackFn);
            libeys3dsample::DepthCallbackHelper depthCallback(cameraHandle, devSelInfo, dw, dh, 2, depthCallbackFn);

            colorCallback.start();
            sleep(6);
            depthCallback.start();

            fprintf(stderr, "IMU++++\n");
            unsigned short nVID = 0x1E4E, nPID = 0x0163;

            CIMUModel::INFO info {
                    .nVID = nVID,
                    .nPID = nPID,
                    .axis = CIMUModel::IMU_9_AXIS
            };

            CIMUModel imuModel = CIMUModel(info, cameraHandle);

            std::string serialNumber = imuModel.GetCameraSerialNumber(cameraHandle, devSelInfo);
            fprintf(stderr, "Camera serialNumber:[%s]\n", serialNumber.c_str());
            std::string imuModuleName = imuModel.GetModuleName();
            fprintf(stderr, "GetModuleName[%s] CMD1-CMD4 \n", imuModuleName.c_str());
            std::string imuFWVersion = imuModel.GetFWVersion();
            fprintf(stderr, "GetFWVersion [%s] CMD5-CMD12\n", imuFWVersion.c_str());
            std::string imuOutputStatus = imuModel.GetStatus();
            fprintf(stderr, "GetOutputStatus[%s] CMD13 \n", imuOutputStatus.c_str());
            imuModel.EnableDataOutput(false);
            imuOutputStatus = imuModel.GetStatus();
            fprintf(stderr, "SetOutputStatus Disable [%s] CMD14 \n", imuOutputStatus.c_str());
            imuModel.EnableDataOutput(true);
            imuOutputStatus = imuModel.GetStatus();
            fprintf(stderr, "SetOutputStatus Enable [%s] CMD15 \n", imuOutputStatus.c_str());

            if (info.axis != CIMUModel::IMU_9_AXIS) {
                int imuDataOutputFormat = imuModel.ReadDataOutputFormat();
                fprintf(stderr, "ReadDataOutputFormat [%d] CMD16 \n", imuDataOutputFormat);

                imuModel.SelectDataFormat(CIMUModel::RAW_DATA_WITHOUT_OFFSET);
                fprintf(stderr, "SelectDataFormat RAW_DATA_WITHOUT_OFFSET 0x01 "
                                "[%d] CMD17-19 \n", imuModel.ReadDataOutputFormat());
                imuModel.SelectDataFormat(CIMUModel::RAW_DATA_WITH_OFFSET);
                fprintf(stderr, "SelectDataFormat RAW_DATA_WITH_OFFSET 0x02 "
                                "[%d] CMD17-19 \n", imuModel.ReadDataOutputFormat());

                imuModel.SelectDataFormat(CIMUModel::OFFSET_DATA);
                fprintf(stderr, "SelectDataFormat OFFSET_DATA 0x03 "
                                "[%d] CMD17-19 \n", imuModel.ReadDataOutputFormat());

                imuModel.SelectDataFormat(CIMUModel::DMP_DATA_WITHOT_OFFSET);
                fprintf(stderr, "SelectDataFormat DMP_DATA_WITHOT_OFFSET 0x04 "
                                "[%d] CMD17-19 \n", imuModel.ReadDataOutputFormat());

                imuModel.SelectDataFormat(CIMUModel::DMP_DATA_WITH_OFFSET);
                fprintf(stderr, "SelectDataFormat DMP_DATA_WITHOT_OFFSET 0x05 "
                                "[%d] CMD17-19 \n", imuModel.ReadDataOutputFormat());

                char checkIMUCalibrated;
                imuModel.CheckCalibratingStatus(&checkIMUCalibrated);
                fprintf(stderr, "CheckCalibratingStatus [%d] CMD22 \n", checkIMUCalibrated);
                imuModel.StartCalibration();

                imuModel.CheckCalibratingStatus(&checkIMUCalibrated);
                fprintf(stderr, "CheckCalibratingStatus StartCalibration CMD23 [CMD22:%d]  \n", checkIMUCalibrated);

                imuModel.ReadCalibrated(&checkIMUCalibrated);
                fprintf(stderr, "ReadCalibrated CMD24 [%d]\n", checkIMUCalibrated);
            }

            uint8_t readIMURegData = 0x0;
            int readIMURegStatus;
            readIMURegStatus = imuModel.ReadRegister(0x0, 0x0 /* REG_WHO_AM_I */, &readIMURegData);
            fprintf(stderr, "ReadRegister CMD25 status[%d] value[0x%x]\n", readIMURegStatus, readIMURegData);

            if (info.axis != CIMUModel::IMU_9_AXIS) {
                std::string imuModuleSN = imuModel.GetSerialNumber();
                fprintf(stderr, "GetSerialNumber[%s] CMD27-CMD31\n", imuModuleSN.c_str());

                imuModel.SetSerialNumber(imuModuleSN);
                fprintf(stderr, "SetSerialNumber[%s] CMD32-CMD39\n", imuModuleSN.c_str());
            }

            {
                FILE* imuLog = fopen("imulog.txt", "a+");
                // 1. Setup callback
                CIMUModel::Callback printCallback([&] (const IMUData *data) {
                    fprintf(imuLog, "CB FrameSN:[%d] mModID[%d] Time:%2d,%2d,%5d Quaternion:%5f,%5f,%5f,%5f _accuracy_FLAG:%d\n",
                            data->_frameCount, data->_moduleID, data->_min, data->_sec, data->_subSecond,
                            data->_quaternion[0], data->_quaternion[1], data->_quaternion[2], data->_quaternion[3],
                            data->_accuracy_FLAG);
                    return true;
                });

                // 2. Enable callback looper
                imuModel.EnableDataCallback(printCallback);
                sleep(10);

                // 3. Disable callback looper after use.
                imuModel.DisableDataCallback();

                time_t now = time(nullptr);
                fprintf(imuLog, "CB End %ld\n", now);
                fflush(imuLog);
                fclose(imuLog);
                fprintf(stderr, "CB End %ld\n", now);
            }
            fprintf(stderr, "IMU----\n");

            sleep(5);
            colorCallback.stop();
            depthCallback.stop();

            APC_CloseDevice(cameraHandle, &devSelInfo);
            APC_Release(&cameraHandle);
            break;
        }
        case 92: {

            /*
             * This case only verified at YX8083 Early sample IMU FW IVY2-D01-ELSM6DSR-BL00U-001-BETA01
             * Implement the STMG431_HID_CMD-1 document CMD1-CMD16, CMD24-CMD45
             * */

            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;

            int ret = APC_Init(&cameraHandle, false);

            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            unsigned short nVID = 0x3438, nPID = 0x0166;

            CIMUModel::INFO info {
                    .nVID = nVID,
                    .nPID = nPID,
                    .axis = CIMUModel::IMU_6_AXIS
            };

            CIMUModel imuModel = CIMUModel(info, cameraHandle);

            std::string serialNumber = imuModel.GetCameraSerialNumber(cameraHandle, devSelInfo);
            fprintf(stderr, "Camera serialNumber:[%s]\n", serialNumber.c_str());
            std::string imuModuleName = imuModel.GetModuleName();
            fprintf(stderr, "GetModuleName[%s] CMD1-CMD4 \n", imuModuleName.c_str());
            std::string imuFWVersion = imuModel.GetFWVersion();
            fprintf(stderr, "GetFWVersion [%s] CMD5-CMD12\n", imuFWVersion.c_str());
            std::string imuOutputStatus = imuModel.GetStatus();
            fprintf(stderr, "GetOutputStatus[%s] CMD13 \n", imuOutputStatus.c_str());
            imuModel.EnableDataOutput(false);
            imuOutputStatus = imuModel.GetStatus();
            fprintf(stderr, "SetOutputStatus Disable [%s] CMD14 \n", imuOutputStatus.c_str());
            imuModel.EnableDataOutput(true);
            imuOutputStatus = imuModel.GetStatus();
            fprintf(stderr, "SetOutputStatus Enable [%s] CMD15 \n", imuOutputStatus.c_str());
            imuModel.EnableDataOutput(false);
            imuOutputStatus = imuModel.GetStatus();
            fprintf(stderr, "SetOutputStatus Disable [%s] CMD14 \n", imuOutputStatus.c_str());

            uint8_t readIMURegData = 0x0;
            int readIMURegStatus;
            readIMURegStatus = imuModel.ReadRegister(0x0, 0x0F /* REG_WHO_AM_I */, &readIMURegData);
            fprintf(stderr, "ReadRegister CMD25 status[%d] value[0x%x]\n", readIMURegStatus, readIMURegData);

            std::string imuModuleSN = imuModel.GetSerialNumber();
            fprintf(stderr, "GetSerialNumber[%s] CMD27-CMD31\n", imuModuleSN.c_str());

            imuModel.SetSerialNumber(imuModuleSN);
            fprintf(stderr, "SetSerialNumber[%s] CMD32-CMD39\n", imuModuleSN.c_str());

            CIMUModel::RTC rtc { 0, 0, 0, 0 };
            imuModel.WriteRTC(rtc);
            auto accFS = imuModel.ReadAccFS();
            fprintf(stderr, "WriteRTC reset to 0 CMD41, CMD42 ACCFS: %d\n", accFS);
            //accFS = 8;
            imuModel.WriteAccFS(accFS);
            if (accFS == imuModel.ReadAccFS()) fprintf(stderr, "WriteAccFS CMD43 equals the value had written\n");

            auto gyro = imuModel.ReadGyroFS();
            fprintf(stderr, "ReadGyroFS CMD44 value %d \n", gyro);
            //gyro = 4;
            imuModel.WriteGyroFS(gyro);
            fprintf(stderr, "WriteGyroFS CMD45 value %d \n", gyro);
            if (gyro == imuModel.ReadGyroFS()) fprintf(stderr, "WriteGyroFS CMD45 equals the value had written\n");

            {
                FILE* imuLog = fopen("imulog.txt", "a+");
                // 1. Setup callback
                CIMUModel::Callback printCallback([&] (const IMUData *data) {
                    fprintf(stderr, "FrameSN:[%d] Time: %2d,%2d,%2d,%5d\nacc:%5f,%5f,%5f tmp:%2d\ngyro:%5f,%5f,%5f\n\n",
                            data->_frameCount, data->_hour, data->_min, data->_sec, data->_subSecond,
                            data->_accelX, data->_accelY, data->_accelZ, data->_temprature,
                            data->_gyroScopeX, data->_gyroScopeY, data->_gyroScopeZ);
                    fprintf(imuLog, "FrameSN:[%d] Time: %2d,%2d,%2d,%5d\nacc:%5f,%5f,%5f tmp:%2d\ngyro:%5f,%5f,%5f\n\n",
                            data->_frameCount, data->_hour, data->_min, data->_sec, data->_subSecond,
                            data->_accelX, data->_accelY, data->_accelZ, data->_temprature,
                            data->_gyroScopeX, data->_gyroScopeY, data->_gyroScopeZ);
                    return true;
                });

                // 2. Enable callback looper
                imuModel.EnableDataCallback(printCallback);
                sleep(2);
                // 3. Test Time Sync function.
                for (int i = 0; i < 60; ++i)
                {
                    imuModel.SetTimeSync(2);
                    //Don's set any HID command between Set and get TimeSync.
                    auto TimeSync = imuModel.ReadTimeSync();
                    fprintf(stderr, "ReadTimeSync: %02x %02x %02x %02x %02x %02x %02x %02x CMD47\n", TimeSync.sn, TimeSync.time1, TimeSync.time2, TimeSync.time3, TimeSync.time4, TimeSync.diff1, TimeSync.diff2, TimeSync.diff3);
                    fprintf(imuLog, "ReadTimeSync: %02x %02x %02x %02x %02x %02x %02x %02x CMD47\n", TimeSync.sn, TimeSync.time1, TimeSync.time2, TimeSync.time3, TimeSync.time4, TimeSync.diff1, TimeSync.diff2, TimeSync.diff3);
                    auto rtc = imuModel.ReadRTC();
                    fprintf(stderr, "GetRTC %d %d %d %d CMD40\n", rtc.hour, rtc.min, rtc.sec, rtc.subSecond);
                    fprintf(imuLog, "GetRTC %d %d %d %d CMD40\n", rtc.hour, rtc.min, rtc.sec, rtc.subSecond);
                    sleep(1);
                }
                // 4. Disable callback looper after use.
                imuModel.DisableDataCallback();

                time_t now = time(nullptr);
                fprintf(imuLog, "CB End %ld\n", now);
                fflush(imuLog);
                fclose(imuLog);
                fprintf(stderr, "CB End %ld\n", now);
            }

            sleep(5);
            APC_Release(&cameraHandle);
            break;
        }
        case 93: {
            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;

            int fps = 30;
            int cw = 1280; int ch = 720;
            int dw = 1280; int dh = 720;
            int depthDataType = 4;
            int isMjpeg = 0;

            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            printf("Color width:\n");
            scanf("%d", &cw);
            printf("Color height:\n");
            scanf("%d", &ch);
            printf("Depth width:\n");
            scanf("%d", &dw);
            printf("Depth height:\n");
            scanf("%d", &dh);
            printf("Choose FPS:\n");
            scanf("%d", &fps);
            printf("Choose Depth data type\nD(11Bits)=4\nZ(14Bits)=2\nColor Only L'+R'=5\nColor Only L+R=0\n");
            scanf("%d", &depthDataType);

            int ret = APC_Init(&cameraHandle, true);

            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            // IVY2 optical control is in the eSP777 side, so select 1. IVY4 select eSP876 directly.
            DEVINFORMATION info;
            APC_GetDeviceInfo(cameraHandle, &devSelInfo, &info);
            DEVSELINFO devOpticalSensorSelInfo;
            int eSP777Index = 1;
            devOpticalSensorSelInfo.index = info.wPID == APC_PID_IVY2 ? eSP777Index : devSelInfo.index;

            APC_SetDepthDataType(cameraHandle, &devSelInfo, depthDataType);

            APC_SetupBlock(cameraHandle, &devSelInfo, true);

            APC_Setup_v4l2_requestbuffers(cameraHandle, &devSelInfo, 5);

            ret = APC_OpenDevice2(cameraHandle, &devSelInfo, cw, ch, isMjpeg, dw, dh, DEPTH_IMG_NON_TRANSFER, false,
                                  nullptr, &fps);
            if (ret) {
                fprintf(stderr, "Open failed reason %d\n", ret);
                return 0;
            }

            /**
             * Dump 10 images in each stage. 1) Original 2) Set analog gain 3) Set digital gain. 4) Set exposure ms
             */
            static int nSettingUpExposure = 0;
            libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([] (const libeys3dsample::Frame *f) {
                if (nSettingUpExposure < 10) {
                    fprintf(stderr, "Inside color callback frame.sn=%d st=%d adjusted image\n", f->sn, f->status);
                    nSettingUpExposure++;
                    std::string fileName(SAVE_FILE_PATH);
                    fileName.append(std::to_string(f->sn));
                    saveRawFile(fileName.c_str(), (unsigned char*) f->dataVec.data(), f->dataVec.size());
                    fprintf(stderr, "Saved frame.sn=%d in %s --\n", f->sn, fileName.c_str());
                }
                return f->status;
            });

            libeys3dsample::FrameCallbackHelper::Callback depthCallbackFn([] (const libeys3dsample::Frame *f) {
                static bool everEnter = false;
                if (!everEnter) {
                    fprintf(stderr, "...\n");
                    everEnter = true;
                }
                return f->status;
            });

            libeys3dsample::ColorCallbackHelper colorCallback(cameraHandle, devSelInfo, cw, ch, 2, colorCallbackFn);
            libeys3dsample::DepthCallbackHelper depthCallback(cameraHandle, devSelInfo, dw, dh, 2, depthCallbackFn);

            colorCallback.start();
            if (info.wPID == APC_PID_IVY2) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
            depthCallback.start();

            APC_SetCTPropVal(cameraHandle, &devOpticalSensorSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL,
                             AE_MOD_MANUAL_MODE);

            float fAnalogGain = 1.0f;
            float fSetAnalogGain = 1.0f;
            int retAnalogGain = APC_GetAnalogGain(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, &fAnalogGain);
            fprintf(stderr, "APC_GetAnalogGain original value = %f ret=%d\n", fAnalogGain, retAnalogGain);
            fprintf(stderr, "Please Input Analog gain\n");
            scanf("%f", &fSetAnalogGain);
            retAnalogGain = APC_SetAnalogGain(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, fSetAnalogGain);
            fprintf(stderr, "APC_SetAnalogGain set value = %f ret=%d\n", fSetAnalogGain, retAnalogGain);
            retAnalogGain = APC_GetAnalogGain(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, &fAnalogGain);
            fprintf(stderr, "APC_GetAnalogGain value = %f ret=%d \n", fAnalogGain, retAnalogGain);
            nSettingUpExposure = 0;

            float fDigitalGain = 1.0f;
            float fSetDigitalGain = 1.0f;
            int retDigitalGain = APC_GetDigitalGain(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, &fDigitalGain);
            fprintf(stderr, "APC_GetDigitalGain original value = %f ret=%d\n", fDigitalGain, retDigitalGain);
            fprintf(stderr, "Please Input Digital gain\n");
            scanf("%f", &fSetDigitalGain);
            retDigitalGain = APC_SetDigitalGain(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, fSetDigitalGain);
            fprintf(stderr, "APC_SetDigitalGain set value = %f ret=%d\n", fSetDigitalGain, retDigitalGain);
            retDigitalGain = APC_GetDigitalGain(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, &fDigitalGain);
            fprintf(stderr, "APC_GetDigitalGain value = %f ret=%d\n", fDigitalGain, retDigitalGain);
            nSettingUpExposure = 0;

            float fExposureTime = 0.0f;
            float fSetExposureTime = 0.0f;
            int retExposureTime = APC_GetExposureTime(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, &fExposureTime);
            fprintf(stderr, "APC_GetExposureTime original value = %f ret=%d\n", fExposureTime, retExposureTime);
            fprintf(stderr, "Please Input Exposure Time ms\n");
            scanf("%f", &fSetExposureTime);
            retExposureTime = APC_SetExposureTime(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, fSetExposureTime);
            fprintf(stderr, "APC_SetExposureTime set value = %f ret=%d\n", fSetExposureTime, retExposureTime);
            retExposureTime = APC_GetExposureTime(cameraHandle, &devOpticalSensorSelInfo, SENSOR_BOTH, &fExposureTime);
            fprintf(stderr, "APC_GetExposureTime value = %f ret=%d \n", fExposureTime, retExposureTime);
            nSettingUpExposure = 0;

            sleep(5);
            colorCallback.stop();
            depthCallback.stop();

            APC_CloseDevice(cameraHandle, &devSelInfo);
            APC_Setup_v4l2_requestbuffers(cameraHandle, &devSelInfo, 32);
            APC_Release(&cameraHandle);
            break;
        }
        case 94: {
            // YX8083 early sample Integration Requirements:
            // 1. 2560x960 C 1280x960 D 30fps ,depth_data_type=4, ZD_index=0
            // 2. Printing out both camera and IMU FW name.
            // 3. Callback function printing out image serial number and timestamp.
            // 4. Saving C+D and Z image. Including yuy2.yuv and rgb.raw every 5 seconds.
            // 5. Callback Function IMU printing out raw data.
            // 6. Printing out camera intrinsic and extrinsic parameters.
            // 7. Execute the program 30 seconds and release.

            /*
             * This case only verified at YX8083 early sample IMU FW IVY2-D01-ELSM6DSR-BL00U-001-BETA01
             * */

            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;
            int fps = 30;
            int cw = 2560, ch = 960;
            int dw = 1280, dh = 960;
            int depthDataType = 4;
            int isMjpeg = 0;

            int ret = APC_Init(&cameraHandle, false);

            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            printf("Color width:\n");
            scanf("%d", &cw);
            printf("Color height:\n");
            scanf("%d", &ch);
            printf("Depth width:\n");
            scanf("%d", &dw);
            printf("Depth height:\n");
            scanf("%d", &dh);
            printf("Choose FPS:\n");
            scanf("%d", &fps);
            printf("Choose Depth data type\nD(11Bits)=4\nZ(14Bits)=2\nColor Only L'+R'=5\nColor Only L+R=0\n");
            scanf("%d", &depthDataType);

            int deviceCount = APC_GetDeviceNumber(cameraHandle);
            DEVSELINFO printDevSelInfo;
            bool isIVY2 = false;
            for(int i = 0; i < deviceCount; i++) {
                printDevSelInfo.index = i;

                const size_t kFWVersionSize = 256;
                char fwVersion[kFWVersionSize];
                int nActualLength = 0;
                if (!APC_GetFwVersion(cameraHandle, &printDevSelInfo, fwVersion, kFWVersionSize, &nActualLength)) {
                    fprintf(stderr, "Camera firmware version %s\n", fwVersion);
                }

                DEVINFORMATION info;
                if (!APC_GetDeviceInfo(cameraHandle, &printDevSelInfo, &info)) {
                    fprintf(stderr, "Device Name: %s\n", info.strDevName);
                    fprintf(stderr, "Product ID: 0x%04x\n", info.wPID);
                    fprintf(stderr, "Vendor ID: 0x%04x\n", info.wVID);
                    fprintf(stderr, "ChipID: 0x%x\n", info.nChipID);
                    fprintf(stderr, "Device Type: %d\n", info.nDevType);
                    if (info.wPID == APC_PID_IVY2 || info.wPID == APC_PID_IVY2_S) {
                        isIVY2 |= true;
                    }
                }
            }

            APC_SetDepthDataType(cameraHandle, &devSelInfo, 4);

            APC_SetupBlock(cameraHandle, &devSelInfo, true);

            ret = APC_OpenDevice2(cameraHandle, &devSelInfo, cw, ch, false, dw, dh, DEPTH_IMG_NON_TRANSFER, false,
                                  nullptr, &fps);

            if (ret) {
                fprintf(stderr, "Open device failed %d\n", ret);
                return 0;
            }

            std::vector<uint16_t> zdBuffer;
            zdBuffer.resize(APC_ZD_TABLE_FILE_SIZE_11_BITS/2);
            int actualZDLen = 0;
            ZDTABLEINFO zdInfo { .nIndex = 0, .nDataType = APC_DEPTH_DATA_11_BITS};
            ret = APC_GetZDTable(cameraHandle, &devSelInfo, (uint8_t *) zdBuffer.data(),
                                 APC_ZD_TABLE_FILE_SIZE_11_BITS, &actualZDLen, &zdInfo);

            if (ret) fprintf(stderr, "Get ZD Table failed %d\n", ret);

            for (int i = 1; i < APC_ZD_TABLE_FILE_SIZE_11_BITS / 2; ++i) {
                zdBuffer[i] = __bswap_16(zdBuffer[i]);
            }

            eSPCtrl_RectLogData rectifyData;
            ret = APC_GetRectifyMatLogData(cameraHandle, &devSelInfo, &rectifyData, zdInfo.nIndex);
            if (ret) fprintf(stderr, "Get RectifyMatLogData Table failed %d\n", ret);
            else {
                float baseline = 1.0f / rectifyData.ReProjectMat[14];
                float focalLength = rectifyData.ReProjectMat[11];
                float cx = rectifyData.CamMat1[2];
                float fx = rectifyData.CamMat1[0];
                float cy = rectifyData.CamMat1[5];
                float fy = rectifyData.CamMat1[4];
                float fxp = rectifyData.NewCamMat1[0];
                float cxp = rectifyData.NewCamMat1[2];
                float fyp = rectifyData.NewCamMat1[5];
                float cyp = rectifyData.NewCamMat1[6];
                float r0 = rectifyData.LRotaMat[0];
                float r1 = rectifyData.LRotaMat[1];
                float r2 = rectifyData.LRotaMat[2];
                float r3 = rectifyData.LRotaMat[3];
                float r4 = rectifyData.LRotaMat[4];
                float r5 = rectifyData.LRotaMat[5];
                float r6 = rectifyData.LRotaMat[6];
                float r7 = rectifyData.LRotaMat[7];
                float r8 = rectifyData.LRotaMat[8];

                fprintf(stderr, "Rectify Log Data\ncx:[%4f], fx[%4f], cy[%4f], fy[%4f]\n"
                                "cx':[%4f], cy':[%4f], fx':[%4f], fy':[%4f]\n"
                                "rectification transform = [%4f %4f %4f %4f %4f %4f %4f %4f %4f]\n"
                                "baseline:[%f] focal length:[%f]", cx, fx, cy, fy, cxp, cyp, fxp, fyp,
                                r0, r1, r2, r3, r4, r5, r6, r7, r8,
                                baseline, focalLength);
            }

            libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([&] (const libeys3dsample::Frame *f) {
                static uint64_t timeLastSavingTsUs = 0;
                uint64_t currentTsUs = f->ts_s * 1000000 + f->ts_us;
                bool isNeedSavingFile = currentTsUs - timeLastSavingTsUs > 5000000;

                fprintf(stderr, "Inside color callback frame.sn=%d status=%d [ts_s=%ld ts_us=%ld]\n", f->sn, f->status,
                        f->ts_s, f->ts_us);

                if (isNeedSavingFile) {
                    std::string rawFileName(SAVE_FILE_PATH"color_");
                    rawFileName.append(std::to_string(f->sn));
                    if (isMjpeg) {
                        rawFileName.append(".jpg");
                    } else {
                        rawFileName.append(".yuv");
                    }
                    fprintf(stderr, "Saving start... %s\n", rawFileName.c_str());
                    saveRawFile(rawFileName.c_str(), (uint8_t *) f->dataVec.data(), f->dataVec.size());

                    fprintf(stderr, "Saving RGB file...\n");

                    std::string rgbFileName(SAVE_FILE_PATH"rgb_");
                    rgbFileName.append(std::to_string(f->sn));
                    rgbFileName.append(".raw");

                    auto writeRGBFileBuffer = new std::vector<uint8_t>();
                    writeRGBFileBuffer->resize(f->width * f->height * 3);

                    APC_ColorFormat_to_RGB24(cameraHandle, &devSelInfo, writeRGBFileBuffer->data(),
                                             (uint8_t*) f->dataVec.data(), f->actualImageSize, f->width, f->height,
                                             isMjpeg ? APCImageType::COLOR_MJPG : APCImageType::COLOR_YUY2);

                    saveRawFile(rgbFileName.c_str(), (unsigned char*) writeRGBFileBuffer->data(),
                                writeRGBFileBuffer->size());

                    delete writeRGBFileBuffer;
                    timeLastSavingTsUs = currentTsUs;
                    fprintf(stderr, "Saving end... %s %s\n", rawFileName.c_str(), rgbFileName.c_str());
                }

                return f->status;
            });

            libeys3dsample::FrameCallbackHelper::Callback depthCallbackFn([&zdBuffer] (const libeys3dsample::Frame *f) {
                static uint64_t timeLastSavingTsUs = 0;
                uint64_t currentTsUs = f->ts_s * 1000000 + f->ts_us;
                bool isNeedSavingFile = currentTsUs - timeLastSavingTsUs > 5000000;

                fprintf(stderr, "Inside depth callback frame.sn=%d status=%d [ts_s=%ld ts_us=%ld]\n", f->sn, f->status,
                        f->ts_s, f->ts_us);

                if (isNeedSavingFile) {
                    fprintf(stderr, "Saving start... %d\n", f->sn);
                    std::string rawFileName(SAVE_FILE_PATH"DisparityMap_");
                    rawFileName.append(std::to_string(f->sn));
                    rawFileName.append(".raw");
                    saveRawFile(rawFileName.c_str(), (uint8_t *) f->dataVec.data(), f->dataVec.size());

                    fprintf(stderr, "Saving Distance Map...\n");
                    std::string distanceFileName(SAVE_FILE_PATH"Z14_");
                    distanceFileName.append(std::to_string(f->sn));
                    distanceFileName.append(".yuv");

                    auto distanceBuffer = new std::vector<uint16_t>();
                    distanceBuffer->resize(f->width * f->height);

                    auto pD11BufPixelWalker = (uint16_t*) f->dataVec.data();
                    for (int i = 0, maxPixelCnt = f->width * f->height; i < maxPixelCnt; ++i, pD11BufPixelWalker++) {
                        // Verification formula:
                        // Z(mm) = [Focal Len(mm) * Baseline(mm) / Disparity(mm)] * 8
                        // Multiply by 8 due to interpolation from 8 Bits to 11 Bits
                        uint16_t index = *pD11BufPixelWalker;
                        distanceBuffer->at(i) = zdBuffer[index];
                    }

                    saveRawFile(distanceFileName.c_str(), (uint8_t *) distanceBuffer->data(),
                                distanceBuffer->size() * sizeof(uint16_t));

                    timeLastSavingTsUs = currentTsUs;
                    fprintf(stderr, "Saving end... %s:%zu %s:%zu Bytes \n", rawFileName.c_str(), f->dataVec.size(),
                            distanceFileName.c_str(), distanceBuffer->size());
                    delete distanceBuffer;
                }

                return f->status;
            });

            libeys3dsample::ColorCallbackHelper colorCallback(cameraHandle, devSelInfo, cw, ch, 2, colorCallbackFn);
            libeys3dsample::DepthCallbackHelper depthCallback(cameraHandle, devSelInfo, dw, dh, 2, depthCallbackFn);

            colorCallback.start();
            if (isIVY2) {
                sleep(6);
            }
            depthCallback.start();

            if (isIVY2) {
                unsigned short nVID = 0x3438, nPID = 0x0166;

                CIMUModel::INFO info{
                        .nVID = nVID,
                        .nPID = nPID,
                        .axis = CIMUModel::IMU_6_AXIS
                };

                CIMUModel imuModel = CIMUModel(info, cameraHandle);
                std::string imuFWVersion = imuModel.GetFWVersion();
                fprintf(stderr, "GetFWVersion [%s] CMD5-CMD12\n", imuFWVersion.c_str());
                FILE *imuLog = fopen("imulog.txt", "a+");
                // 1. Setup callback
                CIMUModel::Callback printCallback([&](const IMUData *data) {
                    fprintf(stderr,
                            "FrameSN:[%d] Time: %2d,%2d,%2d,%5d\nacc raw:%5f,%5f,%5f\ngyro raw:%5f,%5f,%5f st:%x\n",
                            data->_frameCount, data->_hour, data->_min, data->_sec, data->_subSecond,
                            data->_accelX, data->_accelY, data->_accelZ,
                            data->_gyroScopeX, data->_gyroScopeY, data->_gyroScopeZ, data->_updateReason);
                    fprintf(imuLog,
                            "FrameSN:[%d] Time: %2d,%2d,%2d,%5d\nacc raw:%5f,%5f,%5f\ngyro raw:%5f,%5f,%5f st:%x\n",
                            data->_frameCount, data->_hour, data->_min, data->_sec, data->_subSecond,
                            data->_accelX, data->_accelY, data->_accelZ,
                            data->_gyroScopeX, data->_gyroScopeY, data->_gyroScopeZ, data->_updateReason);
                    return true;
                });

                // 2. Enable callback looper
                imuModel.EnableDataCallback(printCallback);
                sleep(2);
                // 3. Test Time Sync function.
                for (int i = 0; i < 60; ++i) {
                    imuModel.SetTimeSync(2);
                    //Don's set any HID command between Set and get TimeSync.
                    auto TimeSync = imuModel.ReadTimeSync();
                    fprintf(stderr, "ReadTimeSync: %02x %02x %02x %02x %02x %02x %02x %02x CMD47\n", TimeSync.sn,
                            TimeSync.time1, TimeSync.time2, TimeSync.time3, TimeSync.time4, TimeSync.diff1,
                            TimeSync.diff2, TimeSync.diff3);
                    fprintf(imuLog, "ReadTimeSync: %02x %02x %02x %02x %02x %02x %02x %02x CMD47\n", TimeSync.sn,
                            TimeSync.time1, TimeSync.time2, TimeSync.time3, TimeSync.time4, TimeSync.diff1,
                            TimeSync.diff2, TimeSync.diff3);
                    auto rtc = imuModel.ReadRTC();
                    fprintf(stderr, "GetRTC %d %d %d %d CMD40\n", rtc.hour, rtc.min, rtc.sec, rtc.subSecond);
                    fprintf(imuLog, "GetRTC %d %d %d %d CMD40\n", rtc.hour, rtc.min, rtc.sec, rtc.subSecond);
                    sleep(1);
                }

                // 3. Disable callback looper after use.
                imuModel.DisableDataCallback();
            }

            colorCallback.stop();
            depthCallback.stop();

            APC_CloseDevice(cameraHandle, &devSelInfo);
            APC_Release(&cameraHandle);
            break;
        }

        case 95: {
            void* cameraHandle = nullptr;

            int ret = APC_Init(&cameraHandle, false);
            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            DEVINFORMATION devInfo;
            DEVSELINFO devselinfo;
            devselinfo.index = 0;
            ret = APC_GetDeviceInfo(cameraHandle, &devselinfo, &devInfo);

            if (devInfo.wPID == APC_PID_IVY2) {
                unsigned short nVID = 0x3438, nPID = 0x0166;
                CIMUModel::INFO info {
                        .nVID = nVID,
                        .nPID = nPID,
                        .axis = CIMUModel::IMU_6_AXIS
                };

                CIMUModel imuModel = CIMUModel(info, cameraHandle);
                imuModel.RebootBootloader();
            }

            APC_Release(&cameraHandle);
            break;
        }

        case 96: {
            void* cameraHandle = nullptr;

            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;

            int fps = 30;
            int colorOption = 0;
            int depthOption = 0;
            int depthSwitch = 0x0;
            int depthDataType = 4;
            int isMjpeg = 0;

            int ret = APC_Init(&cameraHandle, false);

            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            int deviceCount = APC_GetDeviceNumber(cameraHandle);
            DEVSELINFO printDevSelInfo;

            for(int i = 0; i < deviceCount; i++) {
                printDevSelInfo.index = i;
                DEVINFORMATIONEX info;

                constexpr int APC_MAX_STREAM_COUNT = 64;
                APC_STREAM_INFO ep0StreamInfo[APC_MAX_STREAM_COUNT] = {{0}};
                APC_STREAM_INFO ep1StreamInfo[APC_MAX_STREAM_COUNT] = {{0}};
                int ep0StreamOptionCount;
                int ep1StreamOptionCount;

                auto getResolution = [&] (APC_STREAM_INFO* pStreamRes0, int* pStreamResCount0,
                                          APC_STREAM_INFO* pStreamRes1, int* pStreamResCount1) {

                    int ret = APC_GetDeviceResolutionList(cameraHandle, &printDevSelInfo,
                                                          APC_MAX_STREAM_COUNT, pStreamRes0,
                                                          APC_MAX_STREAM_COUNT, pStreamRes1);
                    int ep0Cnt = 0;
                    for (ep0Cnt = 0; ep0Cnt < APC_MAX_STREAM_COUNT; ep0Cnt++) {
                        if (pStreamRes0[ep0Cnt].nHeight <= 0 || pStreamRes0[ep0Cnt].nWidth <= 0) {
                            break;
                        }
                    }
                    *pStreamResCount0 = ep0Cnt;

                    int ep1Cnt = 0;
                    for (ep1Cnt = 0; ep1Cnt < APC_MAX_STREAM_COUNT; ep1Cnt++) {
                        if (pStreamRes1[ep1Cnt].nHeight <= 0 || pStreamRes1[ep1Cnt].nWidth <= 0) {
                            break;
                        }
                    }
                    *pStreamResCount1 = ep1Cnt;
                };

                getResolution(ep0StreamInfo, &ep0StreamOptionCount, ep1StreamInfo, &ep1StreamOptionCount);

                for (unsigned short resCount0 = 0; resCount0 < ep0StreamOptionCount; resCount0++) {
                    printf("EP0 [%d] %d x %d, %s\n", resCount0, ep0StreamInfo[resCount0].nWidth,
                           ep0StreamInfo[resCount0].nHeight, ep0StreamInfo[resCount0].bFormatMJPG ? "MJPEG" : "YUYV");
                }

                for (unsigned short resCount1 = 0; resCount1 < ep1StreamOptionCount; resCount1++) {
                    printf("EP1 [%d] %d x %d, %s\n", resCount1, ep1StreamInfo[resCount1].nWidth,
                           ep1StreamInfo[resCount1].nHeight, ep1StreamInfo[resCount1].bFormatMJPG ? "MJPEG" : "YUYV");
                }

                if (!APC_GetDeviceInfoEx(cameraHandle, &printDevSelInfo, &info)) {
                    fprintf(stderr, "Product ID: 0x%04x\n", info.wPID);
                    fprintf(stderr, "Vendor ID:  0x%04x\n", info.wVID);
                    fprintf(stderr, "Device Name: %s\n", info.strDevName);
                    fprintf(stderr, "strDevPath: %s \n", info.strDevPath);
                    fprintf(stderr, "ChipID: 0x%x\n", info.nChipID);
                    fprintf(stderr, "Device Type: %d\n", info.nDevType);
                    fprintf(stderr, "wUsbNode: %x \n", info.wUsbNode);
                }
            }

            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            printf("Color Option:\n");
            scanf("%d", &colorOption);
            printf("Depth Option:\n");
            scanf("%d", &depthOption);
            printf("Choose FPS:\n");
            scanf("%d", &fps);
            printf("Choose Depth Data Type\n");
            scanf("%d", &depthDataType);
            printf("Depth Data Type %d\n", depthDataType);

            ret = APC_Init(&cameraHandle, false);
            fprintf(stderr, "APC_Init reason %d\n", ret);
            ret = APC_SetDepthDataType(cameraHandle, &devSelInfo, depthDataType);
            fprintf(stderr, "APC_SetDepthDataType reason %d\n", ret);
            /** Interleave case */
            int interleaveEnable = 0;
            printf("Choose Interleave enable (1 or 0)\n");
            scanf("%d", &interleaveEnable);
            printf("Interleave on %d ? (Need provide supported HW && Resolution)\n", interleaveEnable);
            ret = APC_SetInterleaveMode(cameraHandle, &devSelInfo, interleaveEnable);
            fprintf(stderr, "APC_SetInterleaveMode reason %d\n", ret);

            /** Common case close immediate */
            ret = APC_OpenDeviceCallback(cameraHandle, &devSelInfo, colorOption, depthOption, depthSwitch, fps,
                                         SimpleColorDepthCallback, nullptr, APC_PID_IVY2);
            fprintf(stderr, "APC_OpenDeviceCallback 01 open close reason %d\n", ret);
            ret = APC_CloseDevice(cameraHandle, &devSelInfo);
            fprintf(stderr, "APC_CloseDevice 01 reason %d\n", ret);

            /** Common case close 5s */
            ret = APC_OpenDeviceCallback(cameraHandle, &devSelInfo, colorOption, depthOption, depthSwitch, fps,
                                         SimpleColorDepthCallback, nullptr, APC_PID_IVY2);
            fprintf(stderr, "APC_OpenDeviceCallback 02 open 5s close reason %d\n", ret);
            std::this_thread::sleep_for(std::chrono::seconds(5));
            ret = APC_CloseDevice(cameraHandle, &devSelInfo);
            fprintf(stderr, "APC_CloseDevice reason 02 %d\n", ret);

            /** Double Open */
            for (int k = 0; k < 10; k++) {
                ret = APC_OpenDeviceCallback(cameraHandle, &devSelInfo, colorOption, depthOption, depthSwitch, fps,
                                             SimpleColorDepthCallback, nullptr, APC_PID_IVY2);
                fprintf(stderr, "APC_OpenDeviceCallback 03 double open reason %d\n", ret);
                std::this_thread::sleep_for(std::chrono::milliseconds(random() % 2000));

                ret = APC_OpenDeviceCallback(cameraHandle, &devSelInfo, colorOption, depthOption, depthSwitch, fps,
                                             SimpleColorDepthCallback, nullptr, APC_PID_IVY2);
                fprintf(stderr, "APC_OpenDeviceCallback 03 double2 reason %d\n", ret);
                std::this_thread::sleep_for(std::chrono::seconds(10));
                ret = APC_CloseDevice(cameraHandle, &devSelInfo);
                fprintf(stderr, "APC_CloseDevice 03 reason %d\n", ret);
            }

            /** Double Close */
            ret = APC_CloseDevice(cameraHandle, &devSelInfo);
            fprintf(stderr, "APC_CloseDevice 04 reason %d\n", ret);

            /** Null callback function pointer */
            ret = APC_OpenDeviceCallback(cameraHandle, &devSelInfo, colorOption, depthOption, depthSwitch, fps,
                                         nullptr, nullptr, APC_PID_IVY2);
            fprintf(stderr, "APC_OpenDeviceCallback 05 NULL open then... close reason %d\n", ret);
            std::this_thread::sleep_for(std::chrono::seconds(5));
            ret = APC_CloseDevice(cameraHandle, &devSelInfo);
            fprintf(stderr, "APC_CloseDevice 05 reason %d\n", ret);

            APC_Release(&cameraHandle);
            break;
        }
        case 97: {
            // IVY2 + IVY3 + IMU / IVY4 stream together case.
            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;

            /* IVY2 resolution parameters */
            int fps = 30;
            int cw = 2560, ch = 960;
            int dw = 1280, dh = 960;
            int depthDataType = APC_DEPTH_DATA_11_BITS;
            int isMjpeg = false;
            /* IVY3 resolution parameters*/
            int ivy3_cw = 640, ivy3_ch = 480;
            int ivy3_fps = 30;
            int ivy3_is_mjpeg = false;
            int ivy3_depthDataType = APC_DEPTH_DATA_OFF_RECTIFY;
            int ret = APC_Init(&cameraHandle, false);

            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            fprintf(stderr, "Case 97 IVY2 IVY3 IMU / IVY4 stream on test.\n");
            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            fprintf(stderr, "Color width:\n");
            scanf("%d", &cw);
            fprintf(stderr,"Color height:\n");
            scanf("%d", &ch);
            fprintf(stderr,"Depth width:\n");
            scanf("%d", &dw);
            fprintf(stderr,"Depth height:\n");
            scanf("%d", &dh);
            fprintf(stderr, "FPS?\n");
            scanf("%d", &fps);
            fprintf(stderr,"Choose IVY2 Depth Data Type D11 set 4 Z14 set 2\n");
            scanf("%d", &depthDataType);
            fprintf(stderr,"Depth Data Type %d\n", depthDataType);

            fprintf(stderr,"IVY3 Color width:\n");
            scanf("%d", &ivy3_cw);
            fprintf(stderr,"IVY3 Color height:\n");
            scanf("%d", &ivy3_ch);
            fprintf(stderr, "IVY3 FPS?\n");
            scanf("%d", &ivy3_fps);
            fprintf(stderr, "IVY3 YUYV set 0 or MJPEG set 1:\n");
            scanf("%d", &ivy3_is_mjpeg);
            fprintf(stderr,"IVY3 Depth Data Type K(No Dewarp) set 0 K'(Dewarp) set 5\n");
            scanf("%d", &ivy3_depthDataType);
            fprintf(stderr,"IVY3 Depth Data Type %d\n", ivy3_depthDataType);

            int sec = 30;
            fprintf(stderr, "How many seconds?\n");
            scanf("%d", &sec);
            fprintf(stderr, "IVY3 Image FMT %d %s \n", ivy3_is_mjpeg, ivy3_is_mjpeg ? "MJPEG" : "YUYV");
            fprintf(stderr, "Running case 97 for %d secs. With FPS %d, %d\n", sec, fps, ivy3_fps);

            int deviceCount = APC_GetDeviceNumber(cameraHandle);
            DEVSELINFO printDevSelInfo;
            int ivy2DeviceIndex = -1;
            int ivy3DeviceIndex = -1;
            int ivy4DeviceIndex = -1;
            for(int i = 0; i < deviceCount; i++) {
                printDevSelInfo.index = i;

                const size_t kFWVersionSize = 256;
                char fwVersion[kFWVersionSize];
                int nActualLength = 0;
                if (!APC_GetFwVersion(cameraHandle, &printDevSelInfo, fwVersion, kFWVersionSize, &nActualLength)) {
                    fprintf(stderr, "Camera firmware version %s\n", fwVersion);
                }

                DEVINFORMATION info;
                if (!APC_GetDeviceInfo(cameraHandle, &printDevSelInfo, &info)) {
                    fprintf(stderr, "Device Name: %s\n", info.strDevName);
                    fprintf(stderr, "Product ID: 0x%04x\n", info.wPID);
                    fprintf(stderr, "Vendor ID: 0x%04x\n", info.wVID);
                    fprintf(stderr, "ChipID: 0x%x\n", info.nChipID);
                    fprintf(stderr, "Device Type: %d\n", info.nDevType);
                    if (info.wPID == APC_PID_IVY3) {
                        ivy3DeviceIndex = i;
                    } else if (info.wPID == APC_PID_IVY2) {
                        ivy2DeviceIndex = i;
                    } else if (info.wPID == APC_PID_IVY4) {
                        ivy4DeviceIndex = i;
                    }
                }
            }

            bool isIVY2 = ivy2DeviceIndex != -1;
            bool isIVY3 = ivy3DeviceIndex != -1;
            bool isIVY4 = ivy4DeviceIndex != -1;

            libeys3dsample::ColorCallbackHelper* colorCallback = nullptr;
            libeys3dsample::DepthCallbackHelper* depthCallback = nullptr;
            int dropColor = 0;
            int dropDepth = 0;
            int ivy2TotalColorFrames = 0;
            int ivy2TotalDepthFrames = 0;
            if (isIVY2 || isIVY4) {
                devSelInfo.index = isIVY4 ? ivy4DeviceIndex : ivy2DeviceIndex;
                APC_SetDepthDataType(cameraHandle, &devSelInfo, depthDataType);

                APC_SetupBlock(cameraHandle, &devSelInfo, true);
                ret = APC_OpenDevice2(cameraHandle, &devSelInfo, cw, ch, isMjpeg, dw, dh, DEPTH_IMG_NON_TRANSFER, false,
                                      nullptr, &fps);

                if (ret) {
                    fprintf(stderr, "Open device failed %d\n", ret);
                    APC_Release(&cameraHandle);
                    return 0;
                }

                libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([&] (const libeys3dsample::Frame *f) {
                    static int64_t lastTimeUs = 0;
                    static int lastSN = 0;
                    int64_t curTimeUs = f->ts_s * 1000000 + f->ts_us;
                    if (f->sn - lastSN > 1) {
                        fprintf(stderr, "[DropFrame] Inside color callback frame.sn=%d lastSN=%d status=%d diffMs=%f\n",
                                f->sn, lastSN, f->status, (curTimeUs - lastTimeUs) / 1000.0f);
                        dropColor++;
                    }
                    lastTimeUs = curTimeUs;
                    lastSN = f->sn;
                    ivy2TotalColorFrames++;
                    return f->status;
                });

                libeys3dsample::FrameCallbackHelper::Callback depthCallbackFn([&] (const libeys3dsample::Frame *f) {
                    static int64_t lastTimeUs = 0;
                    static int lastSN = 0;
                    int64_t curTimeUs = f->ts_s * 1000000 + f->ts_us;
                    if (f->sn - lastSN > 1) {
                        fprintf(stderr, "[DropFrame] Inside depth callback frame.sn=%d lastSN=%d status=%d diffMs=%f\n",
                                f->sn, lastSN, f->status, (curTimeUs - lastTimeUs) / 1000.0f);
                        dropDepth++;
                    }
                    lastTimeUs = curTimeUs;
                    lastSN = f->sn;
                    ivy2TotalDepthFrames++;
                    return f->status;
                });

                colorCallback = new libeys3dsample::ColorCallbackHelper(cameraHandle, devSelInfo, cw, ch, 2,
                                                                        colorCallbackFn);
                depthCallback = new libeys3dsample::DepthCallbackHelper(cameraHandle, devSelInfo, dw, dh, 2,
                                                                        depthCallbackFn);
                colorCallback->start();
                if(isIVY2) {
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
                depthCallback->start();

                // Registers should be set after depth stream on due to firmware will set default value when preview.
                // RegisterSettings::DM_Quality_Register_Setting(cameraHandle, &devSelInfo, 0x191);
            }

            libeys3dsample::ColorCallbackHelper* ivy3ColorCallback = nullptr;
            int ivy3DropColor = 0;
            int ivy3TotalFrames = 0;
            if (isIVY3) {
                devSelInfo.index = ivy3DeviceIndex;
                APC_SetDepthDataType(cameraHandle, &devSelInfo, ivy3_depthDataType);
                APC_SetupBlock(cameraHandle, &devSelInfo, true);

                ret = APC_OpenDevice2(cameraHandle, &devSelInfo, ivy3_cw, ivy3_ch, ivy3_is_mjpeg, 0, 0,
                                      DEPTH_IMG_NON_TRANSFER, false, nullptr, &ivy3_fps);

                if (ret) {
                    fprintf(stderr, "IVY3 Open device failed %d\n", ret);
                    if (ivy2DeviceIndex == -1) {
                        APC_Release(&cameraHandle);
                        return 0;
                    }
                    // Otherwise release later
                }

                int acceptableOffsetUs = 5000;
                int frameDropThresholdUs = (1 / (double) ivy3_fps) * 1000000 + 5000;

                fprintf(stderr, "IVY3 Color Rectify Mode Set (%d) and drop threshold = %d\n", ivy3_depthDataType,
                       frameDropThresholdUs);

                int64_t ivy3LastTimeUs = 0ul;
                libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([&] (const libeys3dsample::Frame *f) {
                    ivy3LastTimeUs = 0;
                    int64_t curTimeUs = f->ts_s * 1000000 + f->ts_us;
                    if (ivy3LastTimeUs && curTimeUs - ivy3LastTimeUs > frameDropThresholdUs) {
                        fprintf(stderr, "[DropFrame] Inside IVY3 color callback status=%d diffMs=%f\n", f->status,
                                (curTimeUs - ivy3LastTimeUs) / 1000.0f);
                        ivy3DropColor++;
                    }
                    ivy3LastTimeUs = curTimeUs;
                    ivy3TotalFrames++;
                    return f->status;
                });
                ivy3ColorCallback = new libeys3dsample::ColorCallbackHelper(cameraHandle, devSelInfo, ivy3_cw, ivy3_ch,
                                                                            2, colorCallbackFn);
                ivy3ColorCallback->start();
            }

            CIMUModel* pImuModel = nullptr;
            if (isIVY2) {
                unsigned short nVID = 0x3438, nPID = 0x0166;
                CIMUModel::INFO info {
                        .nVID = nVID,
                        .nPID = nPID,
                        .axis = CIMUModel::IMU_6_AXIS
                };

                pImuModel = new CIMUModel(info, cameraHandle);
                std::string imuFWVersion = pImuModel->GetFWVersion();
                fprintf(stderr, "IMU GetFWVersion [%s] CMD5-CMD12\n", imuFWVersion.c_str());
                bool isIMUWork = false;

                CIMUModel::Callback printCallback([&] (const IMUData *data) {
                    if (!isIMUWork) {
                        fprintf(stderr, "IMU work !!\n");
                        isIMUWork = true;
                    }
                    return true;
                });

                pImuModel->EnableDataCallback(printCallback);
            }

            sleep(sec);
            fprintf(stderr, "Times up!!\n");

            if (isIVY2) {
                if (pImuModel) {
                    pImuModel->DisableDataCallback();
                    pImuModel = nullptr;
                }
            }

            if (colorCallback) {
                colorCallback->stop();
                colorCallback = nullptr;
                fprintf(stderr, "Color End !!\n");
            }

            if (depthCallback) {
                depthCallback->stop();
                depthCallback = nullptr;
                fprintf(stderr, "Depth End !!\n");
            }

            if (ivy3ColorCallback) {
                ivy3ColorCallback->stop();
                ivy3ColorCallback = nullptr;
                fprintf(stderr, "IVY3 Color End !!\n");
            }

            fprintf(stderr, "Total Color %d, Depth %d, IVY3 %d frames / Drop Color %d Depth: %d IVY3 %d\n",
                    ivy2TotalColorFrames, ivy2TotalDepthFrames, ivy3TotalFrames, dropColor, dropDepth, ivy3DropColor);

            if (ivy2DeviceIndex >= 0) {
                devSelInfo.index = ivy2DeviceIndex;
                APC_CloseDevice(cameraHandle, &devSelInfo);
            }

            if (ivy3DeviceIndex >= 0) {
                devSelInfo.index = ivy3DeviceIndex;
                APC_CloseDevice(cameraHandle, &devSelInfo);
            }

            if (ivy4DeviceIndex >= 0) {
                devSelInfo.index = ivy4DeviceIndex;
                APC_CloseDevice(cameraHandle, &devSelInfo);
            }

            fprintf(stderr, "Close End !!\n");

            APC_Release(&cameraHandle);
            fprintf(stderr, "Release End !!\n");
            sleep(10); // For multiple test purpose APC_Release will reset eSP777
            break;
        }
        case 98: {
            // [IVY4 TOOL 1]
            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;

            int fps = 15;
            int cw = 1280; int ch = 960;
            int dw = 0; int dh = 0;
            int depthDataType = 4;
            int isMjpeg = 0;

            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            printf("Sensor output Color image width IVY4 is 3200:\n");
            scanf("%d", &cw);
            printf("Sensor output Color image height IVY4 is 1200:\n");
            scanf("%d", &ch);
            printf("Choose FPS:\n");
            scanf("%d", &fps);
            printf("Choose Depth data type \nColor Only L+R=0\n");
            scanf("%d", &depthDataType);
            int sec = 30;
            fprintf(stderr, "How many seconds?\n");
            scanf("%d", &sec);

            int ret = APC_Init(&cameraHandle, true);

            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            APC_SetDepthDataType(cameraHandle, &devSelInfo, depthDataType);

            APC_SetupBlock(cameraHandle, &devSelInfo, true);

            ret = APC_OpenDevice2(cameraHandle, &devSelInfo, cw, ch, isMjpeg, dw, dh, DEPTH_IMG_NON_TRANSFER, false,
                                  nullptr, &fps);
            if (ret) {
                fprintf(stderr, "Open failed reason %d\n", ret);
                return 0;
            }

            bool isStreamOn = false;
            bool isISPEnabled = true; /* Default the image pass ISP block */

            std::map<uint16_t, uint16_t> mapBackupRegister;
            mapBackupRegister[0xf1af] = 0x0;
            mapBackupRegister[0xf1e0] = 0x0;

            libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([&] (const libeys3dsample::Frame *f) {
                fprintf(stderr, "Inside color callback frame.sn=%d status=%d save one image \n", f->sn, f->status);
                isStreamOn = true;
                if (isISPEnabled && f->sn == 30) {
                    isISPEnabled = false;
                    fprintf(stderr, "Step2 After frame.sn=%d ** ISP Disabled \n", f->sn);

                    // P1_RAW_SEL set Bit1 as 1
                    uint16_t p1RawSel = mapBackupRegister[0xf1af] | 0x02;
                    p1RawSel &= ~0x08;
                    ret = APC_SetHWRegister(cameraHandle, &devSelInfo, 0xf1af, p1RawSel,
                                            FG_Address_2Byte | FG_Value_1Byte);
                    if (ret) fprintf(stderr, "Can't set %x", 0xf1af);

                    // RAW82_ENB set Bit0 as 1
                    uint16_t raw82Enb = mapBackupRegister[0xf1e0] | 0x01;
                    ret = APC_SetHWRegister(cameraHandle, &devSelInfo, 0xf1e0, raw82Enb,
                                            FG_Address_2Byte | FG_Value_1Byte);
                    if (ret) fprintf(stderr, "Can't set %x", 0xf1e0);

                    uint16_t readValue = 0x0;
                    for (auto item: mapBackupRegister) {
                        ret = APC_GetHWRegister(cameraHandle, &devSelInfo, item.first, &readValue,
                                                FG_Address_2Byte | FG_Value_1Byte);
                        if (ret) fprintf(stderr, "Can't get %x", item.first);
                        fprintf(stderr, "Step2 Case 98 check registers again %x %x\n", item.first, readValue);
                    }
                }

                std::string fileName = SAVE_FILE_PATH;
                fileName.append(std::to_string(f->sn));
                saveRawFile(fileName.c_str(), (unsigned char*) f->dataVec.data(), f->dataVec.size());
                fprintf(stderr, "Saved frame.sn=%d --\n", f->sn);

                return f->status;
            });

            libeys3dsample::ColorCallbackHelper colorCallback(cameraHandle, devSelInfo, cw, ch, 2, colorCallbackFn);
            colorCallback.start();

            while (!isStreamOn) {
                sleep(1);
                fprintf(stderr, "Wait for stream on\n");
            }

            auto switchISPFunctions = [&] (bool isAEEnabled, bool isAWBEnabled) {
                ret = APC_SetCTPropVal(cameraHandle, &devSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL,
                                       isAEEnabled ? AE_MOD_APERTURE_PRIORITY_MODE : AE_MOD_MANUAL_MODE);
                if (ret) fprintf(stderr, "AE can't be switched %d\n", ret);

                ret = APC_SetPUPropVal(cameraHandle, &devSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL,
                                       isAWBEnabled ? PU_PROPERTY_ID_AWB_ENABLE : PU_PROPERTY_ID_AWB_DISABLE);
                if (ret) fprintf(stderr, "AWB can't be switched %d\n", ret);
            };

            // Disable the AE AWB. These function will control ISP related registers in the firmware.
            switchISPFunctions(false, false);

            // Backup the values of ISP related registers.
            for (auto item: mapBackupRegister) {
                ret = APC_GetHWRegister(cameraHandle, &devSelInfo, item.first, &item.second,
                                        FG_Address_2Byte | FG_Value_1Byte);
                if (ret) fprintf(stderr, "Can't get %x", item.first);
                fprintf(stderr, "Step1 Case 98 Backup registers %x %x\n", item.first, item.second);
            }

            sleep(sec);

            // Restore the values of ISP related registers.
            for (auto item: mapBackupRegister) {
                ret = APC_SetHWRegister(cameraHandle, &devSelInfo, item.first, item.second,
                                        FG_Address_2Byte | FG_Value_1Byte);
            }

            // Enable the AE AWB.
            switchISPFunctions(true, true);

            colorCallback.stop();

            APC_CloseDevice(cameraHandle, &devSelInfo);
            APC_Release(&cameraHandle);
            break;
        }
        case 99 : {
            // [IVY4 TOOL 1]
            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;

            int fps = 15;
            int cw = 1280; int ch = 960;
            int dw = 0; int dh = 0;
            int depthDataType = 4;
            int isMjpeg = 0;
            int i2cSelector = 0;
            int i2cSlaveAddress = 0; /* Now we had 2 PCBs, and their addresses are different. */

            printf("I2C slave address 0x6C:0 0x20:1\n");
            scanf("%d", &i2cSelector);
            i2cSlaveAddress = i2cSelector == 0 ? 0x6C : 0x20;
            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            printf("Color width:\n");
            scanf("%d", &cw);
            printf("Color height:\n");
            scanf("%d", &ch);
            printf("Choose FPS:\n");
            scanf("%d", &fps);
            printf("Choose Depth data type\nD(11Bits)=4\nZ(14Bits)=2\nColor Only L'+R'=5\nColor Only L+R=0\n");
            scanf("%d", &depthDataType);
            int sec = 30;
            fprintf(stderr, "How many seconds?\n");
            scanf("%d", &sec);

            int ret = APC_Init(&cameraHandle, true);

            if (!cameraHandle) {
                fprintf(stderr, "Null handle reason %d\n", ret);
                return 0;
            }

            APC_SetupBlock(cameraHandle, &devSelInfo, true);

            gJsonFileSerializer = new JsonFileSerializer();
            JsonFileSerializer::createFolder(JsonFileSerializer::makeStorePath(std::string(SAVE_FILE_PATH)));
            /// case99_static
            static uint16_t sValue320e = 0x0;
            static uint16_t sValue320f = 0x0;
            static int16_t sFPS = 0;
            sFPS = fps;
            auto metaDataCallbackFunc = [] (APC_META_DATA data) {
                MetaDataPayloadParser::ProtoZeroData parsedData;

                int ret = MetaDataPayloadParser::parseProtocolZeroData(&data, &parsedData);
                if (ret) {
                    fprintf(stderr, "[eys3d_meta] Meta data wrong %d\n", ret);
                    return;
                }

                std::string fullPath;
                fullPath.append(SAVE_FILE_PATH);
                fullPath.append(std::to_string(parsedData.frameCount));
                fullPath.append("_");
                auto currentTime = std::chrono::system_clock::now().time_since_epoch();
                auto unixTimeMillis = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime).count();
                fullPath.append(std::to_string(unixTimeMillis));
                fullPath.append(JsonFileSerializer::kJsonFileSuffix);

                gJsonFileSerializer->cacheIn(JsonFileSerializer::kKeyFrameCount, parsedData.frameCount);

                uint8_t analogGainValue = MetaDataPayloadParser::GetAnalogGainMappingValue(parsedData.analogGain);
                gJsonFileSerializer->cacheIn(JsonFileSerializer::kKeyAnalogGain,analogGainValue);

                double digitalGainValue = MetaDataPayloadParser::GetDigitalGainMappingValue(parsedData.digitalGain);
                gJsonFileSerializer->cacheIn(JsonFileSerializer::kKeyDigitalGain,digitalGainValue);

                /* 8051 not suitable for long division calculate in the application layer */
                uint16_t frameLength = (sValue320e & 0x7Fu) << 8u;
                frameLength |= sValue320f;
                double exposureLineTimeMs = 1000.0 * (1.0 / (sFPS * frameLength));
                auto exposureTimeMs = parsedData.exposureRows * exposureLineTimeMs;
                if (!frameLength || !sFPS) {
                    gJsonFileSerializer->cacheIn(JsonFileSerializer::kKeyExposureTimeMs, "NotReady");
                } else {
                    gJsonFileSerializer->cacheIn(JsonFileSerializer::kKeyExposureTimeMs, exposureTimeMs);
                }

                gJsonFileSerializer->cacheIn(JsonFileSerializer::kKeyLuminousMean, parsedData.yAverage);
                gJsonFileSerializer->cacheIn(JsonFileSerializer::kKeyOtherData, parsedData.otherData);
                gJsonFileSerializer->serializeToFile(fullPath);

                fprintf(stderr, "[eys3d_meta_mapped] PV:%02x FC:%d AG:%dX DG:%fX EXP:%f yAVG:%d OT:%02x FPS:%d FL:%d\n",
                        parsedData.protocolVersion, parsedData.frameCount, analogGainValue, digitalGainValue,
                        exposureTimeMs, parsedData.yAverage, parsedData.otherData, sFPS, frameLength);
                fprintf(stderr, "\n");
            };

            APC_MetaDataCallbackFn callback = metaDataCallbackFunc;
            // Case 0: Normal callback
            fprintf(stderr, "[Case99] Case0 ++\n");

            APC_SetMetaDataEventState(cameraHandle, &devSelInfo, true, callback);

            APC_SetDepthDataType(cameraHandle, &devSelInfo, depthDataType);

            ret = APC_OpenDevice2(cameraHandle, &devSelInfo, cw, ch, isMjpeg, dw, dh, DEPTH_IMG_NON_TRANSFER, false,
                                  nullptr, &fps);
            if (ret) {
                fprintf(stderr, "Open failed reason %d\n", ret);
                return 0;
            }

            // case99_static
            static bool sFirstFrame = false;
            libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([&] (const libeys3dsample::Frame *f) {
                int ret = APC_OK;
                uint16_t value3e01 = 0x0;
                uint16_t value3e02 = 0x0;
                if (!sFirstFrame) {
                    ret = APC_GetSensorRegister(cameraHandle, &devSelInfo, i2cSlaveAddress, 0x320e, &sValue320e,
                                                FG_Address_2Byte | FG_Value_1Byte, SENSOR_BOTH);

                    ret = APC_GetSensorRegister(cameraHandle, &devSelInfo, i2cSlaveAddress, 0x320f, &sValue320f,
                                                FG_Address_2Byte | FG_Value_1Byte, SENSOR_BOTH);
                    fprintf(stderr, "Frame Length %x %x = %d\n", sValue320e, sValue320f,
                            ((sValue320e & 0x7Fu) << 8u) | sValue320f);
                    sFirstFrame = true;
                    uint16_t exposureRows = (value3e01 & 0xFFu) << 4u | (value3e02 & 0xF0u) >> 4u;
                    fprintf(stderr, "Exposure Rows %02x %02x = %d\n", value3e01, value3e02, exposureRows);
                }

                uint16_t value3e07 = 0x0;
                ret = APC_GetSensorRegister(cameraHandle, &devSelInfo, i2cSlaveAddress, 0x3e07, &value3e07,
                                            FG_Address_2Byte | FG_Value_1Byte, SENSOR_BOTH);
                if (ret) fprintf(stderr, "0x3e07 failed");

                uint16_t value3e06 = 0x0;
                ret = APC_GetSensorRegister(cameraHandle, &devSelInfo, i2cSlaveAddress, 0x3e06, &value3e06,
                                            FG_Address_2Byte | FG_Value_1Byte, SENSOR_BOTH);
                if (ret) fprintf(stderr, "0x3e06 failed");

                uint16_t value3e09 = 0x0;
                ret = APC_GetSensorRegister(cameraHandle, &devSelInfo, i2cSlaveAddress, 0x3e09, &value3e09,
                                            FG_Address_2Byte | FG_Value_1Byte, SENSOR_BOTH);
                if (ret) fprintf(stderr, "0x3e09 failed");

                ret = APC_GetSensorRegister(cameraHandle, &devSelInfo, i2cSlaveAddress, 0x3e01, &value3e01,
                                            FG_Address_2Byte | FG_Value_1Byte, SENSOR_BOTH);
                if (ret) fprintf(stderr, "0x3e01 failed");

                ret = APC_GetSensorRegister(cameraHandle, &devSelInfo, i2cSlaveAddress, 0x3e02, &value3e02,
                                            FG_Address_2Byte | FG_Value_1Byte, SENSOR_BOTH);
                if (ret) fprintf(stderr, "0x3e02 failed");

                fprintf(stderr, "Inside callback f.sn=%d st=%d 3e06=%04x 3e07=%04x 3e09=%04x 320e=%04x 320f=%04x\n",
                        f->sn, f->status, value3e06, value3e07, value3e09, sValue320e, sValue320f);

                std::string fileName = SAVE_FILE_PATH;
                fileName.append(std::to_string(f->sn));
                saveRawFile(fileName.c_str(), (unsigned char*) f->dataVec.data(), f->dataVec.size());
                fprintf(stderr, "Saved frame.sn=%d --\n", f->sn);

                return f->status;
            });

            libeys3dsample::ColorCallbackHelper colorCallback(cameraHandle, devSelInfo, cw, ch, 2, colorCallbackFn);

            colorCallback.start();
            sleep(sec);
            APC_SetMetaDataEventState(cameraHandle, &devSelInfo, false);
            fprintf(stderr, "[Case99] Case0 --\n");
            sleep(2);

            // Case 1: Without callback function.
            fprintf(stderr, "[Case99] Case1 ++\n");
            APC_SetMetaDataEventState(cameraHandle, &devSelInfo, true, nullptr);
            sleep(2);
            APC_SetMetaDataEventState(cameraHandle, &devSelInfo, false);
            fprintf(stderr, "[Case99] Case1 --\n");
            sleep(2);

            // Case 2 Switch fastly
            fprintf(stderr, "[Case99] Case2 ++\n");
            for (int i = 0; i < 1000; i++) {
                long randomNum = random();
                long randomSleepUs = (randomNum % 4) * (1000 * 1000 / fps);
                fprintf(stderr, "[Case99] Case2 run %d sleep:%ldUs**\n", i, randomSleepUs);
                APC_SetMetaDataEventState(cameraHandle, &devSelInfo, true, callback);
                usleep(randomSleepUs);
                APC_SetMetaDataEventState(cameraHandle, &devSelInfo, false);
                fprintf(stderr, "[Case99] Case2 run %d .. \n", i);
            }
            fprintf(stderr, "[Case99] Case2 --\n");
            sleep(2);
            colorCallback.stop();

            // Case 3: Double close
            fprintf(stderr, "[Case99] Case3 ++\n");
            APC_SetMetaDataEventState(cameraHandle, &devSelInfo, false);
            colorCallback.start();
            sleep(2);
            APC_SetMetaDataEventState(cameraHandle, &devSelInfo, false);
            fprintf(stderr, "[Case99] Case3 --\n");

            // Case 4: Not call close and release camera.
            fprintf(stderr, "[Case99] Case4 ++\n");
            APC_SetMetaDataEventState(cameraHandle, &devSelInfo, true, callback);
            sleep(2);
            fprintf(stderr, "[Case99] Case4 --\n");
            colorCallback.stop();

            delete gJsonFileSerializer;
            gJsonFileSerializer = nullptr;
            // case99_static_clear
            sFirstFrame = false;
            sValue320e = 0x0;
            sValue320f = 0x0;
            sFPS = 0;

            APC_CloseDevice(cameraHandle, &devSelInfo);
            APC_Release(&cameraHandle);

            /** $ rm -rf out_img/ *.json json/ jsonFolder/
             * After the test delete the following folders & files.
             */
            break;
        }
        case 100:
            CT_DEBUG("test pause/resume streaming\n");
            snapShot_color = false;
            snapShot_depth = false;
            bKeepStreaming = true;

            init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++) {
                CT_DEBUG("######################################################################\n");
                CT_DEBUG("Device Count: %d, Select Device Index: %d\n", APC_GetDeviceNumber(EYSD), i);
                SelectDevInx(i);
                open_device_default(true, 1280, 720, 1280, 720, 60, APC_DEPTH_DATA_11_BITS, false);
            }
            test_pause_resume_streaming();
            close_device();
            release_device();
            CT_DEBUG("finish test.\n");
            exit(0);
            break;
        case 101:
            // IVY2 / IVY4 876 or 777 can use this
            init_device(true);
            //open_device();
            RegisterSettings::Batch_read_ASIC(EYSD,GetDevSelectIndexPtr(), g_pDevInfo[gsDevSelInfo.index].nChipID);
            break;
        case 102: {
            //IVY2 876+777 camera please choose 777 for FW & Sensor Register, device index will be 1 insert only 1 cam.
            //IVY4 876 camera please choose FW & Sensor Register, usually device index 0 if you insert only 1 cam.
            init_device(true);

            int pid = g_pDevInfo[gsDevSelInfo.index].wPID;
            int SensorSlaveAddress = pid == APC_PID_IVY2_S ? 0x20 : pid == APC_PID_IVY4 ? 0x6C : 0x0;
            if (!pid) CT_DEBUG("This case only apply in the IVY camera. Please choose another item.\n");

            RegisterSettings::Batch_read_ASIC(EYSD,GetDevSelectIndexPtr(), g_pDevInfo[gsDevSelInfo.index].nChipID);
            RegisterSettings::Batch_read_Sensor(EYSD,GetDevSelectIndexPtr(), SensorSlaveAddress, FG_Address_2Byte|FG_Value_1Byte,
                                                g_pDevInfo[gsDevSelInfo.index].wPID);
            RegisterSettings::Batch_read_FW(EYSD,GetDevSelectIndexPtr(), 1);
            RegisterSettings::Batch_read_FW(EYSD,GetDevSelectIndexPtr(), 2);
            RegisterSettings::Batch_read_FW(EYSD,GetDevSelectIndexPtr(), 3);
            RegisterSettings::Batch_read_FW(EYSD,GetDevSelectIndexPtr(), 4);
            RegisterSettings::Batch_read_FW(EYSD,GetDevSelectIndexPtr(), 5);
            RegisterSettings::Batch_read_FW(EYSD,GetDevSelectIndexPtr(), 6);
            break;
        }
        case 103:
             init_device(false);
            for (int i = 0; i < APC_GetDeviceNumber(EYSD); i++)
                CT_DEBUG("Device Count: %d, Device Index List: %d\n", APC_GetDeviceNumber(EYSD), i);
            for (int i = 0; i < APC_GetSimpleDeviceNumber(EYSD); i++)
                CT_DEBUG("Simple Device Count: %d, Simple Device Index List: %d\n", APC_GetSimpleDeviceNumber(EYSD), SIMPLE_DEV_START_IDX + i);
            printf("Select Device Index (0...%d): ",
                (APC_GetSimpleDeviceNumber(EYSD) > 0) ? (SIMPLE_DEV_START_IDX + APC_GetSimpleDeviceNumber(EYSD) - 1) : (APC_GetDeviceNumber(EYSD) - 1));
            scanf("%d", &input_idx);
            printf("Selected Deivce Index is = [%d]\n", input_idx);
            SelectDevInx(input_idx);
            ReadAllCalibrationData();
            break;
        case 104: {
            //Step 1 :QCS610 or RK board take picture and send to windows calibration tool
            //Step 2 :Send output bin file to QCS610 or RK board, and run this case
            init_device(true); //choose 876
            write_calibration_data(0);
            //for (int index = 0; index < APC_USER_SETTING_OFFSET; ++index) {
                //write_calibration_data(index);
            //}
            break;
        }
        case 105: {
            // Stress Test from init to release.
            // Init -> Open -> StreamOn -> Close -> Release
            fprintf(stderr, "Case 105 Stress Test Start\n");
            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;

            int fps = 30;
            int cw = 1280, ch = 720;
            int dw = 1280, dh = 720;
            int isMjpeg = 0;
            int depthDataType = APC_DEPTH_DATA_11_BITS;
            int frameNumber = 0;
            int run = 0;
            int irIntensity = 0;

            printf("\nPlease input how many times to run open/close device test : \n");
            scanf("%d", &run);
            printf("\nPlease input how many received frames to close device each run : \n");
            scanf("%d", &frameNumber);
            printf("Choose YUYV:0 MJPEG:1 \n");
            scanf("%d", &isMjpeg);
            printf("Color width:\n");
            scanf("%d", &cw);
            printf("Color height:\n");
            scanf("%d", &ch);
            printf("Depth width:\n");
            scanf("%d", &dw);
            printf("Depth height:\n");
            scanf("%d", &dh);
            printf("Choose FPS:\n");
            scanf("%d", &fps);
            printf("Choose Depth data type\nD(11Bits)=4\nZ(14Bits)=2\nColor Only L'+R'=5\nColor Only L+R=0\n");
            scanf("%d", &depthDataType);
            printf("IR Intensity 0 - 6\n");
            scanf("%d", &irIntensity);

            std::size_t runCount = 0;
            while (runCount < run) {
                runCount++;
                fprintf(stderr, "Run %zu\n", runCount);
                int ret = APC_Init(&cameraHandle, true);

                fprintf(stderr, "(01) Init device (%d)\n", ret);

                if (!cameraHandle) {
                    fprintf(stderr, "Null handle reason %d\n", ret);
                    return 0;
                }

                ret = APC_SetDepthDataType(cameraHandle, &devSelInfo, depthDataType);

                ret = APC_SetCurrentIRValue(cameraHandle, &devSelInfo, irIntensity);

                ret = APC_SetupBlock(cameraHandle, &devSelInfo, true);

                ret = APC_OpenDevice2(cameraHandle, &devSelInfo, cw, ch, isMjpeg, dw, dh, DEPTH_IMG_NON_TRANSFER, false,
                                      nullptr, &fps);

                fprintf(stderr, "(02) Open device (%d)\n", ret);

                if (ret) {
                    fprintf(stderr, "Open device failed \n");
                    return 0;
                }

                fprintf(stderr, "(03) Frame number each run = (%d)\n", frameNumber);

                std::size_t dropColor = 0;
                std::size_t dropDepth = 0;
                std::size_t countColor = 0;
                std::size_t countDepth = 0;

                static int64_t lastColorTimeUs;
                lastColorTimeUs = 0;
                libeys3dsample::FrameCallbackHelper::Callback colorCallbackFn([&] (const libeys3dsample::Frame *f) {
                    static int lastSN = 0;
                    int64_t curTimeUs = f->ts_s * 1000000 + f->ts_us;
                    if (lastColorTimeUs && f->sn - lastSN > 1) {
                        fprintf(stderr, "Inside color callback frame.sn=%d lastSN=%d status=%d diffMs=%f \n",
                                f->sn, lastSN, f->status, (curTimeUs - lastColorTimeUs) / 1000.0f);
                        dropColor++;
                    }
                    fprintf(stderr, "+");
                    lastColorTimeUs = curTimeUs;
                    lastSN = f->sn;
                    countColor++;
                    return f->status;
                });

                static int64_t lastDepthTimeUs;
                lastDepthTimeUs = 0;
                libeys3dsample::FrameCallbackHelper::Callback depthCallbackFn([&] (const libeys3dsample::Frame *f) {
                    static int lastSN = 0;
                    int64_t curTimeUs = f->ts_s * 1000000 + f->ts_us;
                    if (lastDepthTimeUs && f->sn - lastSN > 1) {
                        fprintf(stderr, "Inside depth callback frame.sn=%d lastSN=%d status=%d diffMs=%f \n",
                                f->sn, lastSN, f->status, (curTimeUs - lastDepthTimeUs) / 1000.0f);
                        dropDepth++;
                    }
                    fprintf(stderr, ".");
                    lastDepthTimeUs = curTimeUs;
                    lastSN = f->sn;
                    countDepth++;
                    return f->status;
                });

                libeys3dsample::ColorCallbackHelper colorCallback(cameraHandle, devSelInfo, cw, ch, 2, colorCallbackFn);
                libeys3dsample::DepthCallbackHelper depthCallback(cameraHandle, devSelInfo, dw, dh, 2, depthCallbackFn);

                colorCallback.start();
                depthCallback.start();

                while (1) {
                    bool colorEnd = cw == 0 ? true : countColor > frameNumber;
                    bool depthEnd = dw == 0 ? true : countDepth > frameNumber;
                    if (colorEnd && depthEnd)
                        break;
                    usleep(10 * 1000);
                }

                fprintf(stderr, "Stopping Streams !!\n");

                depthCallback.stop();
                colorCallback.stop();

                fprintf(stderr, "[Result] Last Color SN %d Last Depth SN %d / DropColor: %lu DropDepth: %lu\n",
                        colorCallback.mFrame->sn, depthCallback.mFrame->sn, dropColor, dropDepth);

                fprintf(stderr, "(04) Get image done\n");

                ret = APC_CloseDevice(cameraHandle, &devSelInfo);
                fprintf(stderr, "(05) Close device ret=%d) !!\n", ret);

                APC_Release(&cameraHandle);
                fprintf(stderr, "(06) Release device\n");
            }

            break;
        }
        case 106: {
            void* cameraHandle = nullptr;
            DEVSELINFO devSelInfo;
            devSelInfo.index = 0;
            int cw = 1280, ch = 720;
            int run = 2000;

            printf("\nPlease input run to profile the API : \n");
            scanf("%d", &run);
            printf("Color width:\n");
            scanf("%d", &cw);
            printf("Color height:\n");
            scanf("%d", &ch);
            int ret = APC_Init(&cameraHandle, false);
            if (ret || !run) {
                printf("\nPlease plugin 1 camera or valid run > 0 !! \n");
                break;
            }

            const std::size_t imageSize = cw * ch * 2;
            auto randomImage = new unsigned char[imageSize];
            auto decodedImage = new unsigned char[imageSize/2 * 3];

            auto generateTestPattern = [] (unsigned char* buffer, int width, int height) {
                for (int y = 0; y < height; ++y) {
                    for (int x = 0; x < width; x += 2) {
                        int index = y * width * 2 + x * 2;
                        // Y1 U Y2 V
                        buffer[index] = static_cast<unsigned char>((x + y) % 256);
                        buffer[index + 1] = static_cast<unsigned char>((x * y) % 256);
                        buffer[index + 2] = static_cast<unsigned char>((x + y + 1) % 256);
                        buffer[index + 3] = static_cast<unsigned char>((x * y + 128) % 256);
                    }
                }
            };

            // Generate test pattern
            generateTestPattern(randomImage, cw, ch);

            size_t durationUs = 0;
            for (int i = 0; i < run; ++i) {
                auto start = std::chrono::high_resolution_clock::now();

                ret = APC_ColorFormat_to_RGB24(cameraHandle, &devSelInfo, decodedImage, randomImage, imageSize,
                                               cw, ch, APCImageType::Value::COLOR_YUY2);
                auto end = std::chrono::high_resolution_clock::now();
                durationUs += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }

            auto averageFrameProcessTimeUs = (double) durationUs / run;

            fprintf(stderr, "[eys3d_case_106] Resolution %dx%d Average Ms: %f for %d frames\n", cw, ch,
                    averageFrameProcessTimeUs / 1000, run);

            delete randomImage;
            delete decodedImage;

            APC_Release(&cameraHandle);
            break;
        }
        case 255:
            close_device();
            release_device();
            return 0;
            break; 
        default:
            continue;
        }
    } while(1);

    return 0;
}

static void ReadAllCalibrationData(void)
{
    int index = 0;
    int nxybfferLength  = APC_Y_OFFSET_FILE_SIZE;
    int nrecbfferLength = APC_RECTIFY_FILE_SIZE;
    int nzdbfferLength  = APC_ZD_TABLE_FILE_SIZE_11_BITS;
    int nlogbfferLength = APC_CALIB_LOG_FILE_SIZE;

    int pxyActualLength  = 0;
    int precActualLength = 0;
    int pzdActualLength  = 0;
    int plogActualLength = 0;

    ZDTABLEINFO zdTableInfo;
    zdTableInfo.nDataType = APC_DEPTH_DATA_11_BITS;

    BYTE *xydata = new BYTE[nxybfferLength];
    memset(xydata, 0x0, nxybfferLength);
    BYTE *recdata = new BYTE[nrecbfferLength];
    memset(recdata, 0x0, nrecbfferLength);
    BYTE *zddata = new BYTE[nzdbfferLength];
    memset(zddata, 0x0, nzdbfferLength);
    BYTE *logdata = new BYTE[nlogbfferLength];
    memset(logdata, 0x0, nlogbfferLength);

    int group = 1;
    int CheckSupportFP = APC_GetFlashProtectionSupported(EYSD, GetDevSelectIndexPtr());
    if (CheckSupportFP == APC_OK){
        printf("\nThis camera support FW protection, which group do you want to read? \n");
        printf("1 is Protection area(Factory), 2 is User area \n");
        printf("Please enter Group: 1 or 2\n");
        scanf("%d", &group);
        printf("group is %d\n\n",group);
        if (group < 1 || group > 2)
        {
            printf("Wrong number, Please enter Group: 1 or 2\n");
            return;
        }
    }
    else if (CheckSupportFP == APC_NotSupport){
        printf("This camera not support FW protection, read group1 data \n");
    }
    else{
        printf("CheckSupportFP fail\n");
        return;
    }

    if (group == 2)
    {
            if (APC_OK == APC_GetYOffset(EYSD, GetDevSelectIndexPtr(), xydata, nxybfferLength, &pxyActualLength, index + FW_FID_GROUP_OFFSET))
            {
                printf("Read3%d XY offset Success\n", index + FW_FID_GROUP_OFFSET);
                for (int i = 0; i < nxybfferLength; i++)
                {
                    printf("%02x ", xydata[i]);
                }
                printf("\n");
                char xyoffsetfileName[256] = {0};
                printf("read_file name, index number still 0, because need feed to K tool\n\n");
                sprintf(xyoffsetfileName, "calibrationdata/read/read_fileio_buffer_xyoffset_%d.bin", index);
                std::ofstream file(xyoffsetfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(xydata), nxybfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read3%d XY offset Fail!\n", index + FW_FID_GROUP_OFFSET);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }

            if (APC_OK == APC_GetRectifyTable(EYSD, GetDevSelectIndexPtr(), recdata, nrecbfferLength, &precActualLength, index + FW_FID_GROUP_OFFSET))
            {
                printf("Read4%d Rectify Success\n", index + FW_FID_GROUP_OFFSET);
                for (int i = 0; i < nrecbfferLength; i++)
                {
                    printf("%02x ", recdata[i]);
                }
                printf("\n");
                char recyifyfileName[256] = {0};
                printf("read_file name, index number still 0, because need feed to K tool\n\n");
                sprintf(recyifyfileName, "calibrationdata/read/read_fileio_buffer_rectify_%d.bin", index);
                std::ofstream file(recyifyfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(recdata), nrecbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read4%d Rectify Fail!\n", index + FW_FID_GROUP_OFFSET);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }

            zdTableInfo.nIndex = index + FW_FID_GROUP_OFFSET;

            if (APC_OK == APC_GetZDTable(EYSD, GetDevSelectIndexPtr(), zddata, nzdbfferLength, &pzdActualLength, &zdTableInfo))
            {
                printf("Read5%d ZD Success\n", index + FW_FID_GROUP_OFFSET);
                for (int i = 0; i < nzdbfferLength; i++)
                {
                    printf("%02x ", zddata[i]);
                }
                printf("\n");
                char zdtablefileName[256] = {0};
                printf("read_file name, index number still 0, because need feed to K tool\n\n");
                sprintf(zdtablefileName, "calibrationdata/read/read_fileio_buffer_zdtable_%d.bin", index);
                std::ofstream file(zdtablefileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(zddata), nzdbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read5%d ZD Fail!\n", index + FW_FID_GROUP_OFFSET);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }

            if (APC_OK == APC_GetLogData(EYSD, GetDevSelectIndexPtr(), logdata, nlogbfferLength, &plogActualLength, index + FW_FID_GROUP_OFFSET, ALL_LOG))
            {
                printf("Read24%d ALL_LOG Success\n", index + FW_FID_GROUP_OFFSET);
                for (int i = 0; i < nlogbfferLength; i++)
                {
                    printf("%02x ", logdata[i]);
                }
                printf("\n");
                char logfileName[256] = {0};
                printf("read_file name, index number still 0, because need feed to K tool\n\n");
                sprintf(logfileName, "calibrationdata/read/read_fileio_buffer_log_%d.bin", index);
                std::ofstream file(logfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(logdata), nlogbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read24%d ALL_LOG Fail!\n", index + FW_FID_GROUP_OFFSET);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }
    }
    else if (group == 1)
    {
            if (APC_OK == APC_GetYOffset(EYSD, GetDevSelectIndexPtr(), xydata, nxybfferLength, &pxyActualLength, index))
            {
                printf("Read3%d XY offset Success\n", index);
                for (int i = 0; i < nxybfferLength; i++)
                {
                    printf("%02x ", xydata[i]);
                }
                printf("\n");
                char xyoffsetfileName[256] = {0};
                printf("\n");
                sprintf(xyoffsetfileName, "calibrationdata/read/read_fileio_buffer_xyoffset_%d.bin", index);
                std::ofstream file(xyoffsetfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(xydata), nxybfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read3%d XY offset Fail!\n", index);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }

            if (APC_OK == APC_GetRectifyTable(EYSD, GetDevSelectIndexPtr(), recdata, nrecbfferLength, &precActualLength, index))
            {
                printf("Read4%d Rectify Success\n", index);
                for (int i = 0; i < nrecbfferLength; i++)
                {
                    printf("%02x ", recdata[i]);
                }
                printf("\n");
                char recyifyfileName[256] = {0};
                printf("\n");
                sprintf(recyifyfileName, "calibrationdata/read/read_fileio_buffer_rectify_%d.bin", index);
                std::ofstream file(recyifyfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(recdata), nrecbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read4%d Rectify Fail!\n", index);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }

            zdTableInfo.nIndex = index;
            if (APC_OK == APC_GetZDTable(EYSD, GetDevSelectIndexPtr(), zddata, nzdbfferLength, &pzdActualLength, &zdTableInfo))
            {
                printf("Read5%d ZD success\n", index);
                for (int i = 0; i < nzdbfferLength; i++)
                {
                    printf("%02x ", zddata[i]);
                }
                printf("\n");
                char zdtablefileName[256] = {0};
                printf("\n");
                sprintf(zdtablefileName, "calibrationdata/read/read_fileio_buffer_zdtable_%d.bin", index);
                std::ofstream file(zdtablefileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(zddata), nzdbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read5%d ZD Fail!\n", index);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }

            if (APC_OK == APC_GetLogData(EYSD, GetDevSelectIndexPtr(), logdata, nlogbfferLength, &plogActualLength, index, ALL_LOG))
            {
                printf("Read24%d ALL_LOG success\n", index);
                for (int i = 0; i < nlogbfferLength; i++)
                {
                    printf("%02x ", logdata[i]);
                }
                printf("\n");
                char logfileName[256] = {0};
                sprintf(logfileName, "calibrationdata/read/read_fileio_buffer_log_%d.bin", index);
                std::ofstream file(logfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(logdata), nlogbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("Read24%d ALL_LOG Fail!\n", index);
                printf("Note that a 'fail' might occur if the camera doesn't require the data area for the calibration. \n\n");
            }
    }
    delete[] xydata;
    delete[] recdata;
    delete[] zddata;
    delete[] logdata;
}

int load_file_data_to_buffer(uint8_t* p_buffer, int size, const char* filename) {

    CT_DEBUG("Load file %s to buffer data\n",filename);
    FILE* pFile;
    pFile = fopen(filename, "rb+");

    if (pFile == nullptr) {
        CT_DEBUG("Can not open file %s \n",filename);
        return -1;
    }
    size_t data_size = fread(p_buffer, sizeof(uint8_t), size, pFile);
    if (data_size != size) {
        CT_DEBUG("Bin size not match, target size is %d \n",size);
        return -1;
    }
    CT_DEBUG("Load file to data success\n");
    fclose(pFile);
    return 0;
}

static void write_calibration_data(int fileIndex) {
    if (fileIndex > APC_USER_SETTING_OFFSET) return;
    auto devSelInfo = &gsDevSelInfo;

    int final_fileIndex = 0;
    int CheckSupportFP = APC_GetFlashProtectionSupported(EYSD, devSelInfo);
    if (CheckSupportFP == APC_OK){
        final_fileIndex = fileIndex + FW_FID_GROUP_OFFSET;
        CT_DEBUG("This camera support FW protection, write data to group2 because group1 is Protection Area \n");
    }
    else if (CheckSupportFP == APC_NotSupport){
        final_fileIndex = fileIndex;
        CT_DEBUG("This camera not support FW protection, write data to group1 \n");
    }
    else{
        CT_DEBUG("CheckSupportFP fail\n");
        return;
    }
    //xy offset
    CT_DEBUG("Start write XY offset file ID 3%d \n",fileIndex);
    int xyactualLength = 0;
    uint8_t* xy_buffer_r = new uint8_t[APC_Y_OFFSET_FILE_SIZE];
    memset(xy_buffer_r, 0, APC_Y_OFFSET_FILE_SIZE);
    char xyfileName[256] = {0};
    sprintf(xyfileName, "calibrationdata/write/write_fileio_buffer_xyoffset_%d.bin", fileIndex);

    if (load_file_data_to_buffer(xy_buffer_r, APC_Y_OFFSET_FILE_SIZE, xyfileName) < 0) {
        CT_DEBUG("Load XY offset file fail \n");
    } else {
        if (APC_SetYOffset(EYSD, devSelInfo, xy_buffer_r, APC_Y_OFFSET_FILE_SIZE, &xyactualLength,final_fileIndex) < 0) {
            CT_DEBUG("APC_SetYOffset fail\n");
            return;
        }
        else {
            CT_DEBUG("APC_SetYOffset success \n");
        }
    }
    delete[] xy_buffer_r;

    //rectify
    CT_DEBUG("Start write Rectify file ID 4%d \n",fileIndex);
    int recactualLength = 0;
    uint8_t* rec_buffer_r = new uint8_t[APC_RECTIFY_FILE_SIZE];
    memset(rec_buffer_r, 0, APC_RECTIFY_FILE_SIZE);
    char recfileName[256] = {0};
    sprintf(recfileName, "calibrationdata/write/write_fileio_buffer_rectify_%d.bin", fileIndex);

    if (load_file_data_to_buffer(rec_buffer_r, APC_RECTIFY_FILE_SIZE, recfileName) < 0) {
        CT_DEBUG("Load Rectify file fail \n");
    } else {
        if (APC_SetRectifyTable(EYSD, devSelInfo, rec_buffer_r, APC_RECTIFY_FILE_SIZE, &recactualLength,final_fileIndex) < 0) {
            CT_DEBUG("APC_SetRectifyTable fail \n");
            return;
        }
        else {
            CT_DEBUG("APC_SetRectifyTable success \n");
        }
    }
    delete[] rec_buffer_r;

    //zdtable
    CT_DEBUG("Start write ZDtable file ID 5%d \n",fileIndex);
    int zdactualLength = 0;
    uint8_t* zd_buffer_r = new uint8_t[APC_ZD_TABLE_FILE_SIZE_11_BITS];
    memset(zd_buffer_r, 0, APC_ZD_TABLE_FILE_SIZE_11_BITS);
    char zdfileName[256] = {0};
    sprintf(zdfileName, "calibrationdata/write/write_fileio_buffer_zdtable_%d.bin", fileIndex);

    if (load_file_data_to_buffer(zd_buffer_r, APC_ZD_TABLE_FILE_SIZE_11_BITS, zdfileName) < 0) {
        CT_DEBUG("Load ZDtable file fail \n");
    } else {
        ZDTABLEINFO ZDTableInfo;
        ZDTableInfo.nIndex = final_fileIndex;
        if (APC_SetZDTable(EYSD, devSelInfo, zd_buffer_r, APC_ZD_TABLE_FILE_SIZE_11_BITS, &zdactualLength, &ZDTableInfo) < 0) {
            CT_DEBUG("APC_SetZDTable fail \n");
            return;
        }
        else {
            CT_DEBUG("APC_SetZDTable success \n");
        }
    }
    delete[] zd_buffer_r;

    //log
    CT_DEBUG("Start write log file ID 24%d \n",fileIndex);
    int logactualLength = 0;
    uint8_t* log_buffer_r = new uint8_t[APC_CALIB_LOG_FILE_SIZE];
    memset(log_buffer_r, 0, APC_CALIB_LOG_FILE_SIZE);
    char logfileName[256] = {0};
    sprintf(logfileName, "calibrationdata/write/write_fileio_buffer_log_%d.bin", fileIndex);

    if (load_file_data_to_buffer(log_buffer_r, APC_CALIB_LOG_FILE_SIZE, logfileName) < 0) {
        CT_DEBUG("Load log file fail \n");
    } else {
        if (APC_SetLogData(EYSD, devSelInfo, log_buffer_r, APC_CALIB_LOG_FILE_SIZE, &logactualLength, final_fileIndex) < 0) {
            CT_DEBUG("APC_SetLogData fail \n");
            return;
        }
        else {
            CT_DEBUG("APC_SetLogData success \n");
        }
    }
    delete[] log_buffer_r;
}

static void copy_file_to_g2(int fileIndex) {

    if (fileIndex > APC_USER_SETTING_OFFSET) return;

    auto bufferYOffset = new BYTE[APC_Y_OFFSET_FILE_SIZE];
    auto bufferYOffsetBackup = new BYTE[APC_Y_OFFSET_FILE_SIZE];
    auto devSelInfo = &gsDevSelInfo;
    int actualYOffsetBufLen = 0;

    int ret = APC_GetYOffset(EYSD, devSelInfo, bufferYOffset, APC_Y_OFFSET_FILE_SIZE, &actualYOffsetBufLen, fileIndex);

    if (APC_OK != ret || actualYOffsetBufLen != APC_Y_OFFSET_FILE_SIZE) {
        CT_DEBUG("### [Stage-YOffset] Read error\n");
    } else {
        CT_DEBUG("### [Stage-YOffset] Read actualYOffsetBufLen %d file 3%d ret=%d\n", actualYOffsetBufLen, fileIndex, ret);

        memcpy(bufferYOffsetBackup, bufferYOffset, actualYOffsetBufLen);
#ifdef COPY_CLEAN
        memset(bufferYOffset, 0x0, APC_Y_OFFSET_FILE_SIZE);
#endif
        ret = APC_SetYOffset(EYSD, devSelInfo, bufferYOffset, APC_Y_OFFSET_FILE_SIZE, &actualYOffsetBufLen,
                             fileIndex + APC_USER_SETTING_OFFSET);

        if (ret != APC_OK || actualYOffsetBufLen != APC_Y_OFFSET_FILE_SIZE) {
            CT_DEBUG("### [Stage-YOffset] Write error\n");
        } else {
            CT_DEBUG("### [Stage-YOffset] Write actualYOffsetBufLen %d file %d ret=%d\n", actualYOffsetBufLen,
                     APC_Y_OFFSET_FILE_ID_0 + fileIndex + APC_USER_SETTING_OFFSET,
                     ret);

            memset(bufferYOffset, 0xff, APC_Y_OFFSET_FILE_SIZE);
            ret = APC_GetYOffset(EYSD, devSelInfo, bufferYOffset, APC_Y_OFFSET_FILE_SIZE, &actualYOffsetBufLen,
                                 fileIndex + APC_USER_SETTING_OFFSET);

            if (ret != APC_OK || actualYOffsetBufLen != APC_Y_OFFSET_FILE_SIZE ||
                memcmp(bufferYOffset, bufferYOffsetBackup, actualYOffsetBufLen)) {
                CT_DEBUG("### [Stage-YOffset] Verify error. Please check.\n");
            } else {
                CT_DEBUG("### [Stage-YOffset] Verify successfully.\n");
            }
        }
    }

    delete [] bufferYOffset;
    delete [] bufferYOffsetBackup;

    auto bufferRectifyTable = new BYTE[APC_RECTIFY_FILE_SIZE];
    auto bufferRectifyTableBackup = new BYTE[APC_RECTIFY_FILE_SIZE];
    int actualRectifyBufLen = 0;
    ret = APC_GetRectifyTable(EYSD, devSelInfo, bufferRectifyTable, APC_RECTIFY_FILE_SIZE,
                              &actualRectifyBufLen, fileIndex);
    if (ret != APC_OK || actualRectifyBufLen != APC_RECTIFY_FILE_SIZE) {
        CT_DEBUG("### [Stage-Rectify] Read error\n");
    } else {
        CT_DEBUG("### [Stage-Rectify] Read actualRectifyBufLen %d file 4%d ret=%d\n", actualRectifyBufLen,
                 fileIndex, ret);

        memcpy(bufferRectifyTableBackup, bufferRectifyTable, actualRectifyBufLen);
#ifdef COPY_CLEAN
        memset(bufferRectifyTable, 0x0, APC_RECTIFY_FILE_SIZE);
#endif
        ret = APC_SetRectifyTable(EYSD, devSelInfo, bufferRectifyTable, APC_RECTIFY_FILE_SIZE,
                                  &actualRectifyBufLen, fileIndex + APC_USER_SETTING_OFFSET);

        if (ret != APC_OK || actualRectifyBufLen != APC_RECTIFY_FILE_SIZE) {
            CT_DEBUG("### [Stage-Rectify] Write error\n");
        } else {
            CT_DEBUG("### [Stage-Rectify] Write actualRectifyBufLen %d file %d ret=%d\n", actualRectifyBufLen,
                     APC_RECTIFY_FILE_ID_0 + fileIndex + APC_USER_SETTING_OFFSET, ret);

            memset(bufferRectifyTable, 0xff, APC_RECTIFY_FILE_SIZE);

            ret = APC_GetRectifyTable(EYSD, devSelInfo, bufferRectifyTable, APC_RECTIFY_FILE_SIZE,
                                      &actualRectifyBufLen, fileIndex + APC_USER_SETTING_OFFSET);

            if (ret != APC_OK || actualRectifyBufLen != APC_RECTIFY_FILE_SIZE ||
                memcmp(bufferRectifyTable, bufferRectifyTableBackup, actualRectifyBufLen)) {
                CT_DEBUG("### [Stage-Rectify] Verify error. Please check.\n");
            } else {
                CT_DEBUG("### [Stage-Rectify] Verify successfully.\n");
            }
        }
    }

    delete [] bufferRectifyTable;
    delete [] bufferRectifyTableBackup;

    auto bufferZDTable = new BYTE[APC_ZD_TABLE_FILE_SIZE_11_BITS];
    auto bufferZDTableBackup = new BYTE[APC_ZD_TABLE_FILE_SIZE_11_BITS];
    int actualZDBufLen = 0;
    ZDTABLEINFO tableInfo {
        .nIndex = fileIndex,
        .nDataType = APC_DEPTH_DATA_11_BITS,
    };

    ret = APC_GetZDTable(EYSD, devSelInfo, bufferZDTable, APC_ZD_TABLE_FILE_SIZE_11_BITS,
                         &actualZDBufLen, &tableInfo);

    if (ret != APC_OK || actualZDBufLen != APC_ZD_TABLE_FILE_SIZE_11_BITS) {
        CT_DEBUG("### [Stage-ZD] Read error\n");
    } else {
        CT_DEBUG("### [Stage-ZD] Read actualZDBufLen %d file 5%d ret=%d\n", actualZDBufLen,
                 fileIndex, ret);

        memcpy(bufferZDTableBackup, bufferZDTable, APC_ZD_TABLE_FILE_SIZE_11_BITS);
#ifdef COPY_CLEAN
        memset(bufferZDTable, 0x0, APC_ZD_TABLE_FILE_SIZE_11_BITS);
#endif
        tableInfo.nIndex = fileIndex + APC_USER_SETTING_OFFSET;
        ret = APC_SetZDTable(EYSD, devSelInfo, bufferZDTable, APC_ZD_TABLE_FILE_SIZE_11_BITS,
                             &actualZDBufLen, &tableInfo);

        if (ret != APC_OK || actualZDBufLen != APC_ZD_TABLE_FILE_SIZE_11_BITS) {
            CT_DEBUG("### [Stage-ZD] Write error\n");
        } else {
            CT_DEBUG("### [Stage-ZD] Write actualZDBufLen %d file %d ret=%d\n", actualZDBufLen,
                     APC_ZD_TABLE_FILE_ID_0 + fileIndex + APC_USER_SETTING_OFFSET, ret);

            memset(bufferZDTable, 0xff, APC_ZD_TABLE_FILE_SIZE_11_BITS);
            tableInfo.nIndex = fileIndex + APC_USER_SETTING_OFFSET;
            ret = APC_GetZDTable(EYSD, devSelInfo, bufferZDTable, APC_ZD_TABLE_FILE_SIZE_11_BITS,
                                 &actualZDBufLen, &tableInfo);

            if (ret != APC_OK || actualZDBufLen != APC_ZD_TABLE_FILE_SIZE_11_BITS ||
                memcmp(bufferZDTable, bufferZDTableBackup, APC_ZD_TABLE_FILE_SIZE_11_BITS)) {
                CT_DEBUG("### [Stage-ZD] Verify error. Please check.\n");
            } else {
                CT_DEBUG("### [Stage-ZD] Verify successfully.\n");
            }
        }
    }

    delete [] bufferZDTable;
    delete [] bufferZDTableBackup;

    auto bufferCalibrationLogData = new BYTE[APC_CALIB_LOG_FILE_SIZE];
    auto bufferCalibrationLogDataBackup = new BYTE[APC_CALIB_LOG_FILE_SIZE];
    int actualCalibrationLogDataBufLen = 0;

    ret = APC_GetLogData(EYSD, devSelInfo, bufferCalibrationLogData, APC_CALIB_LOG_FILE_SIZE,
                         &actualCalibrationLogDataBufLen, fileIndex, ALL_LOG);

    if (APC_OK != ret || actualCalibrationLogDataBufLen != APC_CALIB_LOG_FILE_SIZE) {
        CT_DEBUG("### [Stage-LOG] Read error\n");
    } else {
        CT_DEBUG("### [Stage-LOG] Read actualCalibrationLogDataBufLen %d file 24%d ret=%d\n",
                 actualCalibrationLogDataBufLen, fileIndex, ret);

        memcpy(bufferCalibrationLogDataBackup, bufferCalibrationLogData, APC_CALIB_LOG_FILE_SIZE);
#ifdef COPY_CLEAN
        memset(bufferCalibrationLogData, 0x0, APC_CALIB_LOG_FILE_SIZE);
#endif
        ret = APC_SetLogData(EYSD, devSelInfo, bufferCalibrationLogData, APC_CALIB_LOG_FILE_SIZE,
                             &actualCalibrationLogDataBufLen, fileIndex + APC_USER_SETTING_OFFSET);

        if (actualCalibrationLogDataBufLen != APC_CALIB_LOG_FILE_SIZE || ret != APC_OK) {
            CT_DEBUG("### [Stage-LOG] Write error\n");
        } else {
            CT_DEBUG("### [Stage-LOG] Write actualCalibrationLogDataBufLen %d file %d ret=%d\n",
                     actualCalibrationLogDataBufLen, APC_CALIB_LOG_FILE_ID_0 + fileIndex + APC_USER_SETTING_OFFSET, ret);

            memset(bufferCalibrationLogData, 0xff, APC_CALIB_LOG_FILE_SIZE);
            ret = APC_GetLogData(EYSD, devSelInfo, bufferCalibrationLogData, APC_CALIB_LOG_FILE_SIZE,
                                 &actualCalibrationLogDataBufLen, fileIndex + APC_USER_SETTING_OFFSET, ALL_LOG);

            if (ret != APC_OK || actualCalibrationLogDataBufLen != APC_CALIB_LOG_FILE_SIZE ||
                memcmp(bufferCalibrationLogData, bufferCalibrationLogDataBackup, APC_CALIB_LOG_FILE_SIZE)) {
                CT_DEBUG("### [Stage-LOG] Verify error. Please check.\n");
            } else {
                CT_DEBUG("### [Stage-LOG] Verify successfully.\n");
            }
        }
    }

    delete [] bufferCalibrationLogData;
    delete [] bufferCalibrationLogDataBackup;
}

static void *test_color_depth_time_stamp(void *arg)
{
    pthread_t color_thread_id;
    pthread_attr_t color_thread_attr;
    struct sched_param color_thread_param;
    
    pthread_t depth_thread_id;
    pthread_attr_t depth_thread_attr;
    struct sched_param depth_thread_param;
    
    pthread_attr_init(&color_thread_attr);
    pthread_attr_setschedpolicy(&color_thread_attr, SCHED_FIFO);
    pthread_attr_getschedparam (&color_thread_attr, &color_thread_param);
    color_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&color_thread_attr, &color_thread_param);

    pthread_attr_init(&depth_thread_attr);
    pthread_attr_setschedpolicy(&depth_thread_attr, SCHED_FIFO);
    pthread_attr_getschedparam (&depth_thread_attr, &depth_thread_param);
    depth_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&depth_thread_attr, &depth_thread_param);
    
    pthread_create(&color_thread_id, &color_thread_attr, test_color_time_stamp, NULL);
    pthread_create(&depth_thread_id, &depth_thread_attr, test_depth_time_stamp, NULL);
    
    CT_DEBUG("Wait for finish of depth thread..\n");
    pthread_join(depth_thread_id, NULL);
    CT_DEBUG("Wait for finish of color thread..\n");
    pthread_join(color_thread_id, NULL);
    
    return NULL;
}

unsigned char *alloc_single_depth(void *arg)
{
    int ret = APC_OK;
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    int cur_serial_num = -1;
    int mCount = 0;
    static unsigned int m_BufferSize = 0;

    (void) arg;

    CT_DEBUG("\nDepth Image: [%d x %d @ %d]\n", gDepthWidth, gDepthHeight, gActualFps);

    if (gDepthImgBuf == NULL) {
        if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
            m_BufferSize = 2 * gDepthWidth  * 2 * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        } else {
            m_BufferSize = 2 * gDepthWidth * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        }
    }

    if (gDepthImgBuf == NULL) {
        CT_DEBUG("Alloc for gDepthImageBuf Fail!\n");
        return nullptr;
    }

    ret = APC_GetDepthImageWithTimestamp(EYSD, GetDevSelectIndexPtr(), (BYTE*)gDepthImgBuf, &gDepthImgSize, &cur_serial_num,
                                         gDepthDataType, &cur_tv_sec, &cur_tv_usec);

    // After stream on.
    RegisterSettings::DM_Quality_Register_Setting(EYSD, GetDevSelectIndexPtr(), g_pDevInfo[gsDevSelInfo.index].wPID);

    ret = APC_GetDepthImageWithTimestamp(EYSD, GetDevSelectIndexPtr(), (BYTE*)gDepthImgBuf, &gDepthImgSize, &cur_serial_num,
                                         gDepthDataType, &cur_tv_sec, &cur_tv_usec);

    if (gDepthImgSize > m_BufferSize) {
        CT_DEBUG("Alloc size: %u, but get depth size: %lu, check FW and close the application.\n",
                 m_BufferSize, gDepthImgSize);
        return nullptr;
    }

    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetDepthImageWithTimestamp()!\n");
        return nullptr;
    }

    return gDepthImgBuf;
}

static int release_single_depth(void *arg)
{
    if (gDepthImgBuf == nullptr) {
        return APC_NullPtr;
    }

    free(gDepthImgBuf);
    gDepthImgBuf = nullptr;

    return APC_OK;
}

static void *test_color_time_stamp(void *arg)
{
    int ret = APC_OK;
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    int64_t first_tv_sec = 0;
    int64_t first_tv_usec = 0;
    int64_t prv_tv_sec = -1;
    int64_t prv_tv_usec = -1;
    int cur_serial_num = -1;
    int pre_serial_num = -1;
    uint64_t frame_rate_count = 0;
    unsigned int max_calc_frame_count = 150;
    int mCount = 0;
    bool bFirstReceived = true;
    const char *pre_str = COLOR_STR;
    int64_t diff = 0;
    int s_diff = 0;
    int serial_number = 0;
    int i = 0;
    
    CT_DEBUG("\nColor Image: [%d x %d @ %d]\n", gColorWidth, gColorHeight, gActualFps);
    
    if (gColorImgBuf == NULL) {
        gColorImgBuf = (unsigned char*)calloc(2 * gColorWidth * gColorHeight, sizeof(unsigned char));
    }
    if (gColorImgBuf == NULL) {
        CT_DEBUG("Alloc ColorImgBuf Fail!\n");
        return NULL;
    }
#if defined(ONLY_PRINT_OVER_DIFF)
    max_calc_frame_count = 1000;
#endif   
    while (mCount < 1)
    {
        ret = APC_GetColorImageWithTimestamp(EYSD, GetDevSelectIndexPtr(),
                                             (BYTE*)gColorImgBuf, &gColorImgSize, &cur_serial_num, 0, &cur_tv_sec, &cur_tv_usec);
        if ((ret == APC_OK) /*&& (gColorSerial > 0)*/) {
            serial_number = 0;
            /* TODO: 
                 * The following serail_number is for !AXES1 type models (ex: KIWI, PUMA)
                 * The other type models should be implmented in the feature.
                 */
            if (gColorFormat == 0) {   
                //V4L2_PIX_FMT_YUYV
                for (i = 0; i < 16; i++) {
                    serial_number |= (*(((unsigned char*)gColorImgBuf) + i) & 1) << i;
                }
            } else {
                //V4L2_PIX_FMT_MJPEG
                  serial_number = *(((unsigned char*)gColorImgBuf)+ 6) * 256 + *(((unsigned char*)gColorImgBuf) + 7);
            }
            if (bFirstReceived) {
                bFirstReceived = false;
                CT_DEBUG("[%s]SN: [%03d/%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                       (int)cur_serial_num, serial_number, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
            }
            if (frame_rate_count == 0) {
                first_tv_sec  = cur_tv_sec;
                first_tv_usec = cur_tv_usec;
            } else {
                diff = ((cur_tv_sec - prv_tv_sec) * 1000000 + cur_tv_usec) - prv_tv_usec;
                s_diff = cur_serial_num - pre_serial_num;
#if defined(ONLY_PRINT_OVER_DIFF)
                if (gActualFps == 60) {                 
                    if (diff > (16666)) {
                       // CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                         //   (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                    }

                } else  if (gActualFps == 30) {
                    if (diff > (33333)) {
                        //CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                          //  (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                    }
                }

                if (s_diff > 1) {
                    CT_DEBUG("[%s][%03lu]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n",
                            pre_str, frame_rate_count,
                            (int)cur_serial_num, s_diff,
                           (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                }
#else
                CT_DEBUG("[%s]SN: [%03d/%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                       (int)cur_serial_num, serial_number, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
#endif
            }

            if (frame_rate_count == (max_calc_frame_count -1)) {              
                float fltotal_time = 0.0;
                fltotal_time = ((cur_tv_sec - first_tv_sec) * 1000000 + cur_tv_usec) - first_tv_usec;
                CT_DEBUG("[%s] %lu usec per %ufs (%6f)\n", pre_str,
                       (unsigned long)fltotal_time, max_calc_frame_count, (1000000 * max_calc_frame_count)/fltotal_time);
                frame_rate_count = 0;
                if (!bKeepStreaming) {
                    mCount++;
                }
            } else {
                frame_rate_count++;
            }
            prv_tv_sec = cur_tv_sec;
            prv_tv_usec = cur_tv_usec;
            pre_serial_num = cur_serial_num;
        } else {
           CT_DEBUG("Failed to call APC_GetColorImageWithTimestamp()!\n");
           if (ret == APC_DEVICE_TIMEOUT) {
                /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                      */
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
    }
    if (gColorImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gColorImgBuf: %p\n", gColorImgBuf);
        }
        free(gColorImgBuf);
        gColorImgBuf = NULL;
    }
    return NULL;
}

static void *test_depth_time_stamp(void *arg)
{
    int ret = APC_OK;
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    int64_t first_tv_sec = 0;
    int64_t first_tv_usec = 0;
    int64_t prv_tv_sec = -1;
    int64_t prv_tv_usec = -1;
    int cur_serial_num = -1;
    int pre_serial_num = -1;
    uint64_t frame_rate_count = 0;
    unsigned int max_calc_frame_count = 150;
    int mCount = 0;
    static unsigned int m_BufferSize = 0;
    bool bFirstReceived = true;
    bool bAfterQCFG = false;
    bool bSecondReceived = false;
    const char *pre_str = DEPTH_STR;
    int64_t diff = 0;
    int s_diff = 0;
    int serial_number = 0;
    int i = 0;

    (void)arg;

    CT_DEBUG("\nDepth Image: [%d x %d @ %d]\n", gDepthWidth, gDepthHeight, gActualFps);
    
    if (gDepthImgBuf == NULL) {
         if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
            m_BufferSize = 2 * gDepthWidth  * 2 * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        } else {
            m_BufferSize = 2 * gDepthWidth * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        }
    }
    
    if (gDepthImgBuf == NULL) {
        CT_DEBUG("Alloc for gDepthImageBuf Fail!\n");
        return NULL;
    }
#if defined(ONLY_PRINT_OVER_DIFF)
    max_calc_frame_count = 1000;
#endif
    while (mCount < 1)
    {
        ret = APC_GetDepthImageWithTimestamp(EYSD, GetDevSelectIndexPtr(), (BYTE*)gDepthImgBuf, &gDepthImgSize, &cur_serial_num, gDepthDataType, &cur_tv_sec, &cur_tv_usec);

        if (gDepthImgSize > m_BufferSize) {
            CT_DEBUG("Alloc size: %lu, but get depth size: %lu, check FW and close the application.\n", m_BufferSize, gDepthImgSize);
            break;
        }
        if (ret == APC_OK) {
            serial_number = 0;
            /* TODO: 
                 * The following serail_number is for !AXES1 type models (ex: KIWI, PUMA)
                 * The other type models should be implmented in the feature.
                 */
            if (gColorFormat == 0) {   
                //V4L2_PIX_FMT_YUYV
                for (i = 0; i < 16; i++) {
                    serial_number |= (*(((unsigned char*)gDepthImgBuf) + i) & 1) << i;
                }
            } else {
                //V4L2_PIX_FMT_MJPEG
                serial_number = *(((unsigned char*)gDepthImgBuf)+ 6) * 256 + *(((unsigned char*)gDepthImgBuf) + 7);
            }
            if (bAfterQCFG) {
                //CT_DEBUG("[%s]After setting of Quality, s_num: [%d] (0x%08x)\n", __func__, (int)cur_serial_num, (unsigned int)cur_serial_num);
                bAfterQCFG = false;
            }
            
            if (bSecondReceived) {
                bSecondReceived = false;
                //CT_DEBUG("[%s]SN: [%d], TS: [%lu]\n", pre_str, (int)cur_serial_num, (cur_tv_sec * 1000000 + cur_tv_usec));
            }
            
            if (bFirstReceived) {
                bFirstReceived = false;
                CT_DEBUG("[%s]SN: [%03d/%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                       (int)cur_serial_num, serial_number, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                if (RegisterSettings::DM_Quality_Register_Setting(EYSD, GetDevSelectIndexPtr(), g_pDevInfo[gsDevSelInfo.index].wPID) == 0) {
                    bAfterQCFG = true;
                }
                bSecondReceived = true;
            }

            if (frame_rate_count == 0) {
                first_tv_sec  = cur_tv_sec;
                first_tv_usec = cur_tv_usec;

            } else {
                diff = ((cur_tv_sec - prv_tv_sec) * 1000000 + cur_tv_usec) - prv_tv_usec;
                s_diff = cur_serial_num - pre_serial_num;
#if defined(ONLY_PRINT_OVER_DIFF)
                if (gActualFps == 60) {
                    if (diff > (16666)) {
                         //CT_DEBUG("[%s]t_diff: [%lu] usec (s_diff: [%d])\n", __func__, (unsigned long)diff, s_diff);
                    }
                } else  if (gActualFps == 30) {
                    if (diff > (33333)) {
                        //CT_DEBUG("[%s]t_diff: [%lu] usec (s_diff: [%d])\n", __func__, (unsigned long)diff, s_diff);
                    }
                }
                if (s_diff > 1) {
                    CT_DEBUG("[%s][%03lu]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n",
                            pre_str, frame_rate_count,
                            (int)cur_serial_num, s_diff,
                           (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                }
#else     
                CT_DEBUG("[%s]SN: [%03d/%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                       (int)cur_serial_num, serial_number, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
#endif
            }

            if (frame_rate_count == (max_calc_frame_count -1)) {              
                float fltotal_time = 0.0;
                fltotal_time = ((cur_tv_sec - first_tv_sec) * 1000000 + cur_tv_usec) - first_tv_usec;
                CT_DEBUG("[%s] %lu usec per %ufs (%6f)\n", pre_str,
                       (unsigned long)fltotal_time, max_calc_frame_count, (1000000 * max_calc_frame_count)/fltotal_time);
                frame_rate_count = 0;
                if(!bKeepStreaming) {
                    mCount++;
                }
            } else {
                frame_rate_count++;
            }
            prv_tv_sec = cur_tv_sec;
            prv_tv_usec = cur_tv_usec;
            pre_serial_num = cur_serial_num;
        } else {
            CT_DEBUG("Failed to call APC_GetDepthImageWithTimestamp()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                      */
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
    }

    //s:[eys3D] 20200610 implement to save raw data to RGB format
    if (gDepthImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gDepthImgBuf: %p\n", gDepthImgBuf);
        }
        free(gDepthImgBuf);
        gDepthImgBuf = NULL;
    }

    return NULL;
}


int saveRawFile(const char *pFilePath, BYTE *pBuffer, unsigned int sizeByte)
{
    if (!pFilePath || !pBuffer) { 
        return APC_NullPtr;
    }

    FILE *pFile = fopen(pFilePath, "wb");
    if (!pFile) return APC_NullPtr;

    fseek(pFile, 0, SEEK_SET);
    fwrite(&pBuffer[0], sizeof(unsigned char), sizeByte, pFile);
    fclose(pFile);
    return APC_OK;
}

static void *pthread_saving_depth(void *param) {
    int ret = APC_OK;
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    int64_t first_tv_sec = 0;
    int64_t first_tv_usec = 0;
    int64_t prv_tv_sec = -1;
    int64_t prv_tv_usec = -1;
    int cur_serial_num = -1;
    int pre_serial_num = -1;
    uint64_t frame_rate_count = 0;
    unsigned int max_calc_frame_count = DEFAULT_SAVING_FRAME_COUNT;
    int mCount = 0;
    static unsigned int m_BufferSize = 0;
    bool bFirstReceived = true;
    bool bAfterQCFG = false;
    bool bSecondReceived = false;
    const char *pre_str = DEPTH_STR;
    int64_t diff = 0;
    int s_diff = 0;

    (void)param;
    std::string fileName = "";

    CT_DEBUG("\nDepth Image: [%d x %d @ %d], snapshot %u frames!\n", gDepthWidth, gDepthHeight, gActualFps, max_calc_frame_count);
    
    if (gDepthImgBuf == NULL) {
        if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
            m_BufferSize = 2 * gDepthWidth  * 2 * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        } else {
            m_BufferSize = 2 * gDepthWidth * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        }
    }
    
    if (gDepthImgBuf == NULL) {
        CT_DEBUG("Alloc for gDepthImageBuf Fail!\n");
        return NULL;
    }

    while (mCount < 1)
    {
        ret = APC_GetDepthImageWithTimestamp(EYSD, GetDevSelectIndexPtr(),
                                             (BYTE*)gDepthImgBuf, &gDepthImgSize, &cur_serial_num, 
                                             gDepthDataType, &cur_tv_sec, &cur_tv_usec);

        if (gDepthImgSize > m_BufferSize) {
            CT_DEBUG("Alloc size: %lu, but get depth size: %lu, check FW and close the application.\n", m_BufferSize, gDepthImgSize);
            break;
        }
        if (ret == APC_OK) {
            
            if (bAfterQCFG) {
                //CT_DEBUG("[%s]After setting of Quality, s_num: [%d] (0x%08x)\n", __func__, (int)cur_serial_num, (unsigned int)cur_serial_num);
                bAfterQCFG = false;
            }
            
            if (bSecondReceived) {
                bSecondReceived = false;
                //CT_DEBUG("[%s]SN: [%d], TS: [%lu]\n", pre_str, (int)cur_serial_num, (cur_tv_sec * 1000000 + cur_tv_usec));
            }
            
            if (bFirstReceived) {
                bFirstReceived = false;
                CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                       (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                if (RegisterSettings::DM_Quality_Register_Setting(EYSD, GetDevSelectIndexPtr(), g_pDevInfo[gsDevSelInfo.index].wPID) == 0) {
                    bAfterQCFG = true;
                }
                bSecondReceived = true;
            }
            
            // 001 Take care file name
            fileName.append(SAVE_FILE_PATH"DEPTH");
            fileName.append(std::to_string(cur_serial_num));
            fileName.append("-");
            fileName.append(std::to_string(cur_tv_sec * 1000000 + cur_tv_usec));
            saveRawFile(fileName.c_str(), gDepthImgBuf, gDepthImgSize);
            fileName.clear();

            if (frame_rate_count == 0) {
                first_tv_sec  = cur_tv_sec;
                first_tv_usec = cur_tv_usec;

            } else {
                diff = ((cur_tv_sec - prv_tv_sec) * 1000000 + cur_tv_usec) - prv_tv_usec;
                s_diff = cur_serial_num - pre_serial_num;
#if defined(ONLY_PRINT_OVER_DIFF)
                if (gActualFps == 60) {
                    if (diff > (16666)) {
                         //CT_DEBUG("[%s]t_diff: [%lu] usec (s_diff: [%d])\n", __func__, (unsigned long)diff, s_diff);
                    }
                } else if (gActualFps == 30) {
                    if (diff > (33333)) {
                        //CT_DEBUG("[%s]t_diff: [%lu] usec (s_diff: [%d])\n", __func__, (unsigned long)diff, s_diff);
                    }
                }
                /*
                if (s_diff > 1)
                    CT_DEBUG("[%s][%d]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str, frame_rate_count,
                        (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                        */
#else     
                //CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                  //      (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
#endif
            }

            if (frame_rate_count == (max_calc_frame_count -1)) {
                float fltotal_time = 0.0;
                fltotal_time = ((cur_tv_sec - first_tv_sec) * 1000000 + cur_tv_usec) - first_tv_usec;
                CT_DEBUG("[%s] %lu usec per %ufs (%6f)\n", pre_str,
                    (unsigned long)fltotal_time, max_calc_frame_count,(1000000*max_calc_frame_count)/fltotal_time);
                frame_rate_count = 0;
                mCount++;
            } else {
                frame_rate_count++;
            }
            prv_tv_sec = cur_tv_sec;
            prv_tv_usec = cur_tv_usec;
            pre_serial_num = cur_serial_num;
        } else {
            CT_DEBUG("Failed to call APC_GetDepthImageWithTimestamp()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                      */
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
    }
    
    //s:[eys3D] 20200610 implement to save raw data to RGB format
    if (gDepthImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gDepthImgBuf: %p\n", gDepthImgBuf);
        }
        free(gDepthImgBuf);
        gDepthImgBuf = NULL;
     }
}

static void *pthread_saving_color(void *param) {
    int ret = APC_OK;
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    int64_t first_tv_sec = 0;
    int64_t first_tv_usec = 0;
    int64_t prv_tv_sec = -1;
    int64_t prv_tv_usec = -1;
    int cur_serial_num = -1;
    int pre_serial_num = -1;
    uint64_t frame_rate_count = 0;
    unsigned int max_calc_frame_count = DEFAULT_SAVING_FRAME_COUNT;
    int mCount = 0;
    bool bFirstReceived = true;
    const char *pre_str = COLOR_STR;
    int64_t diff = 0;
    int s_diff = 0;
    std::string fileName = "";

    CT_DEBUG("\nColor Image: [%d x %d @ %d], snapshot %u frames!\n", gColorWidth, gColorHeight, gActualFps, max_calc_frame_count);
    

    if (gColorImgBuf == NULL) {
        gColorImgBuf = (unsigned char*)calloc(2 * gColorWidth * gColorHeight, sizeof(unsigned char));
    }
    if (gColorImgBuf == NULL) {
        CT_DEBUG("Alloc ColorImgBuf Fail!\n");
        return NULL;
    }

    const size_t RGB_SIZE = 3, saving_size = gColorWidth * gColorHeight * RGB_SIZE;

    BYTE* writingBuffer = new BYTE[gColorWidth * gColorHeight * RGB_SIZE];
    auto imageType = *(APCImageType::Value*)(param);

    while (mCount < 1)
    {
        // 000 Take out color
        ret = APC_GetColorImageWithTimestamp(EYSD, GetDevSelectIndexPtr(),
                                             (BYTE*)gColorImgBuf, &gColorImgSize, &cur_serial_num,
                                             0, &cur_tv_sec, &cur_tv_usec);
        if ((ret == APC_OK) /*&& (gColorSerial > 0)*/) {
            if (bFirstReceived) {
                bFirstReceived = false;
                CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                       (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
            }
            
            // 000 Convert from to BGR color
            int convertRGBRet = APC_ColorFormat_to_RGB24(EYSD, GetDevSelectIndexPtr(), writingBuffer, gColorImgBuf, gColorImgSize,
                                                         gColorWidth, gColorHeight, imageType);
            // 000 Convert from to RGB color
            // int convertBGRRet = APC_ColorFormat_to_BGR24(EYSD, GetDevSelectIndexPtr(), writingBuffer, gColorImgBuf, gColorImgSize,
            //                                                 gColorWidth, gColorHeight, imageType);

            fileName.append(SAVE_FILE_PATH"COLOR");
            fileName.append(std::to_string(cur_serial_num));
            fileName.append("-");
            fileName.append(std::to_string(cur_tv_sec * 1000000 + cur_tv_usec));
            saveRawFile(fileName.c_str(), writingBuffer, saving_size);
            fileName.clear();

            if (frame_rate_count == 0) {
                first_tv_sec  = cur_tv_sec;
                first_tv_usec = cur_tv_usec;
            } else {
                diff = ((cur_tv_sec - prv_tv_sec) * 1000000 + cur_tv_usec) - prv_tv_usec;
                s_diff = cur_serial_num - pre_serial_num;
#if defined(ONLY_PRINT_OVER_DIFF)
                if (gActualFps == 60) {                 
                    if (diff > (16666)) {
                       // CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                         //   (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                    }

                } else  if (gActualFps == 30) {
                    if (diff > (33333)) {
                        //CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                          //  (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                    }
                }
                /*
                if (s_diff > 1)
                    CT_DEBUG("[%s][%d]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str, frame_rate_count,
                        (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
                        */
#else
                
               // CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
                 //       (int)cur_serial_num, s_diff, (cur_tv_sec * 1000000 + cur_tv_usec), diff);
#endif
            }

            if (frame_rate_count == (max_calc_frame_count -1)) {              
                float fltotal_time = 0.0;
                fltotal_time = ((cur_tv_sec - first_tv_sec) * 1000000 + cur_tv_usec) - first_tv_usec;
                CT_DEBUG("[%s] %lu usec per %ufs (%6f)\n", pre_str,
                       (unsigned long)fltotal_time, max_calc_frame_count,(1000000*max_calc_frame_count)/fltotal_time);
                frame_rate_count = 0;
                mCount++;
            } else {
                frame_rate_count++;
            }
            prv_tv_sec = cur_tv_sec;
            prv_tv_usec = cur_tv_usec;
            pre_serial_num = cur_serial_num;
        } else {
            CT_DEBUG("Failed to call APC_GetColorImageWithTimestamp()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                      */
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
    }

    delete[] writingBuffer;
    
    if (gColorImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gColorImgBuf: %p\n", gColorImgBuf);
        }
        free(gColorImgBuf);
        gColorImgBuf = NULL;
    }
    
    return NULL;
}

static void test_file_saving(APCImageType::Value arg)
{
    pthread_t color_saving_tid = -1;
    sched_param color_saving_thread_param;
    pthread_attr_t color_saving_thread_attr;
    pthread_attr_init(&color_saving_thread_attr);
    pthread_attr_setschedpolicy(&color_saving_thread_attr, SCHED_FIFO);
    pthread_attr_getschedparam (&color_saving_thread_attr, &color_saving_thread_param);
    color_saving_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&color_saving_thread_attr, &color_saving_thread_param);
    pthread_create(&color_saving_tid, &color_saving_thread_attr, pthread_saving_color, (void*) &arg);

    pthread_t depth_saving_tid = -1;
    sched_param depth_saving_thread_param;
    pthread_attr_t depth_saving_thread_attr;
    pthread_attr_init(&depth_saving_thread_attr);
    pthread_attr_setschedpolicy(&depth_saving_thread_attr, SCHED_FIFO);
    pthread_attr_getschedparam (&depth_saving_thread_attr, &depth_saving_thread_param);
    depth_saving_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&depth_saving_thread_attr, &depth_saving_thread_param);
    pthread_create(&depth_saving_tid, &depth_saving_thread_attr, pthread_saving_depth, NULL);
    
    pthread_join(color_saving_tid, NULL);
    pthread_join(depth_saving_tid, NULL);

    return;
}

static int init_device(bool is_select_dev)
{
    int ret, i;
    char FWVersion[128];
    char devBuf[128];
    char devBuf_v4l[128];
    char devBuf_name[128];

    ret = APC_Init(&EYSD, DEBUG_LOG);
    if (ret == APC_OK) {
        if (DEBUG_LOG) CT_DEBUG("APC_Init() Success! (EYSD: %p)\n", EYSD);
    } else {
        CT_DEBUG("APC_Init() Fail! (ret: %d, EYSD: %p)\n", ret, EYSD);
        print_APC_error(ret);
    }

    int nDevCount = APC_GetDeviceNumber(EYSD);
    int nDevSimpleCount = APC_GetSimpleDeviceNumber(EYSD);
    if (nDevCount <= APC_OK && nDevSimpleCount <= APC_OK) {
        if (nDevCount <= APC_OK)        CT_DEBUG("There is no USB device!\n");
        if (nDevSimpleCount <= APC_OK)  CT_DEBUG("There is no MIPI device!\n");
        return APC_NoDevice;
    }

    g_pDevInfo = (DEVINFORMATION*)malloc(sizeof(DEVINFORMATION) * MAX_TOTAL_DEV_CONT);
    g_pMipiInfo = (MipiInfo*)malloc(sizeof(MipiInfo) * MAX_TOTAL_DEV_CONT);

    CT_DEBUG("======================================================================\n");
    CT_DEBUG("nDevCount: %d\n", nDevCount);
    for(i = 0; i < nDevCount; i++) {
        CT_DEBUG("Select Index: %d\n", i);
        gsDevSelInfo.index = i;
        if (APC_OK == APC_GetDeviceInfo(EYSD, GetDevSelectIndexPtr(), g_pDevInfo + gsDevSelInfo.index)) {
            CT_DEBUG("Device Name: %s\n", g_pDevInfo[gsDevSelInfo.index].strDevName);
            if (DEBUG_LOG) CT_DEBUG("PID: 0x%04x\n", g_pDevInfo[gsDevSelInfo.index].wPID);
            if (DEBUG_LOG) CT_DEBUG("VID: 0x%04x\n", g_pDevInfo[gsDevSelInfo.index].wVID);
            if (DEBUG_LOG) CT_DEBUG("ChipID: 0x%x\n", g_pDevInfo[gsDevSelInfo.index].nChipID);
            if (DEBUG_LOG) CT_DEBUG("Device Type: %d\n", g_pDevInfo[gsDevSelInfo.index].nDevType);
        } else {
            CT_DEBUG("APC_GetDeviceInfo() Fail! (%d)\n", ret);
        }

        int nActualLength = 0;
        if (APC_OK == APC_GetFwVersion(EYSD, GetDevSelectIndexPtr(), FWVersion, 256, &nActualLength)) {
            CT_DEBUG("FW Version: %s\n", FWVersion);
            strcpy(devBuf, &g_pDevInfo[gsDevSelInfo.index].strDevName[strlen("/dev/")]);
            sprintf(devBuf_v4l, "/sys/class/video4linux/%s/name", devBuf);
            get_product_name(devBuf_v4l, devBuf_name);
        } else {
            CT_DEBUG("APC_GetFwVersion() Fail! (%d)\n", ret);
        }
    }
    CT_DEBUG("======================================================================\n");
    CT_DEBUG("nDevSimpleCount: %d\n", nDevSimpleCount);
    for (i = SIMPLE_DEV_START_IDX; i < (SIMPLE_DEV_START_IDX + nDevSimpleCount); i++) {
        CT_DEBUG("Select Index: %d\n", i);
        gsDevSelInfo.index = i;
        if (APC_OK == APC_GetDeviceInfo(EYSD, GetDevSelectIndexPtr(), g_pDevInfo + gsDevSelInfo.index)) {
            CT_DEBUG("Device Name: %s\n", g_pDevInfo[gsDevSelInfo.index].strDevName);
            if (DEBUG_LOG) CT_DEBUG("PID: 0x%04x\n", g_pDevInfo[gsDevSelInfo.index].wPID);
            if (DEBUG_LOG) CT_DEBUG("VID: 0x%04x\n", g_pDevInfo[gsDevSelInfo.index].wVID);
            if (DEBUG_LOG) CT_DEBUG("ChipID: 0x%x\n", g_pDevInfo[gsDevSelInfo.index].nChipID);
            if (DEBUG_LOG) CT_DEBUG("Device Type: %d\n", g_pDevInfo[gsDevSelInfo.index].nDevType);
        } else {
            CT_DEBUG("APC_GetDeviceInfo() Fail! (%d)\n", ret);
        }

        ret = GetMipiInfo();
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call GetMipiInfo()(%d)\n", ret);
        }

        if (g_pMipiInfo[gsDevSelInfo.index].vc_id == 0) {
            int nActualLength = 0;
            if (APC_OK == APC_GetFwVersion(EYSD, GetDevSelectIndexPtr(), FWVersion, 256, &nActualLength)) {
                CT_DEBUG("FW Version: %s\n", FWVersion);
                strcpy(devBuf, &g_pDevInfo[gsDevSelInfo.index].strDevName[strlen("/dev/")]);
                sprintf(devBuf_v4l, "/sys/class/video4linux/%s/name", devBuf);
                get_product_name(devBuf_v4l, devBuf_name);
            } else {
                CT_DEBUG("APC_GetFwVersion() Fail! (%d)\n", ret);
            }
        }
    }
    CT_DEBUG("======================================================================\n");

    //s:[eys3D] 20200615 implement ZD table
    if (g_ColorPaletteZ14 == NULL) {
        g_ColorPaletteZ14 = (RGBQUAD *)calloc(16384, sizeof(RGBQUAD));
    }
    if (g_ColorPaletteZ14 == NULL) {
        CT_DEBUG("Alloc g_ColorPaletteZ14 Fail!\n");
    }
    
    if (g_GrayPaletteZ14 == NULL) {
        g_GrayPaletteZ14 = (RGBQUAD *)calloc(16384, sizeof(RGBQUAD));
    }
    if (g_GrayPaletteZ14 == NULL) {
        CT_DEBUG("Alloc g_GrayPaletteZ14 Fail!\n");
    }

    if (is_select_dev == false) {
        CT_DEBUG("Select Index: %d\n", gsDevSelInfo.index);
        //NOTICE: The following code want that the gsDevSelInfo.index will be set as 0.
        if (gsDevSelInfo.index > 0) {
            for (int i = 0; i < gsDevSelInfo.index + 1; i++) {
                char* module = PidToModuleName(g_pDevInfo[i].wPID);
                if (module == nullptr) {
                    module = g_pDevInfo[i].strDevName;
                }
                SelectDevInx(i);
            }
        }
    } else {
        if (nDevCount > 1) {
            if (nDevSimpleCount > 0)
                printf("Please Select Device Index (0...%d):", (SIMPLE_DEV_START_IDX + nDevSimpleCount - 1));
            else
                printf("Please Select Device Index (0...%d):", (nDevCount - 1));
            scanf("%d", &gsDevSelInfo.index);
            printf("Selected Deivce Index is = [%d]\n", gsDevSelInfo.index);
            gCameraPID = g_pDevInfo[gsDevSelInfo.index].wPID;
        } else {
            gCameraPID = g_pDevInfo[gsDevSelInfo.index].wPID;
        }
    }

    if (DEBUG_LOG) CT_DEBUG("[%d]: [DevType, DevNode, PID, VID] = [%d, %s, 0x%04x, 0x%04x]\n",
             gsDevSelInfo.index,
             g_pDevInfo[gsDevSelInfo.index].nDevType,
             g_pDevInfo[gsDevSelInfo.index].strDevName,
             g_pDevInfo[gsDevSelInfo.index].wPID,
             g_pDevInfo[gsDevSelInfo.index].wVID);

    return ret;
}

static int open_device_default(bool two_open, int colorWidth, int colorHeight, int depthWidth, int depthHeight, int fps, WORD videoMode, bool blkMode)
{
    int dtc = 0, ret;
    char input[64];
    int m_output_dtc = 0;
    bool bIsMJPEG = false; //true: V4L2_PIX_FMT_MJPEG, false: V4L2_PIX_FMT_YUYV
    unsigned short DepthDataType = APC_DEPTH_DATA_8_BITS;

    if (!EYSD) {
        CT_DEBUG("EYSD is NULL!\n");
        return APC_NullPtr;
    }

    //s:[eys3D] 20200610 implement hypatia config
    int m_VideoMode = 1;
    {
        if (APC_SetupBlock(EYSD, GetDevSelectIndexPtr(), blkMode) != 0) {
            CT_DEBUG("Setup Blocking Fail!\n");
        }
        gColorFormat = bIsMJPEG;
        gColorWidth = colorWidth;
        gColorHeight = colorHeight;
        dtc = DEPTH_IMG_NON_TRANSFER; // DEPTH_IMG_NON_TRANSFER: non transfer, DEPTH_IMG_GRAY_TRANSFER: gray, DEPTH_IMG_COLORFUL_TRANSFER: colorful
        gDepthWidth = depthWidth;
        gDepthHeight = depthHeight;
        gActualFps = fps; 
        gDepth_Transfer_ctrl = (DEPTH_TRANSFER_CTRL)dtc;

        {
            gDepthDataType = videoMode;
            // TransformDepthDataType(&gDepthDataType, gRectifyData);
            if (DEBUG_LOG) CT_DEBUG("APC_SetDepthDataType(%d)\n", gDepthDataType);
            ret = APC_SetDepthDataType(EYSD, GetDevSelectIndexPtr(), gDepthDataType); //4 ==> 11 bits
            if (ret == APC_OK) {
                if (DEBUG_LOG) CT_DEBUG("APC_SetDepthData() Success!\n");
            } else {
                CT_DEBUG("APC_SetDepthData() Fail! (%d)\n", ret);
                print_APC_error(ret);
            }
            ret = APC_GetDepthDataType(EYSD, GetDevSelectIndexPtr(), &DepthDataType);
            if (ret == APC_OK) {
                if (DEBUG_LOG) CT_DEBUG("APC_GetDepthDataType() Success!\n");
            } else {
                CT_DEBUG("APC_GetDepthDataType() Fail! (%d)\n", ret);
                print_APC_error(ret);
            }
            if (DEBUG_LOG) CT_DEBUG("APC_GetDepthDataType(%d)\n", DepthDataType);
        }

        if (g_pDevInfo[gsDevSelInfo.index].wPID == 0x0124 || g_pDevInfo[gsDevSelInfo.index].wPID == 0x0147) {
            setupFWRegister_EX8038();
        }
    }

    //s:[eys3D] 20200615 implement ZD table
    ret = getZDtable(g_pDevInfo, gsDevSelInfo, gDepthHeight, gColorHeight);
    if (APC_OK != ret) {
        CT_DEBUG("update ZDtable Fail! (%d)\n", ret);
    }
    ColorPaletteGenerator::DmColorMode14(g_ColorPaletteZ14, (int)g_maxFar, (int)g_maxNear);
    ColorPaletteGenerator::DmGrayMode14(g_GrayPaletteZ14, (int)g_maxFar, (int)g_maxNear);

    if (gInterleave)
        ret = APC_Setup_v4l2_requestbuffers(EYSD, GetDevSelectIndexPtr(), V4L2_BUFFER_QUQUE_SIZE_FOR_INTERLEAVE);
    else
        ret = APC_Setup_v4l2_requestbuffers(EYSD, GetDevSelectIndexPtr(), g_v4l2_buffer_quque_size);
    if (APC_OK != ret) {
        CT_DEBUG("APC_Setup_v4l2_requestbuffers Fail!\n");
    }

    if (APC_OK != APC_SetInterleaveMode(EYSD, GetDevSelectIndexPtr(), gInterleave)) {
        CT_DEBUG("APC_SetInterleaveMode Fail!\n");
    }

    if (two_open) {
        ret = APC_OpenDevice2(EYSD,
                              GetDevSelectIndexPtr(), gColorWidth,
                              gColorHeight, (bool)gColorFormat,
                              gDepthWidth, gDepthHeight,
                              gDepth_Transfer_ctrl, false, NULL, &gActualFps, IMAGE_SN_SYNC);
        if (ret == APC_OK) {
            if (DEBUG_LOG) CT_DEBUG("APC_OpenDevice2() Success! (%d)\n", gActualFps);
        } else {
            CT_DEBUG("APC_OpenDevice2() Fail! (ret: %d, FPS: %d)\n", ret, gActualFps);
            print_APC_error(ret);
        }
    } else {
        ret= APC_OpenDevice(EYSD, GetDevSelectIndexPtr(),
                            gColorWidth, gColorHeight, (bool)gColorFormat,
                            gDepthWidth, gDepthHeight,
                            gDepth_Transfer_ctrl,
                            false, NULL,
                            &gActualFps, gRectifyData ? IMAGE_RECTIFY_DATA : IMAGE_NORECTIFY_DATA);
        if (ret == APC_OK) {
            if (DEBUG_LOG) CT_DEBUG("APC_OpenDevice() Success! (%d)\n", gActualFps);
        } else {
            CT_DEBUG("APC_OpenDevice() Fail! (ret: %d, FPS: %d)\n", ret, gActualFps);
            print_APC_error(ret);
        }
    }

    //s:[eys3D] 20200623, implement IR mode
    ret = setupIR(gSetupIRValue);//input 0xff, will use default value (min+max)/2.
    if (APC_OK != ret) {
        CT_DEBUG("Set IR mode Fail! (%d)\n", ret);
    }
    //e:[eys3D] 20200623, implement IR mode

    ret = APC_SetCTPropVal(EYSD, GetDevSelectIndexPtr(), CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, AE_MOD_APERTURE_PRIORITY_MODE);
    if (APC_OK != ret) {
        CT_DEBUG("Set Auto Exposure mode Fail! (%d)\n", ret);
    }
    
    ret = APC_SetPUPropVal(EYSD, GetDevSelectIndexPtr(), PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, PU_PROPERTY_ID_AWB_ENABLE);
    if (APC_OK != ret) {
        CT_DEBUG("Set Auto White Balance Fail! (%d)\n", ret);
    }

    return ret;
}

static int open_device(void)
{
    int dtc = 0, ret;
    char input[64];
    int m_output_dtc = 0;

    if (!EYSD) {
        CT_DEBUG("EYSD is NULL!\n");
        return APC_NullPtr;
    }

    //s:[eys3D] 20200610 implement hypatia config
    int m_VideoMode = 1;

    if (g_pDevInfo[gsDevSelInfo.index].wPID == 0x160) {
        printf("Please enter Mode:\n");
        printf("Mode 1 : L'+D (Color:MJPEG 640x400   30fps, Depth:640x400 30fps)\n");
        printf("Mode 2 : L'+D (Color:MJPEG 320x200   30fps, Depth:320x200 30fps)\n");
        printf("Mode 3 : L'+D (Color:MJPEG 320x104   30fps, Depth:320x104 30fps)\n");
        printf("Mode 4 : L'+D (Color:MJPEG 640x400x2 15fps, Depth:640x400 15fps)\n");
        printf("Mode 5 : L'+D (Color:MJPEG 320x200x2 15fps, Depth:320x200 15fps)\n");
        printf("Mode 6 : L'+D (Color:MJPEG 320x104x2 15fps, Depth:320x104 15fps)\n");
        printf("Mode 7 : L'+D (Color:YUV   640x400x2 15fps )\n");
        printf("Mode 8 : L'+D (Color:YUV   320x200x2 15fps )\n");
        printf("Mode 9 : L'+D (Color:YUV   320x104x2 15fps )\n");
        scanf("%d", &m_VideoMode);

        setHypatiaVideoMode(m_VideoMode);
    }
    //e:[eys3D] 20200610 implement hypatia config
    else {
        printf("Blocking Mode Turn On? (Yes/No)\n");
        scanf("%s", input);

        if (input[0] == 'Y' || input[0] == 'y') {
            if (APC_SetupBlock(EYSD, GetDevSelectIndexPtr(), true) != 0) {
                CT_DEBUG("Setup Blocking Fail!\n");
            }
        } else {
            if (APC_SetupBlock(EYSD, GetDevSelectIndexPtr(), false) != 0) {
                CT_DEBUG("Setup Blocking Fail!\n");
            }
        }

        printf("Set Color stream format & resolution (format = 0: YUYV, 1: MJPEG)\n");
        printf("ex> 0 1280 720 30 (YUYV  foramt, 1280 x 720, 30(FPS))\n");
        printf("ex> 1 1280 720 30 (MJPEG foramt, 1280 x 720, 30(FPS))\n");

        scanf("%d %d %d %d", &gColorFormat, &gColorWidth, &gColorHeight, &gActualFps);

        dtc = 0; // 0: non transfer, 1: gray, 2: colorful

        printf("Set Depth stream resolution\n");
        printf("ex> 640 720 30 (640 x 720, 30(FPS))\n");
        printf("ex> 320 480 30 (320 x 480, 30(FPS))\n");
        printf("ex> 0 0 30 (0 x 0, 30(FPS) if color only )\n");

        scanf("%d %d %d", &gDepthWidth, &gDepthHeight, &gActualFps);

        gDepth_Transfer_ctrl = (DEPTH_TRANSFER_CTRL)dtc;
        if (gDepthWidth != 0) {
            setupDepth();
            // TransformDepthDataType(&gDepthDataType, true);
            if (DEBUG_LOG) CT_DEBUG("APC_SetDepthDataType(%d)\n", gDepthDataType);
            ret = APC_SetDepthDataType(EYSD, GetDevSelectIndexPtr(), gDepthDataType); //4 ==> 11 bits
            if (ret == APC_OK) {
                if (DEBUG_LOG) CT_DEBUG("APC_SetDepthData() Success!\n");
            } else {
                CT_DEBUG("APC_SetDepthData() Fail! (%d)\n", ret);
                print_APC_error(ret);
            }
        }

        m_output_dtc = 2; // 0: non transfer, 1: gray, 2: colorful
        g_depth_output = (DEPTH_TRANSFER_CTRL)m_output_dtc;

        if (g_pDevInfo[gsDevSelInfo.index].wPID == 0x0124 || g_pDevInfo[gsDevSelInfo.index].wPID == 0x0147) {
            setupFWRegister_EX8038();
        }
    }
    //s:[eys3D] 20200615 implement ZD table
    ret = getZDtable(g_pDevInfo, gsDevSelInfo, gDepthHeight, gColorHeight);
    if (APC_OK != ret) {
        CT_DEBUG("Update ZDtable Fail! (%d)\n", ret);
    }
    ColorPaletteGenerator::DmColorMode14(g_ColorPaletteZ14, (int) g_maxFar, (int)g_maxNear);
    ColorPaletteGenerator::DmGrayMode14(g_GrayPaletteZ14, (int) g_maxFar, (int)g_maxNear);
    //e:[eys3D] 20200615 implement ZD table

    setV4L2buffer();
    
    /* APC_OpenDevice2 (Color + Depth) */
    ret = APC_OpenDevice2(EYSD, GetDevSelectIndexPtr(), gColorWidth, gColorHeight, (bool)gColorFormat, gDepthWidth, gDepthHeight, gDepth_Transfer_ctrl, false, NULL, &gActualFps, IMAGE_SN_SYNC);
    if (ret == APC_OK) {
        if (DEBUG_LOG) CT_DEBUG("APC_OpenDevice2() Success! (%d)\n", gActualFps);
    } else {
        CT_DEBUG("APC_OpenDevice2() Fail! (%d)\n", ret);
        print_APC_error(ret);
    }

    //s:[eys3D] 20200623, implement IR mode
    ret = setupIR(0xff);//input 0xff, will use default value (min+max)/2. 
    if (APC_OK != ret) {
        CT_DEBUG("Set IR mode Fail! (%d)\n", ret);
    }
    //e:[eys3D] 20200623, implement IR mode

    property_bar_test_func(NULL);
    return ret;
}

static void *pfunc_thread_color(void *arg) {
    int ret, mCount = 0;
    long long start_time = calcByGetTimeOfDay();
    long long current_time = 0;
    long cnt_frme = 0;
    float fps_tmp = 0, fps_total = 0;;
    
    UNUSED(arg);
    CT_DEBUG("gColorWidth: %d, gColorHeight: %d\n", gColorWidth, gColorHeight);
    if (gColorImgBuf == NULL) {
        gColorImgBuf = (unsigned char*)calloc(2 * gColorWidth * gColorHeight, sizeof(unsigned char));
    }
    if (gColorImgBuf == NULL) {
        CT_DEBUG("Alloc ColorImgBuf Fail!\n");
        return NULL;
    }
    if (DEBUG_LOG) {
        CT_DEBUG("gColorImgBuf: %p\n", gColorImgBuf);
    }

    while (bTesting_color == true && mCount < TEST_RUN_NUMBER) {
        bTestEnd_color = false;
        usleep(1000 * 5);
        /* APC_GetColorImage */
        ret = APC_GetColorImage(EYSD, GetDevSelectIndexPtr(), (BYTE*)gColorImgBuf, &gColorImgSize, &gColorSerial);
        //CT_DEBUG("[%s][%d] %d, errno: %d(%s)(%d)\n", __func__, __LINE__,ret, errno, strerror(errno), gColorSerial);
        if (ret == APC_OK) {
            if (cnt_frme == 0 && mCount == 0) {
                APC_SetCTPropVal(EYSD, GetDevSelectIndexPtr(), CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, AE_MOD_APERTURE_PRIORITY_MODE);
                APC_SetPUPropVal(EYSD, GetDevSelectIndexPtr(), PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, PU_PROPERTY_ID_AWB_ENABLE);
                setupIR(gSetupIRValue);
                usleep(1 * 1000 * 1000);
            }

            cnt_frme++;
            if (cnt_frme >= TEST_FRAME_COUNTER) {
                current_time = calcByGetTimeOfDay();
                fps_tmp = (float)(TEST_FRAME_COUNTER * 1000 * 1000) / (float)((current_time - start_time));
                cnt_frme = 0;
                CT_DEBUG("Color: FPS: %f (size: %lu, serial: %d)\n", fps_tmp, gColorImgSize, gColorSerial);
                start_time = current_time;
                mCount++;
                if (snapShot_color == true) {
                    //pthread_mutex_lock(&save_file_mutex);
                    save_file(gColorImgBuf, gColorImgSize, gColorWidth, gColorHeight, gColorFormat, true);

                    //s:[eys3D] 20200610 implement to save raw data to RGB format
                    if (gColorRGBImgBuf == NULL) {
                        gColorRGBImgBuf = (unsigned char*)calloc(3 * gColorWidth * gColorHeight, sizeof(unsigned char));
                    }

                    if (gColorRGBImgBuf == NULL) {
                        CT_DEBUG("Alloc gColorRGBImgBuf Fail!\n");
                        return NULL;
                    }

                    if (snapShot_raw) {
                        ret = APC_ColorFormat_to_RGB24(EYSD, GetDevSelectIndexPtr(), gColorRGBImgBuf, gColorImgBuf, gColorImgSize,
                                                       gColorWidth, gColorHeight,
                                                       gColorFormat ? APCImageType::Value::COLOR_MJPG : APCImageType::Value::COLOR_YUY2);

                        if (ret == APC_OK) {
                            save_file(gColorRGBImgBuf, 3 * gColorWidth * gColorHeight, gColorWidth, gColorHeight, gColorFormat, false);
                        }

                        if (gTempImgBuf != NULL) {
                            if (DEBUG_LOG) {
                                CT_DEBUG("Free gTempImgBuf: %p\n", gTempImgBuf);
                            }
                            free(gTempImgBuf);
                            gTempImgBuf = NULL;
                        }
                        
                        if (gColorRGBImgBuf != NULL) {
                            if (DEBUG_LOG) {
                                CT_DEBUG("Free gColorRGBImgBuf: %p\n", gColorRGBImgBuf);
                            }
                            free(gColorRGBImgBuf);
                            gColorRGBImgBuf = NULL;
                        }
                        //e:[eys3D] 20200610 implement to save raw data to RGB format
                        //pthread_mutex_unlock(&save_file_mutex);
                    }
                }

                if (mCount > ABORT_FRAME_COUNTER)
                    fps_total += fps_tmp;
            }
        } else {
            CT_DEBUG("APC_GetColorImage() ret = %d\n", ret);
            if (ret == APC_DEVICE_TIMEOUT) {
                /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                      */
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
    }
    bTestEnd_color = true;
    //s:[eys3D] 20200610 implement to save raw data to RGB format
    if (gColorImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gColorImgBuf: %p\n", gColorImgBuf);
        }
        free(gColorImgBuf);
        gColorImgBuf = NULL;
    }

    CT_DEBUG("### Average FPS: %f\n", (float)(fps_total / (TEST_RUN_NUMBER - ABORT_FRAME_COUNTER)));
    //e:[eys3D] 20200610 implement to save raw data to RGB format
    //pthread_exit(NULL);
    return NULL;
}

static void get_color_image(void)
{
#ifdef USE_PTHREAD_TO_GET_COLOR_AND_DEPTH_IMAGE
    pthread_t color_thread_id;
    pthread_attr_t color_thread_attr;
    struct sched_param color_thread_param;

    pthread_attr_init(&color_thread_attr);
    pthread_attr_getschedparam (&color_thread_attr, &color_thread_param);
    color_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&color_thread_attr, &color_thread_param);
    pthread_create(&color_thread_id, &color_thread_attr, pfunc_thread_color, NULL);
    gcolor_thread_id = color_thread_id;
    //pthread_join(color_thread_id, NULL);
#else
    pfunc_thread_color(NULL);
#endif
}

static void *pfunc_thread_depth(void *arg) {
    int ret;
    long long start_time = calcByGetTimeOfDay();
    long long current_time = 0;
    long cnt_frme = 0;
    float fps_tmp = 0, fps_total = 0;;
    int mCount = 0;
    unsigned long int m_BufferSize = 0;
    
    UNUSED(arg);
    CT_DEBUG("gDepthWidth: %d, gDepthHeight: %d\n", gDepthWidth, gDepthHeight);
    if (gDepthImgBuf == NULL) {
         if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
            m_BufferSize = 2 * gDepthWidth  * 2 * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        } else {
            m_BufferSize = 2 * gDepthWidth * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        }
    }
    if (gDepthImgBuf == NULL) {
        CT_DEBUG("Alloc for gDepthImageBuf Fail!\n");
        return NULL;
    }
    if (DEBUG_LOG) {
        CT_DEBUG("gDepthImgBuf: %p\n", gDepthImgBuf);
    }

    //s:[eys3D] 20200610 implement to save raw data to RGB format
    if (gDepthRGBImgBuf == NULL) {
        if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
            gDepthRGBImgBuf = (unsigned char*)calloc(2 * gDepthWidth * gDepthHeight * 3, sizeof(unsigned char));
        } else {
            gDepthRGBImgBuf = (unsigned char*)calloc(gDepthWidth * gDepthHeight * 3, sizeof(unsigned char));
        }
    }

    if (gDepthRGBImgBuf == NULL) {
        CT_DEBUG("Alloc for gDepthRGBImgBuf Fail!\n");
        return NULL;
    }
    if (DEBUG_LOG) {
        CT_DEBUG("gDepthRGBImgBuf: %p\n", gDepthRGBImgBuf);
    }
    //e:[eys3D] 20200610 implement to save raw data to RGB format

    bool bFirstReceived = true;
    while (bTesting_depth == true && mCount < TEST_RUN_NUMBER) {
        bTestEnd_depth = false;
        usleep(1000 * 5);
        ret = APC_GetDepthImage(EYSD, GetDevSelectIndexPtr(), (BYTE*)gDepthImgBuf, &gDepthImgSize, &gDepthSerial, gDepthDataType);
        if (gDepthImgSize > m_BufferSize)
        {
            CT_DEBUG("Alloc size: %lu, but get depth size: %lu, check FW and close the application.\n", m_BufferSize, gDepthImgSize);
            goto EXIT;
        }
        //CT_DEBUG("[%s][%d] %d, errno: %d(%s)(%d)\n", __func__, __LINE__,ret, errno, strerror(errno), gDepthSerial);
        if (ret == APC_OK) {
            cnt_frme++;
            if (bFirstReceived) {
                bFirstReceived = false;
                RegisterSettings::DM_Quality_Register_Setting(EYSD,
                                                              GetDevSelectIndexPtr(),
                                                              g_pDevInfo[gsDevSelInfo.index].wPID);
            }

            if (cnt_frme >= TEST_FRAME_COUNTER && gDepthSerial > 0) {
                current_time = calcByGetTimeOfDay();
                fps_tmp = (float)(TEST_FRAME_COUNTER * 1000 * 1000) / (float)((current_time - start_time));
                cnt_frme = 0;
                start_time = current_time;
                CT_DEBUG("Depth: FPS: %f (size: %lu, serial: %d, datatype: %d)\n", fps_tmp, gDepthImgSize, gDepthSerial, gDepthDataType);
                mCount++;
                if (snapShot_depth == true) {
                    //pthread_mutex_lock(&save_file_mutex);
                    save_file(gDepthImgBuf, gDepthImgSize, gDepthWidth, gDepthHeight, 2, true);
                    //s:[eys3D] 20200615 implement to save raw data to RGB format
                    if (snapShot_rgb) {
                        saveDepth2rgb(gDepthImgBuf, gDepthRGBImgBuf, gDepthWidth, gDepthHeight);
                        //e:[eys3D] 20200615 implement to save raw data to RGB format
                        //pthread_mutex_unlock(&save_file_mutex);
                    }
                }

                if (mCount > ABORT_FRAME_COUNTER)
                    fps_total += fps_tmp;
            }
        } else {
            CT_DEBUG("Failed to call APC_GetDepthImage()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                      */
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
    }
    bTestEnd_depth = true;
    //s:[eys3D] 20200610 implement to save raw data to RGB format
    if (gDepthImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gDepthImgBuf: %p\n", gDepthImgBuf);
        }
        free(gDepthImgBuf);
        gDepthImgBuf = NULL;
    }

    if (gDepthRGBImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gDepthRGBImgBuf: %p\n", gDepthRGBImgBuf);
        }
        free(gDepthRGBImgBuf);
        gDepthRGBImgBuf = NULL;
    }

    CT_DEBUG("### Average FPS: %f\n", (float)(fps_total / (TEST_RUN_NUMBER - ABORT_FRAME_COUNTER)));
    //e:[eys3D] 20200610 implement to save raw data to RGB format
    //pthread_exit(NULL);
    EXIT:
    //exit (1);
    return NULL;
}

static void get_depth_image(void)
{
#ifdef USE_PTHREAD_TO_GET_COLOR_AND_DEPTH_IMAGE
    pthread_t depth_thread_id;
    pthread_attr_t depth_thread_attr;
    struct sched_param depth_thread_param;

    pthread_attr_init(&depth_thread_attr);
    pthread_attr_getschedparam (&depth_thread_attr, &depth_thread_param);
    depth_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&depth_thread_attr, &depth_thread_param);
    pthread_create(&depth_thread_id, &depth_thread_attr, pfunc_thread_depth, NULL);
    //pthread_join(depth_thread_id, NULL);
    gdepth_thread_id = depth_thread_id;
#else
    pfunc_thread_depth(NULL);
#endif
}

static void *pfunc_thread_mipi(void *arg) {
    int ret = APC_OK;
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    long long start_time = calcByGetTimeOfDay();
    long long current_time = 0;
    long cnt_frme = 0;
    float fps_tmp = 0, fps_total = 0;
    int mCount = 0;
    uint32_t BytesPerPixel = 2; // Only support YUYV
    bool bFirstReceived = true;

    UNUSED(arg);
    if (gSplitImage) {
        CT_DEBUG("gColorWidth: %d, gColorHeight: %d\n", gColorWidth, gColorHeight);
        if (gColorImgBuf == NULL) {
            gColorImgBuf = (unsigned char*)calloc(BytesPerPixel * gColorWidth * gColorHeight, sizeof(unsigned char));
        }
    } else {
        CT_DEBUG("gColorWidth: %d, gColorHeight: %d\n", gColorWidth + gDepthWidth, gColorHeight);
        if (gColorImgBuf == NULL) {
            gColorImgBuf = (unsigned char*)calloc(BytesPerPixel * (gColorWidth + gDepthWidth) * gColorHeight, sizeof(unsigned char));
        }
    }
    if (gColorImgBuf == NULL) {
        CT_DEBUG("Alloc ColorImgBuf Fail!\n");
        return NULL;
    }
    if (DEBUG_LOG) {
        CT_DEBUG("gColorImgBuf: %p\n", gColorImgBuf);
    }

    if (gSplitImage) {
        CT_DEBUG("gDepthWidth: %d, gDepthHeight: %d\n", gDepthWidth, gDepthHeight);
        if (gDepthImgBuf == NULL) {
            gDepthImgBuf = (unsigned char*)calloc(BytesPerPixel * gDepthWidth * gDepthHeight, sizeof(unsigned char));
        }
        if (gDepthImgBuf == NULL) {
            CT_DEBUG("Alloc for gDepthImageBuf Fail!\n");
            return NULL;
        }
        if (DEBUG_LOG) {
            CT_DEBUG("gDepthImgBuf: %p\n", gDepthImgBuf);
        }
    }

    ret = SetContinueMode();
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call SetContinueMode()(%d)\n", ret);
    }

    ret = SetVirtualChannel();
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call SetVirtualChannel()(%d)\n", ret);
    }

    while (bTesting_mipi == true && mCount < TEST_RUN_NUMBER_MIPI) {
#if 0
        bool bContinueMode = false;
        int nVirtualChannelID = 0;

        ret = APC_GetHWContinueMode(EYSD, GetDevSelectIndexPtr(), &bContinueMode);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetHWContinueMode()(%d)\n", ret);
        } else {
            CT_DEBUG("Call APC_GetHWContinueMode() (%d, %s)\n", ret, bContinueMode ? "ENABLE" : "DISABLE");
        }

        ret = APC_GetHWVirtualChannel(EYSD, GetDevSelectIndexPtr(), &nVirtualChannelID);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetHWVirtualChannel()(%d)\n", ret);
        } else {
            CT_DEBUG("Call APC_GetHWVirtualChannel() (%d, %d)\n", ret, nVirtualChannelID);
        }
#endif
        bTestEnd_mipi = false;
        if (gSplitImage && (gDepthWidth != 0) && (gDepthHeight != 0))
            ret = APC_Get2ImageWithTimestamp(EYSD, GetDevSelectIndexPtr(), (BYTE*)gColorImgBuf, (BYTE*)gDepthImgBuf, &gColorImgSize, &gDepthImgSize, &gColorSerial, &gDepthSerial, gDepthDataType, &cur_tv_sec, &cur_tv_usec);
        else
            ret = APC_Get2ImageWithTimestampNoSplit(EYSD, GetDevSelectIndexPtr(), (BYTE*)gColorImgBuf, &gColorImgSize, &gColorSerial, &cur_tv_sec, &cur_tv_usec, snapShot_mipi);

        if (ret == APC_OK) {
            if (cnt_frme == 0 && mCount == 0) {
                APC_SetInterleaveMode(EYSD, GetDevSelectIndexPtr(), gInterleave);
                APC_SetCTPropVal(EYSD, GetDevSelectIndexPtr(), CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, 1 << AE_MOD_APERTURE_PRIORITY_MODE);
                APC_SetPUPropVal(EYSD, GetDevSelectIndexPtr(), PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, PU_PROPERTY_ID_AWB_ENABLE);
                setupIR(gSetupIRValue);
                usleep(1 * 1000 * 1000);
            }

            cnt_frme++;
            if (cnt_frme == TEST_FRAME_COUNTER_MIPI || (cnt_frme == (TEST_FRAME_COUNTER_MIPI - 1) && gInterleave)) {
                current_time = calcByGetTimeOfDay();
                fps_tmp = (float)(TEST_FRAME_COUNTER_MIPI * 1000 * 1000) / (float)((current_time - start_time));

                if (DEBUG_LOG) CT_DEBUG("Color: FPS: %02f (size: %lu, serial: %03d)\n", fps_tmp, gColorImgSize, gColorSerial);
                if (gSplitImage)
                    if (DEBUG_LOG) CT_DEBUG("Depth: FPS: %02f (size: %lu, serial: %03d, datatype: %d)\n", fps_tmp, gDepthImgSize, gDepthSerial, gDepthDataType);

                if (snapShot_mipi) {
                    gIgnoreDataTime = true;
                    if (gSplitImage) {
                        save_file(gColorImgBuf, gColorImgSize, gColorWidth, gColorHeight, gColorFormat, true);
                        save_file(gDepthImgBuf, gDepthImgSize, gDepthWidth, gDepthHeight, 2, true);
                    } else {
                        save_file(gColorImgBuf, gColorImgSize, gColorWidth + gDepthWidth, gColorHeight, gColorFormat, true);
                    }
                    gIgnoreDataTime = false;
                }

                start_time = current_time;
                cnt_frme = 0;
                mCount++;

                if (mCount >= ABORT_FRAME_COUNTER_MIPI)
                    fps_total += fps_tmp;
            }
        } else {
            CT_DEBUG("Failed to call APC_Get2ImageWithTimestamp()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
    }
    bTestEnd_mipi = true;

    if (gColorImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gColorImgBuf: %p\n", gColorImgBuf);
        }
        free(gColorImgBuf);
        gColorImgBuf = NULL;
    }

    if (gSplitImage) {
        if (gDepthImgBuf != NULL) {
            if (DEBUG_LOG) {
                CT_DEBUG("Free gDepthImgBuf: %p\n", gDepthImgBuf);
            }
            free(gDepthImgBuf);
            gDepthImgBuf = NULL;
        }
    }

    CT_DEBUG("### Average FPS: %f\n", (float)(fps_total / (TEST_RUN_NUMBER_MIPI - ABORT_FRAME_COUNTER_MIPI)));

    return NULL;
}

static void get_mipi_image(void)
{
#ifdef USE_PTHREAD_TO_GET_COLOR_AND_DEPTH_IMAGE
    pthread_t depth_thread_id;
    pthread_attr_t depth_thread_attr;
    struct sched_param depth_thread_param;

    pthread_attr_init(&depth_thread_attr);
    pthread_attr_getschedparam (&depth_thread_attr, &depth_thread_param);
    depth_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&depth_thread_attr, &depth_thread_param);
    pthread_create(&depth_thread_id, &depth_thread_attr, pfunc_thread_mipi, NULL);
    //pthread_join(depth_thread_id, NULL);
    gdepth_thread_id = depth_thread_id;
#else
    pfunc_thread_depth(NULL);
#endif
}

static void close_device(void)
{
/*
    pthread_t close_thread_id;
    pthread_attr_t close_thread_attr;
    struct sched_param close_thread_param;

    pthread_attr_init(&close_thread_attr);
    pthread_attr_getschedparam (&close_thread_attr, &close_thread_param);
    close_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&close_thread_attr, &close_thread_param);
    pthread_create(&close_thread_id, &close_thread_attr, pfunc_thread_close, NULL);
    pthread_join(close_thread_id, NULL);
*/
    pfunc_thread_close(NULL);
}

static void release_device(void)
{
    APC_Release(&EYSD);
    free(g_pDevInfo);
}

static void *pfunc_thread_point_cloud(void *arg) {

    int ret = APC_OK;
    
    int64_t cur_tv_sec_depth = 0;
    int64_t cur_tv_usec_depth = 0;
    int64_t cur_tv_sec_color = 0;
    int64_t cur_tv_usec_color = 0;
    
    int cur_serial_num_depth = -1;
    int cur_serial_num_color = -1;
    
    unsigned int count = 0;
    unsigned int max_count = 10;
    
    static unsigned int m_BufferSize = 0;
    
    bool bFirstReceived = true;

    int i = 0;
    unsigned char *pPointCloudRGB = NULL;
    float *pPointCloudXYZ = NULL;

    (void)arg;

    CT_DEBUG("Depth Image: [%d x %d @ %d]\n", gDepthWidth, gDepthHeight, gActualFps);
    
    pPointCloudRGB = (unsigned char *)malloc(gDepthWidth * gDepthHeight * 3 * sizeof(unsigned char));
    pPointCloudXYZ = (float *)malloc(gDepthWidth * gDepthHeight * 3 * sizeof(float));
    if ((pPointCloudRGB == NULL) || (pPointCloudXYZ == NULL)) {
        CT_DEBUG("Alloc for pPointCloudRGB or  pPointCloudXYZ Fail!\n");
        goto EXIT;
    }
    
    if (gDepthImgBuf == NULL) {
         if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
                m_BufferSize = 2 * gDepthWidth  * 2 * gDepthHeight;
                gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        } else {
                m_BufferSize = gDepthWidth * gDepthHeight * 2;
                gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        }
    }
    if (gDepthImgBuf == NULL) {
        CT_DEBUG("Alloc for gDepthImageBuf Fail!\n");
         goto EXIT;
    }

    CT_DEBUG("Color Image: [%d x %d @ %d]\n", gColorWidth, gColorHeight, gActualFps);
    if (gColorImgBuf == NULL) {
        gColorImgBuf = (unsigned char*)calloc(2 * gColorWidth * gColorHeight, sizeof(unsigned char));
    }
    if (gColorImgBuf == NULL) {
        CT_DEBUG("Alloc ColorImgBuf Fail!\n");
         goto EXIT;
    }
    
    if (gColorRGBImgBuf == NULL) {
        gColorRGBImgBuf = (unsigned char*)calloc(3 * gColorWidth * gColorHeight, sizeof(unsigned char));
    }
    if (gColorRGBImgBuf == NULL) {
        CT_DEBUG("Alloc gColorRGBImgBuf Fail!\n");
         goto EXIT;
    }

    while (count < max_count) {
        ret = APC_GetColorImageWithTimestamp(EYSD, GetDevSelectIndexPtr(),
                                             (BYTE*)gColorImgBuf, &gColorImgSize, &cur_serial_num_color, 0, &cur_tv_sec_color, &cur_tv_usec_color);
       if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetColorImageWithTimestamp()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                        /*
                            When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                            So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                        */
                        CT_DEBUG("Getting image is timeout!\n");
                        usleep(1 * 1000);
            }
            goto CONTINUE;
            
        }
        
        ret = APC_GetDepthImageWithTimestamp(EYSD, GetDevSelectIndexPtr(),
                                             (BYTE*)gDepthImgBuf, &gDepthImgSize, &cur_serial_num_depth,
                                             gDepthDataType, &cur_tv_sec_depth, &cur_tv_usec_depth);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetDepthImageWithTimestamp()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                        /*
                            When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                            So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                        */
                        CT_DEBUG("Getting image is timeout!\n");
                        usleep(1 * 1000);
            }
            goto CONTINUE;
            
        }

        if (gDepthImgSize > m_BufferSize) {
            CT_DEBUG("Alloc size: %lu, but get depth size: %lu, check FW and close the application.\n", m_BufferSize, gDepthImgSize);
            break;
        }
        
        if (ret == APC_OK) {
            if (bFirstReceived) {
                bFirstReceived = false;
                RegisterSettings::DM_Quality_Register_Setting(EYSD, GetDevSelectIndexPtr(), g_pDevInfo[gsDevSelInfo.index].wPID);
            }
            ret = APC_ColorFormat_to_RGB24(EYSD, GetDevSelectIndexPtr(), gColorRGBImgBuf, gColorImgBuf, gColorImgSize,
                                             gColorWidth, gColorHeight,
                                             gColorFormat ? APCImageType::Value::COLOR_MJPG : APCImageType::Value::COLOR_YUY2);
            // convert_yuv_to_rgb_buffer(gColorImgBuf, gColorRGBImgBuf, gColorWidth, gColorHeight, false);
            ret = getPointCloud(EYSD, GetDevSelectIndexPtr(), gColorRGBImgBuf, gColorWidth, gColorHeight,
                                gDepthImgBuf, gDepthWidth, gDepthHeight, gDepthDataType,
                                pPointCloudRGB, pPointCloudXYZ, g_maxNear, g_maxFar);
        }
CONTINUE:
        count++;
    }

EXIT:
    if (gDepthImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free gDepthImgBuf: %p\n", gDepthImgBuf);
        }
        free(gDepthImgBuf);
        gDepthImgBuf = NULL;
    }

    if (pPointCloudRGB != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free pPointCloudRGB: %p\n", pPointCloudRGB);
        }
        free(pPointCloudRGB);
        pPointCloudRGB = NULL;
    }

    if (pPointCloudXYZ != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free pPointCloudXYZ: %p\n", pPointCloudXYZ);
        }
        free(pPointCloudXYZ);
        pPointCloudXYZ = NULL;
    }

    if (gColorRGBImgBuf != NULL) {
        if (DEBUG_LOG) {
            CT_DEBUG("Free pPointCloudXYZ: %p\n", gColorRGBImgBuf);
        }  
        free(gColorRGBImgBuf);
        gColorRGBImgBuf = NULL;
    }

    return NULL;
}

static void get_point_cloud_8063(void) {
    int ret = APC_OK;
    constexpr int kKolorDeviceIndex = 1;
    constexpr int kDepthDeviceIndex = 0;
    constexpr unsigned RGB_SIZE = 3;
    PointCloudInfo pointCloudInfo;
    float fNear = g_maxNear;
    float fFar = g_maxFar;
    DEVSELINFO kolorDeviceSelInf;
    DEVSELINFO depthDeviceSelInf;
    kolorDeviceSelInf.index = kKolorDeviceIndex;
    depthDeviceSelInf.index = kDepthDeviceIndex;

    // Depth
    gDepthDataType = APC_DEPTH_DATA_14_BITS;
    gDepthWidth = 1280;
    gDepthHeight = 720;
    gColorFormat = false; /* YUYV */
    gActualFps = 30;
    ret = APC_SetDepthDataType(EYSD, &depthDeviceSelInf, gDepthDataType);
    CT_DEBUG("Setup f0 Depth: %d\n", gDepthDataType);
    if (APC_SetupBlock(EYSD, &depthDeviceSelInf, true) != 0) {
        CT_DEBUG("Setup Blocking Failed Depth\n");
    }
    ret = APC_OpenDevice2(EYSD, &depthDeviceSelInf, 0, 0, gColorFormat, gDepthWidth, gDepthHeight,
                    gDepth_Transfer_ctrl, false, NULL, &gActualFps, IMAGE_SN_SYNC);
    CT_DEBUG("APC_OpenDevice2() 000 ret %d!\n", ret);

    // Kolor
    gColorWidth = 1280;
    gColorHeight = 720;
    gColorFormat = false; /* YUYV */
    int kolorDepthDataType = APC_DEPTH_DATA_OFF_RECTIFY;
    ret = APC_SetDepthDataType(EYSD, &kolorDeviceSelInf, kolorDepthDataType);
    CT_DEBUG("Setup f0 Kolor: %d\n", kolorDepthDataType);

    if (APC_SetupBlock(EYSD, &kolorDeviceSelInf, true) != 0) {
        CT_DEBUG("Setup Blocking Failed Kolor\n");
    }
    ret = APC_OpenDevice2(EYSD, &kolorDeviceSelInf, gColorWidth, gColorHeight, gColorFormat, 0, 0,
                    gDepth_Transfer_ctrl, false, NULL, &gActualFps, IMAGE_SN_SYNC);
    CT_DEBUG("APC_OpenDevice2() 001 ret %d!\n", ret);

    ret = getZDtable(g_pDevInfo, depthDeviceSelInf, gDepthHeight, gColorHeight);
    if (APC_OK != ret) {
        CT_DEBUG("update ZDtable Fail! (%d)\n", ret);
    }

    if (gDepthImgBuf == NULL) {
        gDepthImgBuf = (unsigned char *) calloc(gDepthWidth * gDepthHeight * 2, sizeof(unsigned char));
    }

    if (gColorImgBuf == NULL) {
        gColorImgBuf = (unsigned char *) calloc(gColorWidth * gColorHeight * 2, sizeof(unsigned char));
    }

    auto pPointCloudRGB = new unsigned char[gColorWidth * gColorHeight * 3 * sizeof(unsigned char)];
    auto pPointCloudXYZ = new float[gDepthWidth * gDepthHeight * 3 * sizeof(float)];

    unsigned short ddt = 0;

    ret = APC_GetDepthDataType(EYSD, &kolorDeviceSelInf, &ddt);
    CT_DEBUG("Read back Setup f0 Kolor: %d\n", ddt);
    ret = APC_GetDepthDataType(EYSD, &depthDeviceSelInf, &ddt);
    CT_DEBUG("Read back Setup f0 Depth: %d\n", ddt);
    std::thread t_depth = std::thread([&]() {
        int ret = APC_Init_Fail;
        CT_DEBUG("+++++++++++APC_GetDepthImage index%d ret=[%d] dSN[%d]\n", depthDeviceSelInf.index, ret, gDepthSerial);
        constexpr unsigned short kMaxFrameCount = 20;
        for (int i = 0; i < kMaxFrameCount || gDepthSerial < kMaxFrameCount; ++i) {
            ret = APC_GetDepthImage(EYSD, &depthDeviceSelInf, gDepthImgBuf, &gDepthImgSize, &gDepthSerial);
            CT_DEBUG("APC_GetDepthImage ret=[%d] dSN[%d]\n", ret, gDepthSerial);
            usleep(5000);
        }
    });

    sleep(10);

    constexpr unsigned short kMaxFrameCount = 15;
    for (int i = 0; i < kMaxFrameCount || gColorSerial < kMaxFrameCount; ++i) {
        ret = APC_GetColorImage(EYSD, &kolorDeviceSelInf, gColorImgBuf, &gColorImgSize, &gColorSerial);
        CT_DEBUG("APC_GetColorImage index%d ret=[%d] cSN[%d]\n", kolorDeviceSelInf.index, ret, gColorSerial);
        usleep(5000);
    }

    t_depth.join();


    BYTE* rgbBuffer = new BYTE[gColorWidth * gColorHeight * RGB_SIZE];
    std::thread t_rgb = std::thread([&]() {
        int ret = APC_Init_Fail;
        ret = APC_ColorFormat_to_RGB24(EYSD, &kolorDeviceSelInf, rgbBuffer, gColorImgBuf, gColorImgSize, gColorWidth, gColorHeight,
                                 APCImageType::COLOR_YUY2);
        CT_DEBUG("+++++++++++%d APC_ColorFormat_to_RGB24 ret=[%d]\n", t_rgb.get_id(), ret);
    });

    CT_DEBUG("sleep\n");
    sleep(3);
    CT_DEBUG("APC_GetPointCloud start\n");

    std::thread t_pcl = std::thread([&]() {
        int ret = APC_Init_Fail;
        ret = getPointCloudInfo(EYSD, &depthDeviceSelInf, &pointCloudInfo, gDepthDataType, gDepthHeight);
        APC_GetPointCloud(EYSD, &depthDeviceSelInf, rgbBuffer, gColorWidth, gColorHeight, gDepthImgBuf, gDepthWidth, gDepthHeight,
                          &pointCloudInfo, pPointCloudRGB, pPointCloudXYZ, fNear, fFar);
        CT_DEBUG("+++++++++++%d APC_GetPointCloud ret=[%d]\n", t_pcl.get_id(), ret);
    });

    t_pcl.join();
    t_rgb.join();

    delete[] rgbBuffer;
    delete[] gDepthImgBuf;
    delete[] gColorImgBuf;
    delete[] pPointCloudRGB;
    delete[] pPointCloudXYZ;

    while (1) {
        sleep(1);
        ret = APC_CloseDevice(EYSD, &kolorDeviceSelInf);
        if (ret == APC_OK) {
            if (DEBUG_LOG) CT_DEBUG("APC_CloseDevice() Success!\n");
        } else {
            CT_DEBUG("APC_CloseDevice() Fail! (%d)\n", ret);
            error_msg(ret);
            continue;
        }
        break;
    }
    while (1) {
        sleep(1);
        ret = APC_CloseDevice(EYSD, &depthDeviceSelInf);
        if (ret == APC_OK) {
            if (DEBUG_LOG) CT_DEBUG("APC_CloseDevice() Success!\n");
        } else {
            CT_DEBUG("APC_CloseDevice() Fail! (%d)\n", ret);
            error_msg(ret);
            continue;
        }
        break;
    }
}

static void get_point_cloud(void)
{
    pthread_t point_cloud_thread_id;
    pthread_attr_t point_cloud_thread_attr;
    struct sched_param point_cloud_thread_param;

    pthread_attr_init(&point_cloud_thread_attr);
    pthread_attr_getschedparam (&point_cloud_thread_attr, &point_cloud_thread_param);
    point_cloud_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&point_cloud_thread_attr, &point_cloud_thread_param);
    pthread_create(&point_cloud_thread_id, &point_cloud_thread_attr, pfunc_thread_point_cloud, NULL);
    pthread_join(point_cloud_thread_id, NULL);
}

static int getPointCloudInfo(void *pHandleEYSD, DEVSELINFO *pDevSelInfo, PointCloudInfo *pointCloudInfo, int depthDataType, int depthHeight)
{
    float ratio_Mat = 0.0f;
    float baseline  = 0.0f;
    float diff      = 0.0f;
    eSPCtrl_RectLogData rectifyLogData;
    eSPCtrl_RectLogData *pRectifyLogData = NULL;
    int nIndex = g_zdTableInfo_index;
    int ret = APC_NoDevice;
    
    if (!pointCloudInfo) {
        return -EINVAL;
    }

    memset(&rectifyLogData, 0x0, sizeof(eSPCtrl_RectLogData));
    ret = APC_GetRectifyMatLogData(pHandleEYSD, pDevSelInfo, &rectifyLogData, nIndex);
    if (ret == APC_OK) {
        pRectifyLogData = &rectifyLogData;
        ratio_Mat = (float)depthHeight / pRectifyLogData->OutImgHeight;
        baseline  = 1.0f / pRectifyLogData->ReProjectMat[14];
        diff      = pRectifyLogData->ReProjectMat[15] * ratio_Mat;
        
        memset(pointCloudInfo, 0, sizeof(PointCloudInfo));
        pointCloudInfo->wDepthType = depthDataType;
        
        pointCloudInfo->centerX = -1.0f * pRectifyLogData->ReProjectMat[3] * ratio_Mat;
        pointCloudInfo->centerY = -1.0f * pRectifyLogData->ReProjectMat[7] * ratio_Mat;
        pointCloudInfo->focalLength = pRectifyLogData->ReProjectMat[11] * ratio_Mat;
        pointCloudInfo->bIsMIPISplit = gSplitImage;

        pointCloudInfo->fx1 = pRectifyLogData->NewCamMat1[0];
        pointCloudInfo->fy1 = pRectifyLogData->NewCamMat1[5];
        pointCloudInfo->fx2 = pRectifyLogData->NewCamMat2[0];
        pointCloudInfo->fy2 = pRectifyLogData->NewCamMat2[5];
        pointCloudInfo->cx1 = pRectifyLogData->NewCamMat1[2];
        pointCloudInfo->cy1 = pRectifyLogData->NewCamMat1[6];
        pointCloudInfo->cx2 = pRectifyLogData->NewCamMat2[2];
        pointCloudInfo->cy2 = pRectifyLogData->NewCamMat2[6];
        pointCloudInfo->Tx = -1 * baseline;

        switch (APCImageType::DepthDataTypeToDepthImageType(depthDataType)) {
            case APCImageType::DEPTH_14BITS: pointCloudInfo->disparity_len = 0; break;
            case APCImageType::DEPTH_11BITS:
            {
                pointCloudInfo->disparity_len = 2048;
                for(int i = 0 ; i < pointCloudInfo->disparity_len ; ++i) {
                    pointCloudInfo->disparityToW[i] = ( i * ratio_Mat / 8.0f ) / baseline + diff;
                }
                break;
            }
            default:
                pointCloudInfo->disparity_len = 256;
                for(int i = 0 ; i < pointCloudInfo->disparity_len ; ++i) {
                pointCloudInfo->disparityToW[i] = (i * ratio_Mat) / baseline + diff;
                }
                break;
        }
    }
    return APC_OK;
}


int getPointCloud(void *pHandleEYSD, DEVSELINFO *pDevSelInfo, unsigned char *ImgColor, int CW, int CH,
                  unsigned char *ImgDepth, int DW, int DH, int depthDataType,
                  unsigned char *pPointCloudRGB,
                  float *pPointCloudXYZ,
                  float fNear,
                  float fFar)
{
    int ret = APC_OK;
    PointCloudInfo pointCloudInfo;
    std::vector<CloudPoint> cloud;
    char DateTime[32] = {0};
    static unsigned int yuv_index = 0;
    char fname[256] = {0};
    int i = 0;
    CloudPoint cloudpoint = {0};
    
    memset(DateTime, 0, sizeof(DateTime));
    GetDateTime(DateTime);
    
    ret = getPointCloudInfo(pHandleEYSD, pDevSelInfo, &pointCloudInfo, depthDataType, DH);
    if (ret == APC_OK) {
        ret = APC_GetPointCloud(pHandleEYSD, pDevSelInfo, ImgColor, CW, CH, ImgDepth, DW, DH,
                                &pointCloudInfo, pPointCloudRGB, pPointCloudXYZ, fNear, fFar);
        if (ret == APC_OK) {
            snprintf(fname, sizeof(fname), SAVE_FILE_PATH"cloud_%d_%s.ply", yuv_index++, DateTime);
           
            while(i < (DW * DH * 3)) {
                
                if (isnan(pPointCloudXYZ[i]) || isnan(pPointCloudXYZ[i+1]) || isnan(pPointCloudXYZ[i+2])) {
                    //Do nothing!!
                } else {
                    cloudpoint.r = pPointCloudRGB[i];
                    cloudpoint.g = pPointCloudRGB[i + 1];
                    cloudpoint.b = pPointCloudRGB[i + 2];
                    cloudpoint.x = pPointCloudXYZ[i];
                    cloudpoint.y = pPointCloudXYZ[i + 1];
                    cloudpoint.z = pPointCloudXYZ[i+2];
                    cloud.push_back(cloudpoint);
                }
                i+=3;
                if (i == (DW * DH * 3))
                    break;
            }
            PlyWriter::writePly(cloud, fname);
        }
    }
    
    return ret;
}


//s:[eys3D] 20200610 implement to save raw data to RGB format
static void SetSnapShotFlag()
{
    char input[64];
    printf("Enable color image snapshot? (Yes or No)\n");
    scanf("%s", input);
    
    if (input[0] == 'Y' || input[0] == 'y') {
        if (gColorWidth > 0)
            snapShot_color = true;
    } else {
        snapShot_color = false;
    }
       
    printf("Enable depth image snapshot? (Yes or No)\n");
    scanf("%s", input);     
    if (input[0] == 'Y' || input[0] == 'y') {
        if (gDepthWidth > 0)
            snapShot_depth = true;
    } else {
        snapShot_depth = false;
    }
}
//e:[eys3D] 20200610 implement to save raw data to RGB format

static void setupFWRegister_EX8038()
{
    int flag = 0;

    flag |= FG_Address_1Byte;
    flag |= FG_Value_1Byte;

    if (APC_OK != APC_SetFWRegister(EYSD, GetDevSelectIndexPtr(), 0xf0, 0x0d, flag)) {
        CT_DEBUG("APC_SetFWRegister Fail!\n");
    }
}

static void setupFWRegister()
{
    int flag = 0;
    static unsigned char addr;
    unsigned char value;
    
    flag |= FG_Address_1Byte;
    flag |= FG_Value_1Byte;

    printf("Please input one byte FW address: 0x");
    scanf("%02x", (int *)&addr);

    printf("Please input one byte FW setup value: 0x");
    scanf("%02x", (int *)&value);

    CT_DEBUG("Write address: 0x%02x, value: 0x%02x\n", addr, value);
    if (APC_OK != APC_SetFWRegister(EYSD, GetDevSelectIndexPtr(), addr, value, flag)) {
        CT_DEBUG("APC_SetFWRegister Fail!\n");
    }
}

static void readFWRegister(void)
{
    int flag = 0;
    static unsigned char  addr;
    unsigned char value;
    
    flag |= FG_Address_1Byte;
    flag |= FG_Value_1Byte;

    printf("Please input one byte FW address: 0x");
    scanf("%02x", (int *)&addr);

    if (APC_OK != APC_GetFWRegister(EYSD, GetDevSelectIndexPtr(), addr, (unsigned short *)&value, flag)) {
        CT_DEBUG("APC_GetFWRegister Fail!\n");
    }else {
        CT_DEBUG("FW addr 0x%02x: 0x%02x\n", addr, value);
    }
}

static void SetCounterMode()
{
    int nValue;

    printf("Please input value: 0: for Frame Counter, 1: For Serial Counter===> ");
    scanf("%02x", (int *)&nValue);

    if (APC_OK != APC_SetControlCounterMode(EYSD, GetDevSelectIndexPtr(), (unsigned char)nValue)) {
        CT_DEBUG("APC_SetControlCounterMode Fail!\n");
        return;
    }
    if (DEBUG_LOG) CT_DEBUG("Setup Success!\n");
}

static void GetCounterMode()
{
    unsigned char nValue;

    if (APC_OK != APC_GetControlCounterMode(EYSD, GetDevSelectIndexPtr(), &nValue)) {
        CT_DEBUG("APC_GetControlCounterMode Fail!\n");
        return;
    } else {
        CT_DEBUG("ControlCounterMode: %d\n", nValue);
        if (nValue == 1)
            CT_DEBUG("ControlCounterMode: Serial Mode\n");
        else
            CT_DEBUG("ControlCounterMode: Frame Mode\n");
    }
}

static void setV4L2buffer()
{
    int value;

    printf("Please input V4L2 buffer size: (5~32)");
    scanf("%d", (int *)&value);

    g_v4l2_buffer_quque_size = value;
    CT_DEBUG("Set V4L2 buffer size: %d\n", g_v4l2_buffer_quque_size);
    if (APC_OK != APC_Setup_v4l2_requestbuffers(EYSD, GetDevSelectIndexPtr(), g_v4l2_buffer_quque_size)) {
        CT_DEBUG("APC_Setup_v4l2_requestbuffers Fail!\n");
    }
}

//s:[eys3D] 20200623, for IR mode
static void setIRValue()
{
    unsigned short value;
    int ret;
    unsigned short m_nIRMax, m_nIRMin;
    
    ret = APC_GetFWRegister(EYSD, GetDevSelectIndexPtr(),
                                0xE2, &m_nIRMax,FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret || m_nIRMax == 0xff) {
        CT_DEBUG("get IR Max value Fail!\n");
        return;
     }

    ret = APC_GetFWRegister(EYSD, GetDevSelectIndexPtr(),
                                0xE1, &m_nIRMin,FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret|| m_nIRMin == 0xff) {
        CT_DEBUG("get IR Min value Fail!\n");
        return;
     }
    printf("IR range: %d ~ %d\n", m_nIRMin, m_nIRMax);      
    printf("Please input IR value: ");
    scanf("%hu", &value);

    if (APC_OK != setupIR(value)) {
        CT_DEBUG("APC_SetFWRegister Fail!\n");
    }
}

static void Read3X(void)
{
    int index;
    int nbfferLength = APC_Y_OFFSET_FILE_SIZE;
    int pActualLength = 0;
    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);
    int group = 1;
    int CheckSupportFP = APC_GetFlashProtectionSupported(EYSD, GetDevSelectIndexPtr());
    if (CheckSupportFP == APC_OK){
        printf("This camera support FW protection, which group you want to read? \n");
        printf("Please enter Group: 1 or 2\n");
        scanf("%d", &group);
        printf("group is %d\n",group);
        if (group < 1 || group > 2)
        {
            printf("Wrong number, Please enter Group: 1 or 2\n");
            return;
        }
    }
    else if (CheckSupportFP == APC_NotSupport){
        printf("This camera not support FW protection, read group1 data \n");
    }
    else{
        printf("CheckSupportFP fail\n");
        return;
    }

    if (group == 2)
    {
        for (index = 5; index <= 9; index++)
        {
            if (APC_OK == APC_GetYOffset(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
            {
                printf("\n Read3%d\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char xyoffsetfileName[256] = {0};
                printf("read_file name still 0~4, because need feed to K tool\n");
                sprintf(xyoffsetfileName, "calibrationdata/read/read_fileio_buffer_xyoffset_%d.bin", index - FW_FID_GROUP_OFFSET);
                std::ofstream file(xyoffsetfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read3%d Fail!\n", index);
            }
        }
    }
    else if (group == 1)
    {
        for (index = 0; index <= 5; index++)
        {
            if (APC_OK == APC_GetYOffset(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
            {
                printf("\n Read3%d\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char xyoffsetfileName[256] = {0};
                sprintf(xyoffsetfileName, "calibrationdata/read/read_fileio_buffer_xyoffset_%d.bin", index);
                std::ofstream file(xyoffsetfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read3%d Fail!\n", index);
            }
        }
    }
    delete[] data;
}
static void Write3X(void)
{
    int index;
    int nbfferLength = APC_Y_OFFSET_FILE_SIZE;
    int pActualLength = 0;
    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);
    for (index = 0; index <= 9; index++)
    {
        if (APC_OK == APC_GetYOffset(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
        {
            if (APC_OK == APC_SetYOffset(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
            {
                printf("Write3%d Success!\n", index);
            }
            else
            {
                printf("Write3%d Fail!\n", index);
            }
        }
    }

    delete[] data;
}
static void Read4X(void)
{
    int index;
    int nbfferLength = APC_RECTIFY_FILE_SIZE;
    int pActualLength = 0;
    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);
    int group = 1;
    int CheckSupportFP = APC_GetFlashProtectionSupported(EYSD, GetDevSelectIndexPtr());
    if (CheckSupportFP == APC_OK){
        printf("This camera support FW protection, which group you want to read? \n");
        printf("Please enter Group: 1 or 2\n");
        scanf("%d", &group);
        if (group < 1 || group >2)
        {
            printf("Wrong number, Please enter Group: 1 or 2\n");
            return;
        }
    }
    else if (CheckSupportFP == APC_NotSupport){
        printf("This camera not support FW protection, read group1 data \n");
    }
    else{
        printf("CheckSupportFP fail\n");
        return;
    }

    if (group == 2)
    {
        for (index = 5; index <= 9; index++)
        {
            if (APC_OK == APC_GetRectifyTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
            {
                printf("\n Read4%d\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char recyifyfileName[256] = {0};
                printf("read_file name still 0~4, because need feed to K tool\n");
                sprintf(recyifyfileName, "calibrationdata/read/read_fileio_buffer_rectify_%d.bin", index - FW_FID_GROUP_OFFSET);
                std::ofstream file(recyifyfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read4%d Fail!\n", index);
            }
        }
    }
    else if (group == 1)
    {
        for (index = 0; index <= 5; index++)
        {
            if (APC_OK == APC_GetRectifyTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
            {
                printf("\n Read4%d\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char recyifyfileName[256] = {0};
                sprintf(recyifyfileName, "calibrationdata/read/read_fileio_buffer_rectify_%d.bin", index);
                std::ofstream file(recyifyfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read4%d Fail!\n", index);
            }
        }
    }
    delete[] data;
}

static void Write4X(void)
{
    int index;
    int nbfferLength = APC_RECTIFY_FILE_SIZE;
    int pActualLength = 0;
    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);
    for (index = 0; index <= 9; index++)
    {
        if (APC_OK == APC_GetRectifyTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
        {
            if (APC_OK == APC_SetRectifyTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
            {
                printf("Write4%d Success!\n", index);
            }
            else
            {
                printf("Write4%d Fail!\n", index);
            }
        }
    }
    delete[] data;
}

static void Read5X(void)
{
    int index;
    int nbfferLength = APC_ZD_TABLE_FILE_SIZE_11_BITS;
    int pActualLength = 0;

    ZDTABLEINFO zdTableInfo;
    zdTableInfo.nDataType = APC_DEPTH_DATA_11_BITS;

    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);
    int group = 1;
    int CheckSupportFP = APC_GetFlashProtectionSupported(EYSD, GetDevSelectIndexPtr());
    if (CheckSupportFP == APC_OK){
        printf("This camera support FW protection, which group you want to read? \n");
        printf("Please enter Group: 1 or 2\n");
        scanf("%d", &group);
        if (group < 1 || group > 2)
        {
            printf("Wrong number, Please enter Group: 1 or 2\n");
            return;
        }
    }
    else if (CheckSupportFP == APC_NotSupport){
        printf("This camera not support FW protection, read group1 data \n");
    }
    else{
        printf("CheckSupportFP fail\n");
        return;
    }

    if (group == 2)
    {
        for (index = 5; index <= 9; index++)
        {
            zdTableInfo.nIndex = index;
            if (APC_OK == APC_GetZDTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, &zdTableInfo))
            {
                printf("\n Read5%d\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char zdtablefileName[256] = {0};
                printf("read_file name still 0~4, because need feed to K tool\n");
                sprintf(zdtablefileName, "calibrationdata/read/read_fileio_buffer_zdtable_%d.bin", index - FW_FID_GROUP_OFFSET);
                std::ofstream file(zdtablefileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read5%d Fail!\n", index);
            }
        }
    }
    else if (group == 1)
    {
        for (index = 0; index <= 5; index++)
        {
            zdTableInfo.nIndex = index;
            if (APC_OK == APC_GetZDTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, &zdTableInfo))
            {
                printf("\n Read5%d\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char zdtablefileName[256] = {0};
                sprintf(zdtablefileName, "calibrationdata/read/read_fileio_buffer_zdtable_%d.bin", index);
                std::ofstream file(zdtablefileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read5%d Fail!\n", index);
            }
        }
    }
    delete[] data;
}

static void Write5X(void)
{
    int index;
    int nbfferLength = APC_ZD_TABLE_FILE_SIZE_11_BITS;
    int pActualLength = 0;

    ZDTABLEINFO zdTableInfo;
    zdTableInfo.nDataType = APC_DEPTH_DATA_11_BITS;

    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);

    for (index = 0; index <= 9; index++)
    {
        zdTableInfo.nIndex = index;
        if (APC_OK == APC_GetZDTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, &zdTableInfo))
        {
            if (APC_OK == APC_SetZDTable(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, &zdTableInfo))
            {
                printf("Write5%d Success!\n", index);
            }
            else
            {
                printf("Write5%d Fail!\n", index);
            }
        }
    }
    delete[] data;
}

static void Read24X(void)
{
    int index;
    int nbfferLength = APC_CALIB_LOG_FILE_SIZE;
    int pActualLength = 0;

    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);
    int group = 1;
    int CheckSupportFP = APC_GetFlashProtectionSupported(EYSD, GetDevSelectIndexPtr());
    if (CheckSupportFP == APC_OK){
        printf("This camera support FW protection, which group you want to read? \n");
        printf("Please enter Group: 1 or 2\n");
        scanf("%d", &group);
        if (group < 1 || group > 2)
        {
            printf("Wrong number, Please enter Group: 1 or 2\n");
            return;
        }
    }
    else if (CheckSupportFP == APC_NotSupport){
        printf("This camera not support FW protection, read group1 data \n");
    }
    else{
        printf("CheckSupportFP fail\n");
        return;
    }

    if (group == 2)
    {
        for (index = 5; index <= 9; index++)
        {
            if (APC_OK == APC_GetLogData(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index, ALL_LOG))
            {
                printf("\n Read24%d ALL_LOG\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char logfileName[256] = {0};
                printf("read_file name still 0~4, because need feed to K tool\n");
                sprintf(logfileName, "calibrationdata/read/read_fileio_buffer_log_%d.bin", index - FW_FID_GROUP_OFFSET);
                std::ofstream file(logfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read24%d ALL_LOG Fail!\n", index);
            }
        }
    }
    else if (group == 1)
    {
        for (index = 0; index <= 5; index++)
        {
            if (APC_OK == APC_GetLogData(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index, ALL_LOG))
            {
                printf("\n Read24%d ALL_LOG\n", index);
                for (int i = 0; i < nbfferLength; i++)
                {
                    printf("%02x ", data[i]);
                }
                printf("\n");
                char logfileName[256] = {0};
                sprintf(logfileName, "calibrationdata/read/read_fileio_buffer_log_%d.bin", index);
                std::ofstream file(logfileName, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<char*>(data), nbfferLength);
                    file.close();
                }
            }
            else
            {
                printf("\n Read24%d ALL_LOG Fail!\n", index);
            }
        }
    }
    delete[] data;
}

static void ResetUNPData(void)
{
    int ret = APC_OK;

    ret = APC_ResetUNPData(EYSD, GetDevSelectIndexPtr());
    if (ret != APC_OK) {
        printf("Faield to call APC_ResetUNPData()\n");
    }

    return;
}

static void ReadPlugIn(void)
{

    int ret = APC_OK;
    BYTE *pBuffer = NULL;
    unsigned long int BufferLength = 1024*56;
    unsigned long int ActualLength = 0;
    BYTE *PluginHeader = NULL;
    BYTE *STI = NULL;
    char Signature[8] = {0}; //0x00 in header of plugin, 8 bytes
    unsigned short STI_Offset = 0; //0x0a in header of plugin, 2 bytes
    unsigned char StructLen = 0; //0x00 in STI, 1 byte
    unsigned short UnpAreaStartSec = 0; //0x0c in STI, 2byte

    pBuffer = (BYTE *) malloc(BufferLength);
    if (!pBuffer) {
         printf("Faield to allocate buffer with size [%lu].\n", BufferLength);
         return;
    }

    ret = APC_ReadFlashData(EYSD, GetDevSelectIndexPtr(), PLUGIN_ONLY, pBuffer, BufferLength, &ActualLength);
    if (ret != APC_OK) {
         printf("Faield to call APC_ReadFlashData()\n");
         goto EXIT;
    }

    PluginHeader = pBuffer;
    printf("Plugin:\n");
    memcpy(Signature, PluginHeader, 8);
    printf("\tSignature: %s\n", Signature);
    STI_Offset = (PluginHeader[0x0a] << 8) | PluginHeader[0x0b];
    printf("\tSTI_Offset: 0x%04x\n", STI_Offset);
    STI = (BYTE *)(pBuffer + STI_Offset);
    StructLen = *((BYTE *)(STI + 0x00));
    printf("\tStructLen: 0x%02x(%u)\n", StructLen, StructLen);
    UnpAreaStartSec = (STI[0x0c] << 8) | STI[0x0d];
    printf("\tUnpAreaStartSec: 0x%04x(%u)\n", UnpAreaStartSec, UnpAreaStartSec);

EXIT:
    if (pBuffer) {
        free(pBuffer);
        pBuffer = NULL;
    }

    return;
}

static void *pfunc_thread_point_cloud_fps(void *arg) {
    int ret = APC_OK;
    PointCloudInfo pointCloudInfo = {0};
    getPointCloudInfo(EYSD, GetDevSelectIndexPtr(), &pointCloudInfo, gDepthDataType, gDepthHeight);
    if (gColorImgBuf == NULL) {
        gColorImgBuf = (unsigned char*) calloc(2 * gColorWidth * gColorHeight, sizeof(unsigned char));
    }
    if (gColorImgBuf == NULL) {
        CT_DEBUG("Alloc ColorImgBuf Fail!\n");
        return NULL;
    }

    int m_BufferSize = 0;
    if (gDepthImgBuf == NULL) {
        if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
            m_BufferSize = 2 * gDepthWidth  * 2 * gDepthHeight;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        } else {
            m_BufferSize = gDepthWidth * gDepthHeight * 2;
            gDepthImgBuf = (unsigned char*)calloc(m_BufferSize, sizeof(unsigned char));
        }
    }

    if (gDepthImgBuf == NULL) {
        CT_DEBUG("Alloc for gDepthImageBuf Fail!\n");
        return NULL;
    }

    constexpr unsigned kMaxCount = 10000;
    BYTE* rgbBuffer = new BYTE[gColorWidth * gColorHeight * 3];
    BYTE* rgbOutBuffer = new BYTE[gColorWidth * gColorHeight * 3];
    float* xyzOutBuffer = new float[gColorWidth * gDepthHeight * 3];

    long long pointCloudCalculationStartTime = 0;
    long long pointCloudCalculationEndTime = 0;

    int64_t cur_color_tv_sec = 0;
    int64_t cur_color_tv_usec = 0;
    int64_t first_color_tv_sec= 0;
    int64_t first_color_tv_usec = 0;
    int64_t prv_color_tv_sec = -1;
    int64_t prv_color_tv_usec = -1;
    int cur_color_serial_num = -1;
    int pre_color_serial_num = -1;
    int64_t color_diff = 0;
    int s_color_diff = 0;

    int64_t cur_depth_tv_sec = 0;
    int64_t cur_depth_tv_usec = 0;
    int64_t first_depth_tv_sec= 0;
    int64_t first_depth_tv_usec = 0;
    int64_t prv_depth_tv_sec = -1;
    int64_t prv_depth_tv_usec = -1;
    int cur_depth_serial_num = -1;
    int pre_depth_serial_num = -1;
    int64_t depth_diff = 0;
    int s_depth_diff = 0;

    uint64_t frame_rate_count = 0;
    uint64_t frame_depth_rate_count = 0;
    long long start_time = calcByGetTimeOfDay();
    int pointCloudFail = APC_Init_Fail;
    int64_t fist_pcl_time_us = 0;
    int64_t kPointCloudCountInteval = 100;
    for (unsigned mCount = 0, pointCloudCount = 0; mCount < kMaxCount; mCount++) {
        // 000. Get Color Frame
        ret = APC_GetColorImageWithTimestamp(EYSD, GetDevSelectIndexPtr(), gColorImgBuf, &gColorImgSize, &cur_color_serial_num, 0,
                                       &cur_color_tv_sec, &cur_color_tv_usec);
        if (ret != APC_OK) {
           CT_DEBUG("Failed to call APC_GetColorImageWithTimestamp()!\n");
           if (ret == APC_DEVICE_TIMEOUT) {
                    /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                    */
                    CT_DEBUG("Getting image is timeout!\n");
                    usleep(1 * 1000);
            }
            continue;
        }
        if (frame_rate_count == 0) {
            first_color_tv_sec  = cur_color_tv_sec;
            first_color_tv_usec = cur_color_tv_usec;
        } else {
            color_diff = ((cur_color_tv_sec - prv_color_tv_sec) * 1000000 + cur_color_tv_usec)-prv_color_tv_usec;
            s_color_diff = cur_color_serial_num - pre_color_serial_num;

            if (gActualFps == 60) {
                if (color_diff > (16666)) {
//                    CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", pre_str,
//                    (int)cur_color_serial_num, s_color_diff, (cur_color_tv_sec * 1000000 + cur_color_tv_usec), diff);
                }
            } else  if (gActualFps == 30) {
//                if (color_diff > (33333)) {
//                    CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", "ColorImage",
//                    (int)cur_color_serial_num, s_color_diff, (cur_color_tv_sec * 1000000 + cur_color_tv_usec), color_diff);
//                }
            }

            if (s_color_diff > 1) {
                CT_DEBUG("[%s][%03lu] SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", "ColorImage",
                         frame_rate_count,
                         (int)cur_color_serial_num, s_color_diff,
                         (cur_color_tv_sec * 1000000 + cur_color_tv_usec), color_diff);
            }

        }

        prv_color_tv_sec = cur_color_tv_sec;
        prv_color_tv_usec = cur_color_tv_usec;
        pre_color_serial_num = cur_color_serial_num;
        // 001. Decode Color Frame YUY2 to RGB
        APC_ColorFormat_to_BGR24(EYSD, GetDevSelectIndexPtr(), rgbBuffer, gColorImgBuf, gColorImgSize, gColorWidth, gColorHeight,
                                 APCImageType::Value::COLOR_YUY2);
        // 002. Get Depth Frame
       ret = APC_GetDepthImageWithTimestamp(EYSD, GetDevSelectIndexPtr(), gDepthImgBuf, &gDepthImgSize, &cur_depth_serial_num, 0,
                                       &cur_depth_tv_sec, &cur_depth_tv_usec);
       
       if (ret != APC_OK) {
           CT_DEBUG("Failed to call APC_GetDepthImageWithTimestamp()!\n");
           if (ret == APC_DEVICE_TIMEOUT) {
                    /*
                        When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                        So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
                    */
                    CT_DEBUG("Getting image is timeout!\n");
                    usleep(1 * 1000);
            }
            continue;
       }
       
        if (frame_rate_count == 0) {
            first_depth_tv_sec  = cur_depth_tv_sec;
            first_depth_tv_usec = cur_depth_tv_usec;
        } else {
            depth_diff = ((cur_depth_tv_sec - prv_depth_tv_sec) * 1000000 + cur_depth_tv_usec)-prv_depth_tv_usec;
            s_depth_diff = cur_depth_serial_num - pre_depth_serial_num;

            if (gActualFps == 60) {
                if (depth_diff > (16666)) {
//                    CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", "depthImage",
//                      (int)cur_depth_serial_num, s_depth_diff, (cur_depth_tv_sec * 1000000 + cur_depth_tv_usec), depth_diff);
                }

            } else  if (gActualFps == 30) {
                if (depth_diff > (33333)) {
//                    CT_DEBUG("[%s]SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", "depthImage",
//                    (int)cur_depth_serial_num, s_depth_diff, (cur_depth_tv_sec * 1000000 + cur_depth_tv_usec), depth_diff);
                }
            }

            if (s_depth_diff > 1) {
                CT_DEBUG("[%s][%03lu] SN: [%03d], SN_DIFF: [%03d], TS: [%lu], TS_DIFF: [%lu]\n", "depthImage",
                         frame_depth_rate_count, (int) cur_depth_serial_num, s_depth_diff,
                         (cur_depth_tv_sec * 1000000 + cur_depth_tv_usec), depth_diff);
            }

        }

        prv_depth_tv_sec = cur_depth_tv_sec;
        prv_depth_tv_usec = cur_depth_tv_usec;
        pre_depth_serial_num = cur_depth_serial_num;
        pointCloudCalculationStartTime = calcByGetTimeOfDay();
        // Point Cloud Count 100
        if (pointCloudCount == 0) {
            fist_pcl_time_us = pointCloudCalculationStartTime;
        }

        // Point Cloud start from first API is about to call
        if (start_time == 0) {
            start_time = calcByGetTimeOfDay();
        }

        // 003. Get Point Cloud Buffers
        pointCloudFail = APC_GetPointCloud(EYSD, GetDevSelectIndexPtr(), rgbBuffer, gColorWidth, gColorHeight, gDepthImgBuf, gDepthWidth, gDepthHeight,
                          &pointCloudInfo, rgbOutBuffer, xyzOutBuffer, (float) g_maxNear, (float) g_maxFar);
        if (pointCloudFail) {
            CT_DEBUG("Point Cloud Fail! at Count:[%d] Status[%d]\n", mCount, pointCloudFail);
        } else {
            // Successfully
            pointCloudCount++;
        }

        pointCloudCalculationEndTime = calcByGetTimeOfDay();
        CT_DEBUG("[%d] APC_GetPointCloud cSN: %d dSN: %d costs %lld uSec\n", pointCloudCount, cur_color_serial_num, cur_depth_serial_num,
                 pointCloudCalculationEndTime - pointCloudCalculationStartTime);

        if (pointCloudCount % kPointCloudCountInteval == 0) {
            float pcl_total_time = pointCloudCalculationEndTime - fist_pcl_time_us;
            CT_DEBUG("[%s] %lu usec per %u frames (%6f)\n", "Point Cloud", (unsigned long) pcl_total_time, kPointCloudCountInteval,
                     (1000000 * kPointCloudCountInteval) / pcl_total_time);
            pointCloudCount = 0;
            fist_pcl_time_us = 0;
        }

        if (frame_rate_count == (kMaxCount -1)) {
            float fltotal_time = 0.0;
            fltotal_time = ((cur_color_tv_sec - first_color_tv_sec) * 1000000 + cur_color_tv_usec)-first_color_tv_usec;
            CT_DEBUG("[%s] %lu usec per %ufs (%6f)\n", "ColorImage",
                     (unsigned long)fltotal_time, kMaxCount, (1000000 * kMaxCount)/fltotal_time);
            frame_rate_count = 0;
//            mCount++;
        } else {
            frame_rate_count++;
        }

        if (frame_depth_rate_count == (kMaxCount -1)) {
            float fltotal_time = 0.0;
            fltotal_time = ((cur_depth_tv_sec - first_depth_tv_sec) * 1000000 + cur_depth_tv_usec)-first_depth_tv_usec;
            CT_DEBUG("[%s] %lu usec per %ufs (%6f)\n", "DepthImage",
                     (unsigned long)fltotal_time, kMaxCount, (1000000 * kMaxCount)/fltotal_time);
            frame_depth_rate_count = 0;
            //mCount++;
        } else {
            frame_depth_rate_count++;
        }
    }

    long long end_time = calcByGetTimeOfDay();

    if (end_time - start_time > 0) {
        float averageFPS = kMaxCount / ((end_time - start_time) / 1000000);
        CT_DEBUG("### Average Point Cloud FPS=[%f] cost: %lld total frames: %d\n", averageFPS, end_time - start_time, kMaxCount);
    }

    if (gColorImgBuf != NULL) {
        CT_DEBUG("Free gColorImgBuf: %p\n", gColorImgBuf);
        free(gColorImgBuf);
        gColorImgBuf = NULL;
    }

    if (gDepthImgBuf != NULL) {
        CT_DEBUG("Free gDepthImgBuf: %p\n", gDepthImgBuf);
        free(gDepthImgBuf);
        gDepthImgBuf = NULL;
    }

    delete [] rgbBuffer;
    delete [] rgbOutBuffer;
    delete [] xyzOutBuffer;

}

static void PointCloudFPS()
{
    pthread_t pcl_fps_tid;
    pthread_attr_t point_cloud_thread_attr;
    struct sched_param point_cloud_thread_param;

    pthread_attr_init(&point_cloud_thread_attr);
    pthread_attr_getschedparam (&point_cloud_thread_attr, &point_cloud_thread_param);
    point_cloud_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&point_cloud_thread_attr, &point_cloud_thread_param);
    pthread_create(&pcl_fps_tid, &point_cloud_thread_attr, pfunc_thread_point_cloud_fps, NULL);
    pthread_join(pcl_fps_tid, NULL);
}

static void Write24X()
{
    int index;
    int nbfferLength = APC_CALIB_LOG_FILE_SIZE;
    int pActualLength = 0;

    BYTE *data = new BYTE[nbfferLength];
    memset(data, 0x0, nbfferLength);

    for (index = 0; index <= 9; index++)
    {
        if (APC_OK == APC_GetLogData(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index, ALL_LOG))
        {
            if (APC_OK == APC_SetLogData(EYSD, GetDevSelectIndexPtr(), data, nbfferLength, &pActualLength, index))
            {
                printf("Write24%d Success!\n", index);
            }
            else
            {
                printf("Write24%d Fail!\n", index);
            }
        }
    }
    delete[] data;
}

int save_file(unsigned char *buf, int size, int width, int height, int type, bool isRGBNamed)
{
    int ret = APC_OK;
    int fd = -1;
    char fname[256] = {0};

    static unsigned int yuv_index = 0;
    static unsigned int mjpeg_index = 0;
    static unsigned int depth_index = 0;
    static unsigned int yuv_rgb_index = 0;
    static unsigned int mjpeg_rgb_index = 0;
    static unsigned int depth_rgb_index = 0;
    char DateTime[32] = {0};
    memset(DateTime, 0, sizeof(DateTime));

    if (gIgnoreDataTime)
        sprintf(DateTime, "Ignore");
    else
        ret = GetDateTime(DateTime);

    if (isRGBNamed) {
        switch (type) {
            case 0: // Color stream (YUYV)
                snprintf(fname, sizeof(fname), SAVE_FILE_PATH"color_img_%02d_%03d_%s.yuv", gsDevSelInfo.index, yuv_index++, DateTime);
                break;
            case 1: // Color stream (MJPEG)
                snprintf(fname, sizeof(fname), SAVE_FILE_PATH"color_img_%02d_%03d_%s.jpg", gsDevSelInfo.index, mjpeg_index++, DateTime);
                break;
            case 2: // Depth stream
                snprintf(fname, sizeof(fname), SAVE_FILE_PATH"depth_img_%02d_%03d_%s.yuv", gsDevSelInfo.index, depth_index++, DateTime);
                break;
            default:
                break;
        }
    } else {
        switch (type) {
            case 0: // YUV
                snprintf(fname, sizeof(fname), SAVE_FILE_PATH"color_yuv2rgb_%02d_%03d_%s.raw", gsDevSelInfo.index, yuv_rgb_index++, DateTime);
                break;
            case 1: // MJPEG
                snprintf(fname, sizeof(fname), SAVE_FILE_PATH"color_mjpeg2rgb_%02d_%03d_%s.raw", gsDevSelInfo.index, mjpeg_rgb_index++, DateTime);
                break;
            case 2: // Depth stream
                snprintf(fname, sizeof(fname), SAVE_FILE_PATH"depth_imgrgb_%02d_%03d_%s.raw", gsDevSelInfo.index, depth_rgb_index++, DateTime);
                break;
            default:
                break;
        }
    }

    if (DEBUG_LOG) CT_DEBUG("File Name: %s\n", fname);
    fd = open(fname, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

    if (fd < 0) {
        CT_DEBUG("file open error (fd: %d)\n", fd);
        ret = APC_Init_Fail;
    } else if (write(fd, buf, size) != size) {
        CT_DEBUG("write(fd, buf, size) != size\n");
        ret = APC_Init_Fail;
    }

    if (fd >= 0) {
        close(fd);
        sync();
    }

    return ret;
}

//s:[eys3D] 20200610 implement to save raw data to RGB format
// YUYV to RGB +
int convert_yuv_to_rgb_pixel(int y, int u, int v)
{
    unsigned int pixel32 = 0;
    unsigned char *pixel = (unsigned char *)&pixel32;
    int r, g, b;

    r = y + (1.370705 * (v-128));
    g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
    b = y + (1.732446 * (u-128));

    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;

    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;

    pixel[0] = r * 220 / 256;
    pixel[1] = g * 220 / 256;
    pixel[2] = b * 220 / 256;

    return pixel32;
}

int convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height, bool isMIPISplit)
{
    unsigned int in, out = 0;
    unsigned int pixel_16;
    unsigned char pixel_24[3];
    unsigned int pixel32;
    int y0, u, y1, v;
    int src_size = width * height * 2;

    // For MIPI No Split
    if (!isMIPISplit)
        src_size = width * height * 2 * 2;

    for(in = 0; in < src_size; in += 4) {
        // For MIPI No Split
        if (!isMIPISplit && (in % (width * 2) == 0) && (in % (width * 4) != 0)) {
            in = in + (width * 2);
        }

        pixel_16 =
        yuv[in + 3] << 24 |
        yuv[in + 2] << 16 |
        yuv[in + 1] <<  8 |
        yuv[in + 0];

        y0 = (pixel_16 & 0x000000ff);
        u  = (pixel_16 & 0x0000ff00) >>  8;
        y1 = (pixel_16 & 0x00ff0000) >> 16;
        v  = (pixel_16 & 0xff000000) >> 24;

        pixel32 = convert_yuv_to_rgb_pixel(y0, u, v);

        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];

        pixel32 = convert_yuv_to_rgb_pixel(y1, u, v);

        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];
    }

    return 0;
}

int saveDepth2rgb(unsigned char *m_pDepthImgBuf, unsigned char *m_pRGBBuf, unsigned int m_nImageWidth, unsigned int m_nImageHeight)
{
    char fname[256] = {0};
    static unsigned int yuv_index = 0;
    int ret = APC_OK ;
    int m_tmp_width= 0;
    char DateTime[32] = {0};
    
    memset(DateTime, 0, sizeof(DateTime));

     ret = GetDateTime(DateTime);
    
    if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
        m_tmp_width = m_nImageWidth * 2;
    } else {
        m_tmp_width = m_nImageWidth;
    }

    RGBQUAD *palette = g_depth_output == DEPTH_IMG_GRAY_TRANSFER ? g_GrayPaletteZ14 : g_ColorPaletteZ14;
    if (gDepthDataType == APC_DEPTH_DATA_8_BITS || gDepthDataType == APC_DEPTH_DATA_8_BITS_RAW) {
        UpdateD8bitsDisplayImage_DIB24(palette, &m_pDepthImgBuf[0], &m_pRGBBuf[0], m_tmp_width, m_nImageHeight);
    } else if (gDepthDataType == APC_DEPTH_DATA_11_BITS || gDepthDataType == APC_DEPTH_DATA_11_BITS_RAW) {
        UpdateD11DisplayImage_DIB24(palette, &m_pDepthImgBuf[0], &m_pRGBBuf[0], m_tmp_width, m_nImageHeight);
    } else if (gDepthDataType == APC_DEPTH_DATA_14_BITS || gDepthDataType == APC_DEPTH_DATA_14_BITS_RAW || gDepthDataType == 34) {
        UpdateZ14DisplayImage_DIB24(palette, &m_pDepthImgBuf[0], &m_pRGBBuf[0], m_tmp_width, m_nImageHeight);
    }

    snprintf(fname, sizeof(fname), SAVE_FILE_PATH"depth_yuv2rgb_%d_%s.bmp", yuv_index++, DateTime);
    ret = save_file(m_pRGBBuf, m_tmp_width * m_nImageHeight * 3, m_tmp_width, m_nImageHeight, 2, false);
    return ret;
}

//e:[eys3D] 20200610 implement to save raw data to RGB format
//s:[eys3D] 20200615 implement ZD table
int fillZDIndexByProductInfos(unsigned short  pid, int depthHeight,
        int colorHeight, bool isUSB3) {

    if (pid == APC_PID_IVY) { //IVY
        return 1;
    }

    if ((pid == APC_PID_8036) || (pid == APC_PID_MIPI_8036)) {//8036
        if (!isUSB3 && colorHeight && depthHeight && (colorHeight % depthHeight != 0) ) {
            // For mode 34 35 on PIF
            return 2;
        }
        if (depthHeight == 720) {
            return 0;
        } else if (depthHeight >= 480) {
            return 1;
        }
        return 0;
    }

    if (pid == APC_PID_8037) {//8037
        if (depthHeight >= 720) {
            return 0;
        } else if (depthHeight >= 480) {
            return 1;
        } else {
            return 0;
        }
    } 
    
    if (pid == APC_PID_8052) {//8052
        if (!isUSB3 && colorHeight && depthHeight && (colorHeight % depthHeight != 0) ) {
            return 2;
        }
        if (depthHeight == 720) {
            return 0;
        } else if (depthHeight >= 480) {
            return 1;
        }
        return 0;
    }
        
    if (pid == APC_PID_HYPATIA) {//hypatia
        if (depthHeight ==400 ) {
            return 0;
        }
        else if (depthHeight == 200) {
            return 1;
        } else if (depthHeight == 104) {
            return 2;
        }
        return 0;
    }

    if (pid == APC_PID_SANDRA) {
        if (isUSB3) {
            if (colorHeight == 720)
                return 0;
            else 
                return 1;
        } else {
            if ((colorHeight == 720) && (depthHeight == 360)) {
                return 0;
            } else {
                return 1;
            }
        }
    }
    
    CT_DEBUG("NOT define the PID, since return 0\n");
    return 0;
}

int getZDtable(DEVINFORMATION *pDevInfo, DEVSELINFO DevSelInfo, int depthHeight, int colorHeight)
{
    ZDTABLEINFO zdTableInfo;
    int bufSize = 0;
    int nRet = -1;
    unsigned short nZValue;

    zdTableInfo.nDataType = APC_DEPTH_DATA_11_BITS;
    memset(g_pzdTable, 0, sizeof(g_pzdTable));
   
    if (pDevInfo[DevSelInfo.index].nDevType == PUMA) { // 8052, 8053 is used to smae ZD table
        bufSize = APC_ZD_TABLE_FILE_SIZE_11_BITS;
    } else {
        bufSize = APC_ZD_TABLE_FILE_SIZE_8_BITS;
    }

    zdTableInfo.nIndex = fillZDIndexByProductInfos(pDevInfo[DevSelInfo.index].wPID, depthHeight, colorHeight, false);
    CT_DEBUG("zdTableInfo nIndex: %d\n", zdTableInfo.nIndex);
    
    int actualLength = 0;
    if (zdTableInfo.nIndex < 0)
        zdTableInfo.nIndex = 0;

    nRet = APC_GetZDTable(EYSD, &DevSelInfo, g_pzdTable, bufSize, &actualLength, &zdTableInfo);
    if (nRet != APC_OK) {
        CT_DEBUG("Get ZD Table Fail! (%d)\n", nRet);
        return nRet;
    }
    
    g_maxNear = 0xfff;
    g_maxFar = 0;

    if (DEBUG_LOG) CT_DEBUG("[%s][%d]Enter to calac mxaFar and maxNear...\n", __func__, __LINE__);
    for (int i = 0 ; i < APC_ZD_TABLE_FILE_SIZE_11_BITS ; ++i) {
        if ((i * 2) == APC_ZD_TABLE_FILE_SIZE_11_BITS)
            break;
        nZValue = (((unsigned short)g_pzdTable[i * 2]) << 8) + g_pzdTable[i * 2 + 1];
        if (nZValue) {
            g_maxNear = std::min<unsigned short>(g_maxNear, nZValue);
            g_maxFar = std::max<unsigned short>(g_maxFar, nZValue);
        }
    }
    if (DEBUG_LOG) CT_DEBUG("[%s][%d]Leave to calac mxaFar and maxNear...\n", __func__, __LINE__);
    
    if (g_maxNear > g_maxFar)
        g_maxNear = g_maxFar;
    
    if (g_maxFar > 1000)
        g_maxFar = 1000;
    
    g_zdTableInfo_index = zdTableInfo.nIndex;

    CT_DEBUG("Get ZD Table actualLength: %d, g_maxNear: %d, g_maxFar: %d\n", actualLength, g_maxNear, g_maxFar);
    return nRet;
}

void UpdateD8bitsDisplayImage_DIB24(RGBQUAD *pColorPalette, unsigned char *pDepth, unsigned char *pRGB, int width, int height)
{
    int nBPS = width * 3;;
    BYTE* pDL    = pRGB;
    BYTE* pD     = NULL;
    const RGBQUAD* pClr = NULL;
    unsigned short z = 0;
    unsigned short zdIndex;

    if ((width <= 0) || (height <= 0)) return;

    for (int y = 0; y < height; y++) {
        pD = pDL;
        for (int x = 0; x < width; x++) {
            int pixelIndex = y * width + x;
            unsigned short depth = (unsigned short)pDepth[pixelIndex];
            
            if (g_pDevInfo->nDevType == PUMA)
                zdIndex = (depth << 3) * sizeof(unsigned short);
            else
                zdIndex = depth * sizeof(unsigned short);
            z = (((unsigned short)g_pzdTable[zdIndex]) << 8) + g_pzdTable[zdIndex + 1];
            if ( z >= COLOR_PALETTE_MAX_COUNT) continue;
           //CT_DEBUG("depth: %d, z value: %d\n",depth,z);
            pClr = &(pColorPalette[z]);
            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;

            pD += 3;
        }
        pDL += nBPS;
    }
}

void UpdateD11DisplayImage_DIB24(const RGBQUAD* pColorPalette, const unsigned char *pDepth, unsigned char *pRGB, int width, int height)
{
    if (width <=0 || height <= 0 ) return;

    int nBPS = ((width * 3 + 3 ) / 4 ) * 4;
    //BYTE* pDL    = pRGB + (height - 1 ) * nBPS;
    BYTE* pDL    = pRGB;
    BYTE* pD     = NULL;
    const RGBQUAD* pClr = NULL;
    unsigned short z = 0;

    for (int y = 0; y < height; y++) {
        pD = pDL;
        for (int x = 0; x < width; x++) {
            int pixelIndex = y * width + x;
            unsigned short depth = pDepth[pixelIndex * sizeof(unsigned short) + 1] << 8 |  pDepth[pixelIndex * sizeof(unsigned short)];
            unsigned short zdIndex = depth * sizeof(unsigned short);
            z = (((unsigned short)g_pzdTable[zdIndex]) << 8) + g_pzdTable[zdIndex + 1];
            //CT_DEBUG("depth: %d, z value: %d\n",depth,z);
            if ( z >= COLOR_PALETTE_MAX_COUNT) continue;
            pClr = &(pColorPalette[z]);
            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;
            pD += 3;
        }
        pDL += nBPS;
    }
}

void UpdateZ14DisplayImage_DIB24(RGBQUAD *pColorPaletteZ14, unsigned char *pDepthZ14, unsigned char *pDepthDIB24, int cx, int cy)
{
    int x,y,nBPS;
    unsigned short *pWSL,*pWS;
    unsigned char *pDL,*pD;
    RGBQUAD *pClr;
    //
    if ((cx <= 0) || (cy <= 0)) return;
    //
    nBPS = cx * 3;
    pWSL = (unsigned short *)pDepthZ14;
    pDL = pDepthDIB24;
    for (y=0; y<cy; y++) {
        pWS = pWSL;
        pD = pDL;
        for (x=0; x<cx; x++) {
            if ( pWS[x] >= COLOR_PALETTE_MAX_COUNT) continue;
            pClr = &(pColorPaletteZ14[pWS[x]]);
            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;
            pD += 3;
        }
        pWSL += cx;
        pDL += nBPS;
    }
}
//e:[eys3D] 20200615 implement ZD table

char* PidToModuleName(unsigned short pid)
{
    if (pid == APC_PID_8036) {
        return "EX8036";
    } else if (pid == APC_PID_MIPI_8036) {
        return "EX8036-MIPI";
    } else if (pid == APC_PID_8037) {
        return "EX8037";
    } else if (pid == APC_PID_8052) {
        return "EX8052";
    } else if (pid == APC_PID_HYPATIA) {
        return "Hypatia";
    } else if (pid == APC_PID_8038_M0) {
        return "EX8038-1";
    } else if (pid == APC_PID_8038_M1) {
        return "EX8038-2";
    } else {
        return nullptr;
    }
}

int get_product_name(char *path, char *out)
{
    FILE* f;
    char buffer[128];
    int i = 0;

    f = fopen(path, "r");
    if (!f) {
        CT_DEBUG("Could not open %s\n", path);
        return -1;
    }

    fgets(buffer, sizeof(buffer), f);
    do {
        out[i] = buffer[i];
        i++;
    } while(buffer[i] != '\0');
    i--;
    out[i] = ':';
    i++;
    out[i] = '\0';
    fclose(f);

    return 0;
}

static void setupDepth(void)
{
    printf("APC_DEPTH_DATA_DEFAULT: %d\n", APC_DEPTH_DATA_DEFAULT);
    printf("APC_DEPTH_DATA_8_BITS: %d\n", APC_DEPTH_DATA_8_BITS);
    printf("APC_DEPTH_DATA_14_BITS: %d\n", APC_DEPTH_DATA_14_BITS);
    printf("APC_DEPTH_DATA_8_BITS_x80: %d\n", APC_DEPTH_DATA_8_BITS_x80);
    printf("APC_DEPTH_DATA_11_BITS: %d\n", APC_DEPTH_DATA_11_BITS);
    printf("APC_DEPTH_DATA_OFF_RECTIFY: %d\n", APC_DEPTH_DATA_OFF_RECTIFY);
    printf("APC_DEPTH_DATA_8_BITS_RAW: %d\n", APC_DEPTH_DATA_8_BITS_RAW);
    printf("APC_DEPTH_DATA_14_BITS_RAW: %d\n", APC_DEPTH_DATA_14_BITS_RAW);
    printf("APC_DEPTH_DATA_8_BITS_x80_RAW: %d\n", APC_DEPTH_DATA_8_BITS_x80_RAW);
    printf("APC_DEPTH_DATA_11_BITS_RAW: %d\n", APC_DEPTH_DATA_11_BITS_RAW);
    printf("APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY: %d\n", APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY);
    printf("Please input depth type value:\n");
    scanf("%d", &gDepthDataType);
}

static bool IsInterleaveMode(void)
{
    return false;
}
static int NormalizeDepthDataType(int *nDepthDataType)
{
    if (*nDepthDataType >= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)
    {
        *nDepthDataType -= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
    }
    else if (*nDepthDataType >= APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)
    {
        *nDepthDataType -= APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
    }
    else if (*nDepthDataType >= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET)
    {
        *nDepthDataType -= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET;
    }

    return APC_OK;
}

static int TransformDepthDataType(int *nDepthDataType, bool bRectifyData)
{
    NormalizeDepthDataType(nDepthDataType);

    switch (*nDepthDataType) {
        case APC_DEPTH_DATA_8_BITS:
        case APC_DEPTH_DATA_8_BITS_RAW:
            *nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS : APC_DEPTH_DATA_8_BITS_RAW; break;
        case APC_DEPTH_DATA_8_BITS_x80:
        case APC_DEPTH_DATA_8_BITS_x80_RAW:
            *nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS_x80 : APC_DEPTH_DATA_8_BITS_x80_RAW; break;
        case APC_DEPTH_DATA_11_BITS:
        case APC_DEPTH_DATA_11_BITS_RAW:
        case APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
            *nDepthDataType = bRectifyData ? APC_DEPTH_DATA_11_BITS : APC_DEPTH_DATA_11_BITS_RAW;  break;
        case APC_DEPTH_DATA_14_BITS:
        case APC_DEPTH_DATA_14_BITS_RAW:
        case APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY:
            *nDepthDataType = bRectifyData ? APC_DEPTH_DATA_14_BITS : APC_DEPTH_DATA_14_BITS_RAW;  break;
        case APC_DEPTH_DATA_OFF_RAW:
        case APC_DEPTH_DATA_OFF_RECTIFY:
            *nDepthDataType = bRectifyData ? APC_DEPTH_DATA_OFF_RECTIFY : APC_DEPTH_DATA_OFF_RAW;  break;
        default:
            *nDepthDataType = bRectifyData ? APC_DEPTH_DATA_DEFAULT : APC_DEPTH_DATA_OFF_RECTIFY;  break;
    }

    if ((gCameraPID == APC_PID_8036) && (gDepthWidth == 640 && gDepthHeight == 360)) {
        char FWVersion[128];
        int nActualLength = 0;
        char *FindFwVer;
        // Apply the same FW to support 360x640 even if USB & MIPI in AMP project
        if (APC_OK == APC_GetFwVersion(EYSD, GetDevSelectIndexPtr(), FWVersion, 256, &nActualLength)) {
            FindFwVer = strstr(FWVersion, "MIPITX-SLAVE");
            CT_DEBUG("FW Version: %s, FindFwVer: %p\n", FWVersion, FindFwVer);
            if (!FindFwVer)
                *nDepthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        }
    }

    if ((gCameraPID == APC_PID_8052) && (gDepthWidth == 640 && gDepthHeight == 360)) {
        *nDepthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
    }

    if ((gCameraPID == APC_PID_HYPATIA2) && (gDepthWidth == 640 && gDepthHeight == 460)) {
        *nDepthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
    }

    //Donkey
    if ((gCameraPID == APC_PID_8036) && (gDepthWidth == 320 && gDepthHeight == 180)) {
        *nDepthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        if (!IsInterleaveMode()) {
            *nDepthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        }
    }

    //Donkey
    if ((gCameraPID == APC_PID_80362) && (gDepthWidth == 320 && gDepthHeight == 180)) {
        *nDepthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        if (!IsInterleaveMode()) {
            *nDepthDataType += APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
        }
    }

    if (IsInterleaveMode()) *nDepthDataType += APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET;

    CT_DEBUG("gCameraPID: 0x%04x\n", gCameraPID);
    return APC_OK;
}

static void *pfunc_thread_close(void *arg)
{
    int ret;

    UNUSED(arg);
    while (1) {
        sleep(1);
        if (EYSD) {
            ret = APC_CloseDevice(EYSD, GetDevSelectIndexPtr());
            if (ret == APC_OK) {
                if (DEBUG_LOG) CT_DEBUG("APC_CloseDevice() Success!\n");
            } else {
                CT_DEBUG("APC_CloseDevice() Fail! (%d)\n", ret);
                error_msg(ret);
            }
            break;
        }
    }
    return NULL;
}

static long long calcByGetTimeOfDay() 
{
    struct timeval startTime;
    long long elapsed;

    gettimeofday(&startTime, NULL);

    elapsed = (long long)(startTime.tv_sec) * 1000000 + (long long)(startTime.tv_usec);

    return elapsed;
}

void print_APC_error(int error)
{
    const char *errorstr = NULL;

    switch (error) {
    case APC_OK:
        errorstr = "APC_OK";
        break;
    case APC_NoDevice:
        errorstr = "APC_NoDevice";
        break;
    case APC_FIND_DEVICE_FAIL:
        errorstr = "APC_FIND_DEVICE_FAIL";
        break;
    case APC_NullPtr:
        errorstr = "APC_NullPtr";
        break;
    case APC_ErrBufLen:
        errorstr = "APC_ErrBufLen";
        break;
    case APC_RET_BAD_PARAM:
        errorstr = "APC_RET_BAD_PARAM";
        break;
    case APC_Init_Fail:
        errorstr = "APC_Init_Fail";
        break;
    case APC_NoZDTable:
        errorstr = "APC_NoZDTable";
        break;
    case APC_READFLASHFAIL:
        errorstr = "APC_READFLASHFAIL";
        break;
    case APC_WRITEFLASHFAIL:
        errorstr = "APC_WRITEFLASHFAIL";
        break;
    case APC_VERIFY_DATA_FAIL:
        errorstr = "APC_VERIFY_DATA_FAIL";
        break;
    case APC_KEEP_DATA_FAIL:
        errorstr = "APC_KEEP_DATA_FAIL";
        break;
    case APC_RECT_DATA_LEN_FAIL:
        errorstr = "APC_RECT_DATA_LEN_FAIL";
        break;
    case APC_RECT_DATA_PARSING_FAIL:
        errorstr = "APC_RECT_DATA_PARSING_FAIL";
        break;
    case APC_NO_CALIBRATION_LOG:
        errorstr = "APC_NO_CALIBRATION_LOG";
        break;
    case APC_POSTPROCESS_INIT_FAIL:
        errorstr = "APC_POSTPROCESS_INIT_FAIL";
        break;
    case APC_POSTPROCESS_NOT_INIT:
        errorstr = "APC_POSTPROCESS_NOT_INIT";
        break;
    case APC_POSTPROCESS_FRAME_FAIL:
        errorstr = "APC_POSTPROCESS_FRAME_FAIL";
        break;
    case APC_RET_OPEN_FILE_FAIL:
        errorstr = "APC_RET_OPEN_FILE_FAIL";
        break;
    case APC_OPEN_DEVICE_FAIL:
        errorstr = "APC_OPEN_DEVICE_FAIL";
        break;
    case APC_CLOSE_DEVICE_FAIL:
        errorstr = "APC_CLOSE_DEVICE_FAIL";
        break;
    case APC_GET_RES_LIST_FAIL:
        errorstr = "APC_GET_RES_LIST_FAIL";
        break;
    case APC_READ_REG_FAIL:
        errorstr = "APC_READ_REG_FAIL";
        break;
    case APC_WRITE_REG_FAIL:
        errorstr = "APC_WRITE_REG_FAIL";
        break;
    case APC_SET_FPS_FAIL:
        errorstr = "APC_SET_FPS_FAIL";
        break;
    case APC_VIDEO_RENDER_FAIL:
        errorstr = "APC_VIDEO_RENDER_FAIL";
        break;
    case APC_GET_IMAGE_FAIL:
        errorstr = "APC_GET_IMAGE_FAIL";
        break;
    case APC_CALLBACK_REGISTER_FAIL:
        errorstr = "APC_CALLBACK_REGISTER_FAIL";
        break;
    case APC_GET_CALIBRATIONLOG_FAIL:
        errorstr = "APC_GET_CALIBRATIONLOG_FAIL";
        break;
    case APC_SET_CALIBRATIONLOG_FAIL:
        errorstr = "APC_SET_CALIBRATIONLOG_FAIL";
        break;
    case APC_NotSupport:
        errorstr = "APC_NotSupport";
        break;
    case APC_NOT_SUPPORT_RES:
        errorstr = "APC_NOT_SUPPORT_RES";
        break;
    case APC_DEVICE_NOT_SUPPORT:
        errorstr = "APC_DEVICE_NOT_SUPPORT";
        break;
    case APC_DEVICE_BUSY:
        errorstr = "APC_DEVICE_BUSY";
        break;
    default:
        errorstr = "UNKNOWN..";
    }

    CT_DEBUG("%s\n", errorstr);
}

static int error_msg(int error)
{
    int ret = ERROR_NONE;
    const char *errstrEYSD = NULL;
    const char *errstrSensor = NULL;

    switch (error) {
    case APC_OK:
                errstrEYSD = "APC_OK";
        break;
    case APC_NoDevice:
                errstrEYSD = "APC_NoDevice";
        break;
    case APC_FIND_DEVICE_FAIL:
                errstrEYSD = "APC_FIND_DEVICE_FAIL";
        break;
    case APC_NullPtr:
                errstrEYSD = "APC_NullPtr";
        break;
    case APC_ErrBufLen:
                errstrEYSD = "APC_ErrBufLen";
        break;
    case APC_RET_BAD_PARAM:
                errstrEYSD = "APC_RET_BAD_PARAM";
        break;
    case APC_Init_Fail:
                errstrEYSD = "APC_Init_Fail";
        break;
    case APC_NoZDTable:
                errstrEYSD = "APC_NoZDTable";
        break;
    case APC_READFLASHFAIL:
                errstrEYSD = "APC_READFLASHFAIL";
        break;
    case APC_WRITEFLASHFAIL:
                errstrEYSD = "APC_WRITEFLASHFAIL";
        break;
    case APC_VERIFY_DATA_FAIL:
                errstrEYSD = "APC_VERIFY_DATA_FAIL";
        break;
    case APC_KEEP_DATA_FAIL:
                errstrEYSD = "APC_KEEP_DATA_FAIL";
        break;
    case APC_RECT_DATA_LEN_FAIL:
                errstrEYSD = "APC_RECT_DATA_LEN_FAIL";
        break;
    case APC_RECT_DATA_PARSING_FAIL:
                errstrEYSD = "APC_RECT_DATA_PARSING_FAIL";
        break;
    case APC_NO_CALIBRATION_LOG:
                errstrEYSD = "APC_NO_CALIBRATION_LOG";
        break;
    case APC_POSTPROCESS_INIT_FAIL:
                errstrEYSD = "APC_POSTPROCESS_INIT_FAIL";
        break;
    case APC_POSTPROCESS_NOT_INIT:
                errstrEYSD = "APC_POSTPROCESS_NOT_INIT";
        break;
    case APC_POSTPROCESS_FRAME_FAIL:
                errstrEYSD = "APC_POSTPROCESS_FRAME_FAIL";
        break;
    case APC_RET_OPEN_FILE_FAIL:
                errstrEYSD = "APC_RET_OPEN_FILE_FAIL";
        break;
    case APC_OPEN_DEVICE_FAIL:
                errstrEYSD = "APC_OPEN_DEVICE_FAIL";
        break;
    case APC_CLOSE_DEVICE_FAIL:
                errstrEYSD = "APC_CLOSE_DEVICE_FAIL";
        break;
    case APC_GET_RES_LIST_FAIL:
                errstrEYSD = "APC_GET_RES_LIST_FAIL";
        break;
    case APC_READ_REG_FAIL:
                errstrEYSD = "APC_READ_REG_FAIL";
        break;
    case APC_WRITE_REG_FAIL:
                errstrEYSD = "APC_WRITE_REG_FAIL";
        break;
    case APC_SET_FPS_FAIL:
                errstrEYSD = "APC_SET_FPS_FAIL";
        break;
    case APC_VIDEO_RENDER_FAIL:
                errstrEYSD = "APC_VIDEO_RENDER_FAIL";
        break;
    case APC_GET_IMAGE_FAIL:
                errstrEYSD = "APC_GET_IMAGE_FAIL";
        break;
    case APC_CALLBACK_REGISTER_FAIL:
                errstrEYSD = "APC_CALLBACK_REGISTER_FAIL";
        break;
    case APC_GET_CALIBRATIONLOG_FAIL:
                errstrEYSD = "APC_GET_CALIBRATIONLOG_FAIL";
        break;
    case APC_SET_CALIBRATIONLOG_FAIL:
                errstrEYSD = "APC_SET_CALIBRATIONLOG_FAIL";
        break;
    case APC_NotSupport:
                errstrEYSD = "APC_NotSupport";
        break;
    case APC_NOT_SUPPORT_RES:
                errstrEYSD = "APC_NOT_SUPPORT_RES";
        break;
    case APC_DEVICE_NOT_SUPPORT:
                errstrEYSD = "APC_DEVICE_NOT_SUPPORT";
        break;
    case APC_DEVICE_BUSY:
                errstrEYSD = "APC_DEVICE_BUSY";
        break;
    default:
                errstrEYSD = "APC_UNKNOWN..";
    }

    switch (error) {
    case APC_OK:
                ret = ERROR_NONE;
                errstrSensor = "ERROR_NONE";
        break;
    case APC_NoDevice:
    case APC_FIND_DEVICE_FAIL:
                ret = ERROR_NO_SUCH_DEVICE;
                errstrSensor = "ERROR_NO_SUCH_DEVICE";
        break;
    case APC_NullPtr:
    case APC_ErrBufLen:
    case APC_RET_BAD_PARAM:
                ret = ERROR_INVALID_PARAMETER;
                errstrSensor = "ERROR_INVALID_PARAMETER";
        break;
    case APC_Init_Fail:
    case APC_NoZDTable:
    case APC_READFLASHFAIL:
    case APC_WRITEFLASHFAIL:
    case APC_VERIFY_DATA_FAIL:
    case APC_KEEP_DATA_FAIL:
    case APC_RECT_DATA_LEN_FAIL:
    case APC_RECT_DATA_PARSING_FAIL:
    case APC_NO_CALIBRATION_LOG:
    case APC_POSTPROCESS_INIT_FAIL:
    case APC_POSTPROCESS_NOT_INIT:
    case APC_POSTPROCESS_FRAME_FAIL:
    case APC_RET_OPEN_FILE_FAIL:
    case APC_OPEN_DEVICE_FAIL:
    case APC_CLOSE_DEVICE_FAIL:
    case APC_GET_RES_LIST_FAIL:
    case APC_READ_REG_FAIL:
    case APC_WRITE_REG_FAIL:
    case APC_SET_FPS_FAIL:
    case APC_VIDEO_RENDER_FAIL:
    case APC_GET_IMAGE_FAIL:
    case APC_CALLBACK_REGISTER_FAIL:
    case APC_GET_CALIBRATIONLOG_FAIL:
    case APC_SET_CALIBRATIONLOG_FAIL:
                ret = ERROR_IO_ERROR;
                errstrSensor = "ERROR_IO_ERROR";
        break;
    case APC_NotSupport:
    case APC_NOT_SUPPORT_RES:
    case APC_DEVICE_NOT_SUPPORT:
                ret = ERROR_NOT_SUPPORTED;
                errstrSensor = "ERROR_NOT_SUPPORTED";
        break;
    case APC_DEVICE_BUSY:
                ret = ERROR_RESOURCE_BUSY;
                errstrSensor = "ERROR_RESOURCE_BUSY";
        break;
    default:
                ret = ERROR_UNKNOWN;
                errstrSensor = "ERROR_UNKNOWN";
    }

        CT_DEBUG("[ERROR] %s (0x%08x) -> %s (0x%08x)\n", errstrEYSD, error, errstrSensor, ret);

    return ret;
}

//s:[eys3D] 20200623, auto config video mode for Hypatia project
void setHypatiaVideoMode(int mode) {
        int ret = APC_OK;

        if (APC_SetupBlock(EYSD, GetDevSelectIndexPtr(), true) != 0) {
            CT_DEBUG("Setup Blocking Fail!\n");
        }

        snapShot_color = true;
        snapShot_depth = true;
        g_depth_output = DEPTH_IMG_COLORFUL_TRANSFER;
        gDepth_Transfer_ctrl = (DEPTH_TRANSFER_CTRL)DEPTH_IMG_NON_TRANSFER;
        gDepthDataType = APC_DEPTH_DATA_11_BITS;

        switch (mode) {
            case 1:
                gColorFormat = 1;
                gColorWidth = 640;
                gColorHeight = 400;

                gDepthWidth = 640;
                gDepthHeight = 400;
                gActualFps = 30;
                break;
            case 2:
                gColorFormat = 1;
                gColorWidth = 320;
                gColorHeight = 200;

                gDepthWidth = 320;
                gDepthHeight = 200;
                gActualFps = 30;
                break;
            case 3:
                gColorFormat = 1;
                gColorWidth = 320;
                gColorHeight = 104;

                gDepthWidth = 320;
                gDepthHeight = 104;
                gActualFps = 30;
                break;
            case 4:
                gColorFormat = 1;
                gColorWidth = 1280;
                gColorHeight = 400;

                gDepthWidth = 640;
                gDepthHeight = 400;
                gActualFps = 15;
                break;
            case 5:
                gColorFormat = 1;
                gColorWidth = 640;
                gColorHeight = 200;

                gDepthWidth = 320;
                gDepthHeight = 200;
                gActualFps = 15;
                break;
            case 6:
                gColorFormat = 1;
                gColorWidth = 640;
                gColorHeight = 104;

                gDepthWidth = 320;
                gDepthHeight =104;
                gActualFps = 15;
                break;
            case 7:
                gColorFormat = 0;
                gColorWidth = 1280;
                gColorHeight = 400;

                gDepthWidth =0;
                gDepthHeight = 0;
                gActualFps = 15;
                gDepthDataType = APC_DEPTH_DATA_DEFAULT;
                break;
            case 8:
                gColorFormat = 0;
                gColorWidth = 640;
                gColorHeight =200;

                gDepthWidth = 0;
                gDepthHeight = 0;
                gActualFps = 15;
                gDepthDataType = APC_DEPTH_DATA_DEFAULT;
                break;
            case 9:
                gColorFormat = 0;
                gColorWidth = 640;
                gColorHeight = 104;

                gDepthWidth = 0;
                gDepthHeight = 0;
                gActualFps = 15;
                gDepthDataType = APC_DEPTH_DATA_DEFAULT;
                break;
            default:
                break;
        }

        ret = APC_SetDepthDataType(EYSD, GetDevSelectIndexPtr(), gDepthDataType); //4 ==> 11 bits
        if (ret == APC_OK) {
            if (DEBUG_LOG) CT_DEBUG("APC_SetDepthData() Success!\n");
        } else {
            CT_DEBUG("APC_SetDepthData() Fail! (%d)\n", ret);
            print_APC_error(ret);
        }
}
//e:[eys3D] 20200623, auto config video mode for Hypatia project

//s:[eys3D] 20200623, for IR mode
int setupIR(unsigned short IRvalue)
{
    int ret;
    unsigned short m_nIRMax, m_nIRMin, m_nIRValue;
    ret = APC_GetFWRegister(EYSD, GetDevSelectIndexPtr(),
                                0xE2, &m_nIRMax,FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret) return ret;

    ret = APC_GetFWRegister(EYSD, GetDevSelectIndexPtr(),
                                0xE1, &m_nIRMin,FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret) return ret;

    if (IRvalue > m_nIRMax || IRvalue < m_nIRMin) {
        m_nIRValue = (m_nIRMax - m_nIRMin) / 2;
    } else {
        m_nIRValue = IRvalue;
    }
    if (DEBUG_LOG) CT_DEBUG("IR range, IR Min: %d, IR Max: %d, set IR Value: %d\n", m_nIRMin, m_nIRMax, m_nIRValue);

    if (m_nIRValue != 0) {
        ret = APC_SetIRMode(EYSD, GetDevSelectIndexPtr(), 0x63); // 6 bits on for opening both 6 ir
        if (APC_OK != ret) return ret;
        CT_DEBUG("Enable IR and set IR Value: %d\n", m_nIRValue);
        ret = APC_SetCurrentIRValue(EYSD, GetDevSelectIndexPtr(), m_nIRValue);
        if (APC_OK != ret) return ret;
        ret = APC_GetCurrentIRValue(EYSD, GetDevSelectIndexPtr(), &m_nIRValue);
        if (APC_OK != ret) return ret;
        CT_DEBUG("Get IR Value: %d\n", m_nIRValue);
    } else {
        ret = APC_SetCurrentIRValue(EYSD, GetDevSelectIndexPtr(), m_nIRValue);
        if (APC_OK != ret) return ret;
        ret = APC_SetIRMode(EYSD,GetDevSelectIndexPtr(), 0x00); // turn off ir
        if (APC_OK != ret) return ret;
        CT_DEBUG("Disable IR\n");
    }
    return APC_OK;
}
//e:[eys3D] 20200623, for IR mode
//e:[eys3D] 20200623, for IR mode

static void *get_exposure_time_test_func(void *arg)
{   
    int ret = APC_OK;
    float fExposureTimeMs = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_GetAEStatus(EYSD, GetDevSelectIndexPtr(), &AEStatus);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetAEStatus()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_GetAEStatus() (%d, %s)\n", ret, AEStatus == AE_ENABLE ? "ENABLE" : "DISABLE");
    }

    ret = APC_GetExposureTime(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fExposureTimeMs);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetExposureTime()(%d)\n", ret);
    } else {
        CT_DEBUG("ExposureTime: (%f)\n", fExposureTimeMs);
    }

    return NULL;
}

static void *set_exposure_time_test_func(void *arg)
{   
    int ret = APC_OK;
    float fExposureTimeMs = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_DisableAE(EYSD, GetDevSelectIndexPtr());
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_DisableAE()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_DisableAE()(%d)\n", ret);
    }

    get_exposure_time_test_func(NULL);
    
    CT_DEBUG("Please Input ExposureTime(ms): ");
    scanf("%f", &fExposureTimeMs);
    CT_DEBUG("Your input ExposureTime is %f\n", fExposureTimeMs);

    ret = APC_SetExposureTime(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, fExposureTimeMs);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_SetExposureTime()(%d)\n", ret);
    } else {
        ret = APC_GetExposureTime(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fExposureTimeMs);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetExposureTime()(%d)\n", ret);
        } else {
            CT_DEBUG("ExposureTime: (%f)\n", fExposureTimeMs);
        }
    }

    return NULL;
}

static void *get_global_gain_test_func(void *arg)
{   
    int ret = APC_OK;
    float fGlobalGain = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_GetAEStatus(EYSD, GetDevSelectIndexPtr(), &AEStatus);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetAEStatus()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_GetAEStatus() (%d, %s)\n", ret, AEStatus == AE_ENABLE ? "ENABLE" : "DISABLE");
    }

    ret = APC_GetGlobalGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fGlobalGain);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetGlobalGain()(%d)\n", ret);
    } else {
        CT_DEBUG("GlobalGain: (%f)\n", fGlobalGain);
    }

    return NULL;
}

static void *set_global_gain_test_func(void *arg)
{   
    int ret = APC_OK;
    float fGlobalGain = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_DisableAE(EYSD, GetDevSelectIndexPtr());
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_DisableAE()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_DisableAE()(%d)\n", ret);
    }

    get_global_gain_test_func(NULL);

    CT_DEBUG("Please Input GlobalGain: ");
    scanf("%f", &fGlobalGain);
    CT_DEBUG("Your input GlobalGain is %f\n", fGlobalGain);
    
    ret = APC_SetGlobalGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, fGlobalGain);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_SetGlobalGain()(%d)\n", ret);
    } else {
        ret = APC_GetGlobalGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fGlobalGain);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetGlobalGain()(%d)\n", ret);
        } else {
            CT_DEBUG("GlobalGain: (%f)\n", fGlobalGain);
        }
    }

    return NULL;
}


static void *get_analog_gain_test_func(void *arg)
{
    int ret = APC_OK;
    float fGain = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_GetAEStatus(EYSD, GetDevSelectIndexPtr(), &AEStatus);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetAEStatus()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_GetAEStatus() (%d, %s)\n", ret, AEStatus == AE_ENABLE ? "ENABLE" : "DISABLE");
    }

    ret = APC_GetAnalogGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fGain);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetAnalogGain()(%d)\n", ret);
    } else {
        CT_DEBUG("GetAnalogGain: (%f)\n", fGain);
    }

    return NULL;
}

static void *set_analog_gain_test_func(void *arg)
{
    int ret = APC_OK;
    float fGain = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_DisableAE(EYSD, GetDevSelectIndexPtr());
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_DisableAE()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_DisableAE()(%d)\n", ret);
    }

    get_analog_gain_test_func(NULL);

    CT_DEBUG("Please Input AnalogGain: ");
    scanf("%f", &fGain);
    CT_DEBUG("Your input AnalogGain is %f\n", fGain);

    ret = APC_SetAnalogGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, fGain);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_SetAnalogGain()(%d)\n", ret);
    } else {
        ret = APC_GetAnalogGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fGain);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetAnalogGain()(%d)\n", ret);
        } else {
            CT_DEBUG("GetAnalogGain: (%f)\n", fGain);
        }
    }

    return NULL;
}


static void *get_digital_gain_test_func(void *arg)
{
    int ret = APC_OK;
    float fGain = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_GetAEStatus(EYSD, GetDevSelectIndexPtr(), &AEStatus);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetAEStatus()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_GetAEStatus() (%d, %s)\n", ret, AEStatus == AE_ENABLE ? "ENABLE" : "DISABLE");
    }

    ret = APC_GetDigitalGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fGain);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetDigitalGain()(%d)\n", ret);
    } else {
        CT_DEBUG("GetDigitalGain: (%f)\n", fGain);
    }

    return NULL;
}

static void *set_digital_gain_test_func(void *arg)
{
    int ret = APC_OK;
    float fGain = 0.0f;
    AE_STATUS AEStatus;

    ret = APC_DisableAE(EYSD, GetDevSelectIndexPtr());
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_DisableAE()(%d)\n", ret);
    } else {
        CT_DEBUG("Call APC_DisableAE()(%d)\n", ret);
    }

    get_digital_gain_test_func(NULL);

    CT_DEBUG("Please Input DigitalGain: ");
    scanf("%f", &fGain);
    CT_DEBUG("Your input DigitalGain is %f\n", fGain);

    ret = APC_SetDigitalGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, fGain);
    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_SetDigitalGain()(%d)\n", ret);
    } else {
        ret = APC_GetDigitalGain(EYSD, GetDevSelectIndexPtr(), SENSOR_BOTH, &fGain);
        if (ret != APC_OK) {
            CT_DEBUG("Failed to call APC_GetDigitalGain()(%d)\n", ret);
        } else {
            CT_DEBUG("GetDigitalGain: (%f)\n", fGain);
        }
    }

    return NULL;
}

static void *get_property_bar_info_func(void *arg)
{
    int ret = APC_OK;
    int i = 0;
    int id = 0;
    int max = 0;
    int min = 0;
    int step = 0;
    int def = 0;
    int flags = 0;
    long int cur_val = 0;
    unsigned short int support_list = 0;

    (void)arg;

    ret = APC_GetCTPUSupportList(EYSD, GetDevSelectIndexPtr(), CT_PROPERTY_ID, &support_list);
    if (ret == APC_OK) {
        if (DEBUG_LOG) CT_DEBUG("CT Bit Map: [0x%04x]\n", support_list);
    } else {
        CT_DEBUG("Failed to call APC_GetCTPUSupportList()(%d, %d)\n", CT_PROPERTY_ID, ret);
        goto EXIT;
    }
#ifdef CT_DEBUG_ENABLE
    for (i = 0; i < CT_CTRL_SEL_NUM; i++) {
        if (DEBUG_LOG) printf("%45s: [%s]\n", gCtCtrlSelectors[i], ((1 << i) & support_list) ? "*" : " ");
    }
#endif

    ret = APC_GetCTPUSupportList(EYSD, GetDevSelectIndexPtr(), PU_PROPERTY_ID, &support_list);
    if (ret == APC_OK) {
        if (DEBUG_LOG) CT_DEBUG("PU Bit Map: [0x%04x]\n", support_list);
    } else {
        CT_DEBUG("Failed to call APC_GetCTPUSupportList()(%d, %d)\n", PU_PROPERTY_ID, ret);
        goto EXIT;
    }
#ifdef CT_DEBUG_ENABLE
    for (i = 0; i < PU_CTRL_SEL_NUM; i++) {
        if (DEBUG_LOG) printf("%45s: [%s]\n", gPuCtrlSelectors[i], ((1 << i) & support_list) ? "*" : " ");
    }
#endif

    id = CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL;
    ret = APC_GetCTRangeAndStep(EYSD, GetDevSelectIndexPtr(), id, &max, &min, &step, &def, &flags);
    if (ret == APC_OK) {
        CT_DEBUG("ETA[max, min, step, def, flags]: [%d, %d, %d, %d, 0x%08x]\n", max, min, step, def, flags);
    } else {
        CT_DEBUG("Failed to call APC_GetCTRangeAndStep()(%d, %d)\n", id, ret);
        goto EXIT;
    }

    id = CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL;
    ret = APC_GetCTPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
    if (ret == APC_OK) {
        CT_DEBUG("AE: (%d)\n", cur_val);
    } else {
        CT_DEBUG("Failed to call APC_GetCTPropVal()(%d, %d)\n", id, ret);
        goto EXIT;
    }

    id = CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL;
    ret = APC_GetCTPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
    if (ret == APC_OK) {
         CT_DEBUG("ETA: (%d)\n", cur_val);
    } else {
        CT_DEBUG("Failed to call APC_GetCTPropVal()(%d, %d)\n", id, ret);
        goto EXIT;
    }

    id = PU_PROPERTY_ID_WHITE_BALANCE_CTRL;
    ret = APC_GetPURangeAndStep(EYSD, GetDevSelectIndexPtr(), id, &max, &min, &step, &def, &flags);
    if (ret == APC_OK) {
        CT_DEBUG("WB[max, min, step, def, flags]: [%d, %d, %d, %d, 0x%08x]\n", max, min, step, def, flags);
        
    } else {
        CT_DEBUG("Failed to call APC_GetPURangeAndStep()(%d, %d)\n", id, ret);
        goto EXIT;
    }

    id = PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL;
    ret = APC_GetPUPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
    if (ret == APC_OK) {
        CT_DEBUG("AWB: (%d)\n", cur_val);
    } else {
        CT_DEBUG("Failed to call APC_GetPUPropVal()(%d, %d)\n", id, ret);
        goto EXIT;
    }

    id = PU_PROPERTY_ID_WHITE_BALANCE_CTRL;
    ret = APC_GetPUPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
    if (ret == APC_OK) {
        CT_DEBUG("WB: [0x%08x]\n", cur_val);
    } else {
        CT_DEBUG("Failed to call APC_GetPUPropVal()(%d, %d)\n", id, ret);
        goto EXIT;
    }
#if 0 // Marlboro modify because GAIN_CONTROL do NOT be supported on 8036
    id = PU_PROPERTY_ID_GAIN_CTRL;
    ret = APC_GetPURangeAndStep(EYSD, GetDevSelectIndexPtr(), id, &max, &min, &step, &def, &flags);
    if (ret == APC_OK) {
        CT_DEBUG("GAN[max, min, step, def, flags] = [%d, %d, %d, %d, 0x%08x]\n", max, min, step, def, flags);
        
    } else {
        CT_DEBUG("Failed to call APC_GetPURangeAndStep()(%d, %d)\n", id, ret);
        goto EXIT;
    }

    id = PU_PROPERTY_ID_GAIN_CTRL;
    ret = APC_GetPUPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
    if (ret == APC_OK) {
        CT_DEBUG("GAN = [0x%08x]\n", cur_val);
    } else {
        CT_DEBUG("Failed to call APC_GetPUPropVal()(%d, %d)\n", id, ret);
        goto EXIT;
    }
#endif
EXIT:

    return NULL;
}

static void *property_bar_test_func(void *arg)
{
    int ret = APC_OK;
    int id = 0;
    int max = 0;
    int min = 0;
    int step = 0;
    int def = 0;
    int flags = 0;
    long int cur_val = 0;
    long int set_val = 0;
    int is_enable_ae = AE_DISABLE, is_enable_awb = AWB_DISABLE;

    (void)arg;

    id = CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL;
    ret = APC_GetCTPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
    if (ret == APC_OK) {
        CT_DEBUG("AE = [0x%08x], %s\n", cur_val, cur_val == AE_MOD_MANUAL_MODE ? "MANUAL" : "APERTURE_PRIORITY");
        /* 
         * Only support Manual Mode & Aperture Priority Mode
         * 0x01: Manual Mode, manual Exposure Time, manual Iris
         * 0x08: Aperture Priority Mode, auto Exposure Time, manual Iris
         */
        // Marlboro add for testing exposure time of property bar
        CT_DEBUG("Please Input AE(0: Enable, 1: Disable): ");
        scanf("%d", &is_enable_ae);
        if (is_enable_ae != AE_ENABLE) {
            is_enable_ae = AE_DISABLE;
        }
        CT_DEBUG("Your input AE is %d, %s\n", is_enable_ae, is_enable_ae == AE_ENABLE ? "ENABLE" : "DISABLE");

        if (is_enable_ae == AE_DISABLE) {
            set_val = AE_MOD_MANUAL_MODE; //Manual Mode, manual Exposure Time, manual Iris
        } else  {
            set_val = AE_MOD_APERTURE_PRIORITY_MODE; //Aperture Priority Mode, auto Exposure Time, manual Iris
        }

        ret = APC_SetCTPropVal(EYSD, GetDevSelectIndexPtr(), id, set_val);
        if (ret == APC_OK) {
            ret = APC_GetCTPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
            if (ret == APC_OK) {
                CT_DEBUG("AE: (%d) after setting (%d)\n", cur_val, set_val);
            } else {
                CT_DEBUG("Failed to call APC_GetCTPropVal()(%d, %d)\n", id, ret);
                goto EXIT;
            }
         } else {
            CT_DEBUG("Failed to call APC_SetCTPropVal()(%d, %d)\n", id, ret);
            goto EXIT;
        }
    } else {
        CT_DEBUG("Failed to call APC_GetCTPropVal()(%d, %d)\n", id, ret);
        goto EXIT;
    }

    if (is_enable_ae == AE_DISABLE) {
        // Marlboro add for testing exposure time of property bar
        CT_DEBUG("Please Input ETA(%d~%d): ", min, max);
        scanf("%d", &set_val);
        CT_DEBUG("Your input ETA is %d\n", set_val);

        id = CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL;
        ret = APC_SetCTPropVal(EYSD, GetDevSelectIndexPtr(), id, set_val);
        if (ret == APC_OK) {
            ret = APC_GetCTPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
            if (ret == APC_OK) {
                CT_DEBUG("ETA: (%d) after setting (%d)\n", cur_val, set_val);
            }
        } else {
            CT_DEBUG("Failed to call APC_SetCTPropVal()(%d, %d)\n", id, ret);
            goto EXIT;
        }
    }

    id = PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL;
    ret = APC_GetPUPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
    if (ret == APC_OK) {
        CT_DEBUG("AWB: [0x%08x], %s\n", cur_val, cur_val == PU_PROPERTY_ID_AWB_ENABLE ? "ENABLE" : "DISABLE");
        // Marlboro add for testing white balance component auto control of property bar
        CT_DEBUG("Please Input AWB(0: Enable, 1: Disable): ");
        scanf("%d", &is_enable_awb);
        if (is_enable_awb != AWB_ENABLE) {
            is_enable_awb = AWB_DISABLE;
        }
        CT_DEBUG("Your input AWB is %d, %s\n", is_enable_awb, is_enable_awb == AWB_ENABLE ? "ENABLE" : "DISABLE");
        if (is_enable_awb == AE_ENABLE) {
            set_val = PU_PROPERTY_ID_AWB_ENABLE;
        } else  {
            set_val = PU_PROPERTY_ID_AWB_DISABLE;
        }

        ret = APC_SetPUPropVal(EYSD, GetDevSelectIndexPtr(), id, set_val);
        if (ret == APC_OK) {
            ret = APC_GetPUPropVal(EYSD, GetDevSelectIndexPtr(), id, &cur_val);
            if (ret == APC_OK) {
                CT_DEBUG("AWB: (%d) after setting (%d)\n", cur_val, set_val);
            } else {
                CT_DEBUG("Failed to call APC_GetPUPropVal()(%d, %d)\n", id, ret);
                goto EXIT;
            }
         } else {
            CT_DEBUG("Failed to call APC_SetPUPropVal()(%d, %d)\n", id, ret);
            goto EXIT;
        }
    } else {
        CT_DEBUG("Failed to call APC_GetPUPropVal()(%d, %d)\n", id, ret);
        goto EXIT;
    }
EXIT:

    return NULL;
}

#define MIPI_POINT_CLOUD_MAX_CNT 1
static void *mipi_point_cloud_test_func(void *arg) 
{
    int ret = APC_OK;
    int64_t cur_tv_sec = 0, cur_tv_usec = 0;
    int cur_serial_num = -1, cur_depth_serial_num = -1;
    unsigned int count = 0;
    bool bFirstReceived = true;
    uint8_t *pPointCloudRGB = NULL;
    float *pPointCloudXYZ = NULL;

    (void)arg;

    if (gSplitImage) {
        CT_DEBUG("Depth Image: [%d x %d @ %d]\n", gDepthWidth, gDepthHeight, gActualFps);
        if(gDepthImgBuf == NULL) {
            gDepthImgBuf = (uint8_t*)calloc(2 * gDepthWidth * gDepthHeight, sizeof(uint8_t));
        }

        CT_DEBUG("Color Image: [%d x %d @ %d]\n", gColorWidth, gColorHeight, gActualFps);
        if (gColorImgBuf == NULL) {
            gColorImgBuf = (uint8_t*)calloc(2 * gColorWidth * gColorHeight , sizeof(uint8_t));
        }

        if (gColorRGBImgBuf == NULL) {
            gColorRGBImgBuf = (uint8_t*)calloc(3 * gColorWidth * gColorHeight, sizeof(uint8_t));
        }

        if (gDepthImgBuf == NULL || gColorImgBuf == NULL || gColorRGBImgBuf == NULL) {
            CT_DEBUG("Failed to alloc gDepthImageBuf or gColorImgBuf or gColorRGBImgBuf\n");
            goto exit;
        }
    } else {
        CT_DEBUG("Color Image: [(%d + %d) x %d @ %d]\n", gColorWidth, gDepthWidth, gColorHeight, gActualFps);
        if (gColorImgBuf == NULL) {
            gColorImgBuf = (uint8_t*)calloc(2 * (gColorWidth + gDepthWidth) * gColorHeight, sizeof(uint8_t));
        }

        if (gColorImgBuf == NULL) {
            CT_DEBUG("Failed to alloc gColorImgBuf\n");
            goto exit;
        }
    }

    pPointCloudRGB = (uint8_t *)malloc(3 * gDepthWidth * gDepthHeight * sizeof(uint8_t));
    pPointCloudXYZ = (float *)malloc(3 * gDepthWidth * gDepthHeight * sizeof(float));
    if(pPointCloudRGB == NULL || pPointCloudXYZ == NULL) {
        CT_DEBUG("Failed to alloc for pPointCloudRGB or pPointCloudXYZ\n");
        goto exit;
    }

    while (count < MIPI_POINT_CLOUD_MAX_CNT) {
        if (gSplitImage) {
            ret = APC_Get2ImageWithTimestamp(EYSD, GetDevSelectIndexPtr(),
                                             (uint8_t*)gColorImgBuf, (uint8_t *)gDepthImgBuf,
                                             &gColorImgSize, &gDepthImgSize,
                                             &cur_serial_num, &cur_depth_serial_num,
                                             gDepthDataType, &cur_tv_sec, &cur_tv_usec);
            save_file(gColorImgBuf, gColorImgSize, gColorWidth, gColorHeight, gColorFormat, true);
            save_file(gDepthImgBuf, gDepthImgSize, gDepthWidth, gDepthHeight, 2, true);
        } else {
            ret = APC_Get2ImageWithTimestampNoSplit(EYSD, GetDevSelectIndexPtr(),
                                                    (uint8_t*)gColorImgBuf, &gColorImgSize, &gColorSerial,
                                                    &cur_tv_sec, &cur_tv_usec, snapShot_mipi);
            save_file(gColorImgBuf, gColorImgSize, gColorWidth + gDepthWidth, gColorHeight, gColorFormat, true);
        }
        if (ret == APC_OK) {
            if (bFirstReceived) {
                bFirstReceived = false;
            }

            if (gSplitImage) {
                convert_yuv_to_rgb_buffer(gColorImgBuf, gColorRGBImgBuf, gColorWidth, gColorHeight, gSplitImage);
                save_file(gColorRGBImgBuf, 3 * gColorWidth * gColorHeight, gColorWidth, gColorHeight, gColorFormat, false);
                ret = getPointCloud(EYSD, GetDevSelectIndexPtr(),
                                    gColorRGBImgBuf, gColorWidth, gColorHeight,
                                    gDepthImgBuf, gDepthWidth, gDepthHeight, gDepthDataType,
                                    pPointCloudRGB, pPointCloudXYZ, g_maxNear, g_maxFar);
            } else {
                ret = getPointCloud(EYSD, GetDevSelectIndexPtr(),
                                    gColorImgBuf, gColorWidth, gColorHeight,
                                    gColorImgBuf, gDepthWidth, gDepthHeight, gDepthDataType,
                                    pPointCloudRGB, pPointCloudXYZ, g_maxNear, g_maxFar);
                save_file(pPointCloudRGB, 3 * gColorWidth * gColorHeight, gColorWidth, gColorHeight, gColorFormat, false);
            }
        } else {
            CT_DEBUG("Failed to call APC_Get2ImageWithTimestamp()!\n");
            if (ret == APC_DEVICE_TIMEOUT) {
                CT_DEBUG("Getting image is timeout!\n");
                usleep(1 * 1000);
            }
        }
        count++;
    }

exit:
    if (gDepthImgBuf != NULL) {
        free(gDepthImgBuf);
        gDepthImgBuf = NULL;
    }

    if (gColorImgBuf != NULL) {
        free(gColorImgBuf);
        gColorImgBuf = NULL;
    }

    if (gColorRGBImgBuf != NULL) {
        free(gColorRGBImgBuf);
        gColorRGBImgBuf = NULL;
    }

    if (pPointCloudRGB != NULL) {
        free(pPointCloudRGB);
        pPointCloudRGB = NULL;
    }

    if (pPointCloudXYZ != NULL) {
        free(pPointCloudXYZ);
        pPointCloudXYZ = NULL;
    }
 
    return NULL;
}

#define THERMAL_SENSOR_ID 0x90 // (A0, A1, A2) = (0, 0, 0) =>  Target Adderss = 1001 000
static void *get_thermal_sensor_2075_temperature_test_func(void *arg)
{   
    int ret = APC_OK;
    unsigned short TempeRegAddr = 0x00;
    unsigned short TempeRegVal = 0x00;
    float fTemperature = 0.0f;
    bool is_negtive = false;

    ret = APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), THERMAL_SENSOR_ID,
        TempeRegAddr, &TempeRegVal, FG_Address_1Byte | FG_Value_2Byte, SENSOR_BOTH);

    if (ret != APC_OK) {
        CT_DEBUG("Failed to call APC_GetSensorRegister()(%d)\n", ret);
    } else {
        CT_DEBUG("TempeRegVal = (0x%04x)\n", TempeRegVal);
        TempeRegVal = __bswap_16(TempeRegVal);
        TempeRegVal >>= 5;
        is_negtive = TempeRegVal & (1 << 10);
        if (is_negtive)
            TempeRegVal |= (0x1f << 11);
        fTemperature = (float)TempeRegVal * 0.125;
        CT_DEBUG("TempeRegVal = (%f)\n", fTemperature);
    }

    return NULL;
}

#define IMU_READ_CNT 64
#define IMU_DATA_LEN 32

const char READ_OUTPUT_STAUS[8] =   {0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const char DISABLE_OUTPUT[8] =      {0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
const char ENABLE_OUTPUT[8] =       {0x00, 0x11, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
const char READ_OUTPUT_FORMAT[8] =  {0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

enum DATA_FORMAT{
    RAW_DATA_WITHOUT_OFFSET = 1,
    RAW_DATA_WITH_OFFSET,
    OFFSET_DATA,
    DMP_DATA_WITHOT_OFFSET,
    DMP_DATA_WITH_OFFSET
};

struct SetFeatureDATA_Item {
    const char* pData;
    int nDataLength;
};

DATA_FORMAT gCurrentIMUFormat;

void SendFeatureReport(hid_device *device, const char* pData, size_t data_lenght)
{
    if (device) {
        unsigned char *pBuf = nullptr;
        pBuf = (unsigned char *)calloc(data_lenght + 1, sizeof(unsigned char));
        pBuf[0] = {0x0};
        memcpy(pBuf + 1, pData, data_lenght);
        hid_send_feature_report(device, pBuf, data_lenght + 1);
        free(pBuf);
    }
}

void GetFeatureReport(hid_device *device, char* pData, size_t data_lenght)
{
    if (device) {
        unsigned char *pBuf = nullptr;
        pBuf = (unsigned char *)calloc(data_lenght + 1, sizeof(unsigned char));
        pBuf[0] = {0x0};
        hid_get_feature_report(device, pBuf, data_lenght + 1);
        memcpy(pData, pBuf + 1, data_lenght);
        free(pBuf);
    }
}

int GetIMUDataOutputByte(DATA_FORMAT format)
{
    switch (format) {
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

void ReadDataOutputFormat(hid_device *device)
{
    DATA_FORMAT nCurrentIMUFormat;
    char status[8] = {0};

    SetFeatureDATA_Item setFeatureData = {&READ_OUTPUT_FORMAT[0], (sizeof(READ_OUTPUT_FORMAT) / sizeof(READ_OUTPUT_FORMAT[0]))};

    SendFeatureReport(device, setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(device, &status[0], setFeatureData.nDataLength);
    gCurrentIMUFormat = status[0] != 0 ? (DATA_FORMAT)status[0] : RAW_DATA_WITHOUT_OFFSET;
    CT_DEBUG("CurrentIMUFormat = %x\n", gCurrentIMUFormat);
}

void EnableDataOutout(hid_device *device, bool bIsEnbale)
{
    SetFeatureDATA_Item setFeatureData;

    if (bIsEnbale) {
        setFeatureData = {&ENABLE_OUTPUT[0], (sizeof(ENABLE_OUTPUT) / sizeof(ENABLE_OUTPUT[0]))};
    } else {
        setFeatureData = {&DISABLE_OUTPUT[0], (sizeof(DISABLE_OUTPUT) / sizeof(DISABLE_OUTPUT[0]))};
    }

    SendFeatureReport(device, setFeatureData.pData, setFeatureData.nDataLength);
}

int ReadIMURawData(hid_device *device)
{
    unsigned char imuRawData[IMU_DATA_LEN] = {0};

    if (!device) {
        return APC_NullPtr;
    }

    int ret = hid_read(device, imuRawData, sizeof(imuRawData));

    if (ret < APC_OK) {
        CT_DEBUG("Failed to call hid_read()(%d)\n", ret);
    } else {
        int nIMUDataByte = GetIMUDataOutputByte(gCurrentIMUFormat);
        for (int i = 0; i < IMU_DATA_LEN; i++) {
            printf("%02x ", imuRawData[i]);
        }
        printf("\n");
    }

    return ret;
}

static void *imu_test_func_for_YX8062(void *arg)
{
    int ret = APC_OK;
    unsigned short nVID = 0x1E4E, nPID = 0x0163;

    hid_init();

    hid_device_info *deviceInfo = hid_enumerate(nVID, nPID);
    hid_device_info *headInfo = deviceInfo;
    if (deviceInfo) {
        hid_device *device = hid_open_path(deviceInfo->path);
        CT_DEBUG("Find YX8062 IMU Device (VID, PID, Path) = (0x%04x, 0x%04x, %s)\n", nVID, nPID, deviceInfo->path);
        if (device) {
            hid_set_nonblocking(device, true);
            ReadDataOutputFormat(device);
            EnableDataOutout(device, true);
            for (int i = 0; i < IMU_READ_CNT; i++) {
                printf("IMURawData#%02d = ", i);
                ReadIMURawData(device);
                usleep(5 * 1000); // 200HZ
            }

            hid_close(device);
            hid_exit();
        }
    }

    return NULL;
}

static int keycode_of_key_being_pressed() {
  FILE *kbd;
  glob_t kbddev;                                   // Glob structure for keyboard devices
  glob("/dev/input/by-path/*-kbd", 0, 0, &kbddev); // Glob select all keyboards
  int keycode = -1;                                // keycode of key being pressed
  for (int i = 0; i < kbddev.gl_pathc ; i++ ) {    // Loop through all the keyboard devices ...
    if (!(kbd = fopen(kbddev.gl_pathv[i], "r"))) { // ... and open them in turn (slow!)
      perror("Run as root to read keyboard devices");
      exit(1);
    }

    char key_map[KEY_MAX/8 + 1];          // Create a bit array the size of the number of keys
    memset(key_map, 0, sizeof(key_map));  // Fill keymap[] with zero's
    ioctl(fileno(kbd), EVIOCGKEY(sizeof(key_map)), key_map); // Read keyboard state into keymap[]
    for (int k = 0; k < KEY_MAX/8 + 1 && keycode < 0; k++) { // scan bytes in key_map[] from left to right
      for (int j = 0; j <8 ; j++) {       // scan each byte from lsb to msb
        if (key_map[k] & (1 << j)) {      // if this bit is set: key was being pressed
          keycode = 8*k + j ;             // calculate corresponding keycode
          break;                          // don't scan for any other keys
        }
      }
    }

    fclose(kbd);
    if (keycode)
      break;                              // don't scan for any other keyboards
  }
  return keycode;
}

static void test_pause_resume_streaming(void)
{
    pthread_t kbhit_thread_id;
    pthread_attr_t kbhit_thread_attr;
    struct sched_param kbhit_thread_param;

    pthread_t color_thread_id;
    pthread_attr_t color_thread_attr;
    struct sched_param color_thread_param;

    pthread_t depth_thread_id;
    pthread_attr_t depth_thread_attr;
    struct sched_param depth_thread_param;

    pthread_attr_init(&kbhit_thread_attr);
    pthread_attr_getschedparam (&kbhit_thread_attr, &kbhit_thread_param);
    kbhit_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&kbhit_thread_attr, &kbhit_thread_param);

    pthread_attr_init(&color_thread_attr);
    pthread_attr_setschedpolicy(&color_thread_attr, SCHED_FIFO);
    pthread_attr_getschedparam (&color_thread_attr, &color_thread_param);
    color_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&color_thread_attr, &color_thread_param);

    pthread_attr_init(&depth_thread_attr);
    pthread_attr_setschedpolicy(&depth_thread_attr, SCHED_FIFO);
    pthread_attr_getschedparam (&depth_thread_attr, &depth_thread_param);
    depth_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_attr_setschedparam(&depth_thread_attr, &depth_thread_param);

    pthread_create(&kbhit_thread_id, &kbhit_thread_attr, pfunc_thread_keyboard_handler, NULL);
    pthread_create(&color_thread_id, &color_thread_attr, test_color_time_stamp, NULL);
    pthread_create(&depth_thread_id, &depth_thread_attr, test_depth_time_stamp, NULL);

    pthread_join(kbhit_thread_id, NULL);
    pthread_join(depth_thread_id, NULL);
    pthread_join(color_thread_id, NULL);

}

static void *pfunc_thread_keyboard_handler(void *arg) {
    //setvbuf(stdout, NULL, _IONBF, 0); // Set stdout unbuffered
    CT_DEBUG("===================================================================================\n");
    CT_DEBUG("== pressed KEY_P:Pause image streaming; KEY_R:Resume image streaming; KEY_Q:Exit ==\n");
    CT_DEBUG("===================================================================================\n");
    while (1)    {
        int key = keycode_of_key_being_pressed();
        //printf((key < 0 ?  "no key\n" : "keycode: %d\n"), key);
        if (key == KEY_Q) {
            CT_DEBUG("\nKEY_Q:Exit\n");
            bKeepStreaming = false;
            break;
        } else if (key == KEY_P) {
            CT_DEBUG("\nKEY_P:Pause image streaming\n");
            pause_streaming();
        } else if (key == KEY_R) {
            CT_DEBUG("\nKEY_R:Resume image streaming\n");
            resume_streaming();
        } else {
            //CT_DEBUG("keycode:%d\n", key);
        }
        sleep(0.5);
    }
    CT_DEBUG("\nexit thread_keyboard_handler\n");
    pthread_exit(NULL);
}

static void pause_streaming()
{
    int flag = 0;
    flag |= FG_Address_2Byte;
    flag |= FG_Value_2Byte;
    SENSORMODE_INFO SensorMode = SENSOR_BOTH;
    unsigned short value = 0;
    CT_DEBUG("pause_streaming:0x%02x\n", gCameraPID);

    if (gCameraPID == APC_PID_8036) {
        if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag, SensorMode) != APC_OK)
        {
            CT_DEBUG("failed to pause streaming - get original sensor value failed!\n");
        } else {
            value = high_low_exchange(value);
            CT_DEBUG("pause streaming get original sensor value:0x%04x\n", value);
            BIT_CLEAR(value, 2);
            CT_DEBUG("pause streaming set sensor value:0x%04x\n", value);
            if (APC_SetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, value, flag,
                                      SensorMode) != APC_OK) {
                CT_DEBUG("failed to pause streaming - set sensor value failed!\n");
            } else {
                if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag,
                                          SensorMode) != APC_OK) {
                    CT_DEBUG("failed to pause streaming - get after sensor value failed!\n");
                } else {
                    value = high_low_exchange(value);
                    CT_DEBUG("pause streaming get after sensor value:0x%04x\n", value);
                }
            }
        }
    } else if ((gCameraPID == APC_PID_80362)) {
        //80362
        if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag, SensorMode) != APC_OK)
        {
            CT_DEBUG("failed to pause streaming - get original sensor value failed!\n");
        } else {
            value = high_low_exchange(value);
            CT_DEBUG("pause streaming get original sensor value:0x%04x\n", value);
            BIT_CLEAR(value, 8);
            CT_DEBUG("pause streaming set sensor value:0x%04x\n", value);
            if (APC_SetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, value, flag,
                                      SensorMode) != APC_OK) {
                CT_DEBUG("failed to pause streaming - set sensor value failed!\n");
            } else {
                if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag,
                                          SensorMode) != APC_OK) {
                    CT_DEBUG("failed to pause streaming - get after sensor value failed!\n");
                } else {
                    value = high_low_exchange(value);
                    CT_DEBUG("pause streaming get after sensor value:0x%04x\n", value);
                }
            }
        }
    }
}

static void resume_streaming()
{
    int flag = 0;
    flag |= FG_Address_2Byte;
    flag |= FG_Value_2Byte;
    SENSORMODE_INFO SensorMode = SENSOR_BOTH;
    unsigned short value = 0;
    CT_DEBUG("resume_streaming:0x%02x\n", gCameraPID);

    if (gCameraPID == APC_PID_8036) {
        if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag, SensorMode) != APC_OK)
        {
            CT_DEBUG("failed to resume streaming - get original sensor value failed!\n");
        } else {
            value = high_low_exchange(value);
            CT_DEBUG("resume streaming get original sensor value:0x%04x\n", value);
            BIT_SET(value, 2);
            CT_DEBUG("resume streaming set sensor value:0x%04x\n", value);
            if (APC_SetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, value, flag, SensorMode) != APC_OK) {
                CT_DEBUG("failed to resume streaming - set sensor value failed!\n");
            } else {
                if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag, SensorMode) != APC_OK) {
                    CT_DEBUG("failed to resume streaming - get after sensor value failed!\n");
                } else {
                    value = high_low_exchange(value);
                    CT_DEBUG("resume streaming get after sensor value:0x%04x\n", value);
                }
            }
        }
    } else if ((gCameraPID == APC_PID_80362)) {
        //80362
        if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag, SensorMode) != APC_OK)
        {
            CT_DEBUG("failed to resume streaming - get original sensor value failed!\n");
        } else {
            value = high_low_exchange(value);
            CT_DEBUG("resume streaming get original sensor value:0x%04x\n", value);
            BIT_SET(value, 8);
            CT_DEBUG("resume streaming set sensor value:0x%04x\n", value);
            if (APC_SetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, value, flag, SensorMode) != APC_OK) {
                CT_DEBUG("failed to resume streaming - set sensor value failed!\n");
            } else {
                if (APC_GetSensorRegister(EYSD, GetDevSelectIndexPtr(), 0x30, 0x301a, &value, flag, SensorMode) != APC_OK) {
                    CT_DEBUG("failed to resume streaming - get after sensor value failed!\n");
                } else {
                    value = high_low_exchange(value);
                    CT_DEBUG("resume streaming get after sensor value:0x%04x\n", value);
                }
            }
        }
    }
}

static int high_low_exchange(int data)
{
    int data_low;
    int data_high;
    data_low = (data & 0xff00) >> 8;
    data_high = (data & 0x00ff) << 8;
    int exchange_data = data_high + data_low;
    return exchange_data;
}
