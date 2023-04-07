#ifndef CIMAGEPROCESSCONTROLLER_H
#define CIMAGEPROCESSCONTROLLER_H
#include <eSPDI.h>
#include <math.h>
#include <map>
#include <functional>
#include <CVideoDeviceController.h>

#ifndef DISALLOW_COPY_AND_ASSIGN

#define DISALLOW_COPY_AND_ASSIGN(T) \
    T(const T& other) = delete; \
    T& operator=(const T& other) = delete

#endif

class ResizeImgHandler {
    DISALLOW_COPY_AND_ASSIGN(ResizeImgHandler);
public:
    // Common variables
    struct ResolutionResized {
        size_t width = 0u;
        size_t height = 0u;
    };

    APCImageType::Value mType;
    size_t mInWidth = 0u;
    size_t mInHeght = 0u;
    std::map<int, void *> mIndexResizeHandleMap;
    std::map<int, ResolutionResized> mResizedResolutionMap;

    ResizeImgHandler(size_t inWidth, size_t inHeight, APCImageType::Value type) :
        mInWidth(inWidth), mInHeght(inHeight), mType(type) {

    }

    void* getCurrentResizeHandle(const int optionIndex) {
        auto it = mIndexResizeHandleMap.find(optionIndex);
        if (it == mIndexResizeHandleMap.end()) {
            fprintf(stderr, "get mIndexResizeHandleMap failed \n");
            return nullptr;
        }
        return it->second;
    }

    ResolutionResized getCurrentResolution(const int optionIndex) {
        auto it = mResizedResolutionMap.find(optionIndex);
        if (it == mResizedResolutionMap.end()) {
            fprintf(stderr, "getCurrentResolution failed \n");
            ResolutionResized failed;
            failed.width = 0;
            failed.height = 0;
            return failed;
        }
        return it->second;
    }

    // By children
    virtual inline double indexToScaleFactor(const int optionIndex) = 0;
    virtual int process(const int optionIndex, const std::vector<uint8_t>& srcBuffer,
                        std::vector<uint8_t>& dstBuffer, APCImageType::Value imageType) = 0;
};

class ColorResizeHandler : public ResizeImgHandler {
    DISALLOW_COPY_AND_ASSIGN(ColorResizeHandler);
public:
    ColorResizeHandler(size_t inWidth, size_t inHeight,
                       APCImageType::Value type = APCImageType::Value::COLOR_RGB24)
        : ResizeImgHandler(inWidth, inHeight, type) {

        for (int optionIndex = 0; optionIndex <= 2; optionIndex++) {
            float shrinkFactor = indexToScaleFactor(optionIndex);
            void *handle = nullptr;
            ResolutionResized rr;
            int ret = APC_InitResizeProcess(&handle, inWidth, inHeight, &rr.width, &rr.height,
                                            mType, shrinkFactor);
            mIndexResizeHandleMap.emplace(std::make_pair(optionIndex, handle));
            mResizedResolutionMap.emplace(std::make_pair(optionIndex, rr));
            if (APC_OK != ret) {
                fprintf(stderr, "APC_InitResizeProcess APC_OK\n");
            }
        }

    }

    inline double indexToScaleFactor(const int optionIndex) override {
        return pow(0.5, optionIndex);
    }

    int process(const int optionIndex, const std::vector<uint8_t>& srcBuffer, std::vector<uint8_t>& dstBuffer,
                APCImageType::Value imageType) override {
        void *selectedHandle = getCurrentResizeHandle(optionIndex);
        return APC_ResizeProcess(selectedHandle, srcBuffer, dstBuffer, imageType);
    }

    ~ColorResizeHandler() {
        for (auto it : mIndexResizeHandleMap) {
            int ret = APC_ReleaseResizeProcess(it.second);
            if (APC_OK != ret) {
                fprintf(stderr, "~APC_ReleaseResizeProcess !APC_OK\n");
            }
        }
    }
};

class DepthResizeHandler : public ResizeImgHandler {
public:
    DepthResizeHandler(size_t inWidth, size_t inHeight,
                       APCImageType::Value type = APCImageType::Value::DEPTH_11BITS)
        : ResizeImgHandler(inWidth, inHeight, type) {

        for (int optionIndex = 0; optionIndex <= 2; optionIndex++) {
            DECIMATION_PARAMS shrinkFactor;
            shrinkFactor.decimation_sub_sample_factor = indexToScaleFactor(optionIndex);
            void *handle = nullptr;
            ResolutionResized rr;
            int ret = APC_InitDecimationFilter(&handle, inWidth, inHeight,
                                               (unsigned int *) &rr.width, (unsigned int *) &rr.height,
                                               mType, shrinkFactor);
            mIndexResizeHandleMap.emplace(std::make_pair(optionIndex, handle));
            mResizedResolutionMap.emplace(std::make_pair(optionIndex, rr));
            if (APC_OK != ret) {
                fprintf(stderr, "APC_InitDecimationFilter failed \n");
            }
        }
    }

    inline double indexToScaleFactor(const int optionIndex) override {
        return pow(2, optionIndex);
    }

    int process(const int optionIndex, const std::vector<uint8_t>& srcBuffer, std::vector<uint8_t>& dstBuffer,
                APCImageType::Value imageType) override {
        void *selectedHandle = getCurrentResizeHandle(optionIndex);
        return APC_DecimationFilter(selectedHandle, (unsigned char *) srcBuffer.data(), dstBuffer.data(), imageType);
    }

    ~DepthResizeHandler() {
        for (auto it : mIndexResizeHandleMap) {
            int ret = APC_ReleaseDecimationFilter(it.second);
            if (APC_OK != ret) {
                fprintf(stderr, "~APC_ReleaseDecimationFilter failed \n");
            }
        }
    }
};

class CImageProcessController
{
public:
    using ImageProcessControllerCallback = std::function<int(bool)>;

    std::unique_ptr<ResizeImgHandler> mColorResizeHandle;
    std::unique_ptr<ResizeImgHandler> mDepthResizeHandle;
    APCImageType::Value mDepthImageType;

    void process(const int optionIndex, const std::vector<uint8_t>& srcBuffer, std::vector<uint8_t>& dstBuffer,
                 APCImageType::Value imageType) {
        switch (imageType) {
        case APCImageType::COLOR_YUY2:
        case APCImageType::COLOR_RGB24:
            mColorResizeHandle->process(optionIndex, srcBuffer, dstBuffer, imageType);
            break;
        case APCImageType::DEPTH_11BITS:
        case APCImageType::DEPTH_14BITS:
            mDepthResizeHandle->process(optionIndex, srcBuffer, dstBuffer, imageType);
            break;
        default:
            // No operation
            break;
        }
    }

    ResizeImgHandler::ResolutionResized GetResizedResolution(const int optionIndex, APCImageType::Value imageType){
        ResizeImgHandler* handle;
        switch (imageType) {
        case APCImageType::COLOR_YUY2:
            handle = mColorResizeHandle.get();
            return handle->getCurrentResolution(optionIndex);
        case APCImageType::DEPTH_11BITS:
        case APCImageType::DEPTH_14BITS:
            handle = mDepthResizeHandle.get();
            return handle->getCurrentResolution(optionIndex);
        default:
            ResizeImgHandler::ResolutionResized err;
            err.width = 0;
            err.height = 0;
            return err;
        }
    }

    CImageProcessController(size_t colorWidth, size_t colorHeight, size_t depthWidth, size_t depthHeight,
                            APCImageType::Value imageType) : mDepthImageType(imageType) {
        mColorResizeHandle.reset(new ColorResizeHandler(colorWidth, colorHeight, APCImageType::COLOR_YUY2));
        mDepthResizeHandle.reset(new DepthResizeHandler(depthWidth, depthHeight, mDepthImageType));
    }

};

#endif // CIMAGEPROCESSCONTROLLER_H
