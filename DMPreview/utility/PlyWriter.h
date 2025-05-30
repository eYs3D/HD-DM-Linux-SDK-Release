#pragma once
#include <string>
#include <vector>
#include <eSPDI.h>

struct CloudPoint {
	float x;
	float y;
	float z;
	unsigned char r;
	unsigned char g;
	unsigned char b;
};
class PlyWriter {
	PlyWriter();
public:
    static int writePly(std::vector<CloudPoint>& cloud,  std::string filename);
	//static int EYSDFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output,float zNear,float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
	//static int EYSDFrameTo3D(int depthWidth, int depthHeight ,std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool removeINF, bool useDepthResolution, float scale_ratio);
    static int EYSDFrameTo3D_8029(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
    static int EYSDFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
    static int EYSDFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool useDepthResolution, float scale_ratio, bool isMipiDevice);
	static int EYSDFrameTo3D_PlyFilterFloat(int depthWidth, int depthHeight, std::vector<float>& dFloatArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
	static int EYSDFrameTo3DMultiSensor(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool isDownSampleK, float scale_ratio, int degreeOfRectifyLogK);
	static int EYSDFrameTo3DMultiSensorPlyFilterFloat(int depthWidth, int depthHeight, std::vector<float>& dFloatArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool isDownSampleK, float scale_ratio, int degreeOfRectifyLogK);
	static int EYSDFrameTo3DCylinder(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, float scale_ratio);
	static int EYSDFrameTo3DCylinder(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, float scale_ratio);
    static int EYSDFrameTo3D_8063(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray,
                                       eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData* rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping,
                                       float zNear,float zFar,bool removeINF = true, bool useDepthResolution = true, float scale_ratio = 1.0f);
	static std::string generateHeader(bool binary,int size);
//private:
	static void resampleImage(int srcWidth, int srcHeight, unsigned char* srcBuf, int dstWidth, int dstHeight, unsigned char* dstBuf, int bytePerPixel);  
	//static void resampleImage_float(int srcWidth, int srcHeight, float* srcBuf, int dstWidth, int dstHeight, float* dstBuf);
    static void MonoBilinearFineScaler(unsigned char* pIn, unsigned char* pOut, int width, int height, int outwidth, int outheight, int zero_invalid);
	static void MonoBilinearFineScaler(unsigned char *pIn, unsigned char *pOut, int width, int height, int outwidth, int outheight, int zero_invalid, int bytePerPixel);
    static void MonoBilinearFineScaler_short(unsigned short* pIn, unsigned short* pOut, int width, int height, int outwidth, int outheight, int zero_invalid);
	static void MonoBilinearFineScaler_float(float* pIn, float* pOut, int width, int height, int outwidth, int outheight, int zero_invalid);
	
};

