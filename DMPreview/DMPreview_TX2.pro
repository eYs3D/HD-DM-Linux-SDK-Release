#-------------------------------------------------
#
# Project created by QtCreator 2020-03-24T09:27:12
#
#-------------------------------------------------

QT       += core gui dbus

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DMPreview
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += $$PWD/model/
INCLUDEPATH += $$PWD/model/module/
INCLUDEPATH += $$PWD/view/
INCLUDEPATH += $$PWD/controller/
INCLUDEPATH += $$PWD/manager/
INCLUDEPATH += $$PWD/utility/
INCLUDEPATH += $$PWD/utility/sqlite3/
INCLUDEPATH += $$PWD/utility/ModuleCalibParWrite/
INCLUDEPATH += $$PWD/../eSPDI/
INCLUDEPATH += $$PWD/../eSPDI/self_calibration/Self_Calibration_API/aarch64/include/


LIBS += -L$$PWD/../eSPDI -leSPDI_NVIDIA_64
LIBS += -L$$PWD/../eSPDI/self_calibration/Self_Calibration_API/aarch64/lib/ -lSelf_Calibration_API
LIBS += -lrt -lm -ldl -lpthread -ludev -lGL -lGLU
CONFIG += c++11

QMAKE_LFLAGS += '-Wl,-rpath,\'\$$ORIGIN/../eSPDI\''

QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter
QMAKE_CXXFLAGS_WARN_ON += -Wno-reorder
QMAKE_CXXFLAGS += -ffast-math

FORMS += \
    view/mainwindow.ui \
    view/CVideoDeviceDialog.ui \
    view/CVideoDevicePreviewWidget.ui \
    view/CVideoDeviceRegisterWidget.ui \
    view/CVideoDeviceCameraPropertyWidget.ui \
    view/CVideoDeviceIMUWidget.ui \
    view/CVideoDeviceAudioWidget.ui \
    view/CVideoDeviceDepthAccuracyWidget.ui \
    view/CVideoDeviceDepthFilterWidget.ui \
    view/CSelfCalibrationWidget.ui \
    view/CSparseModeWidget.ui


HEADERS += \
    controller/CVideoDeviceController.h \
    controller/CRegisterReadWriteController.h \
    controller/CCameraPropertyController.h \
    controller/CIMUDataController.h \
    controller/CDepthAccuracyController.h \
    controller/CDepthFilterController.h \
    controller/CImageProcessController.h \
    controller/CSelfcalibrationController.h \
    controller/CSparseModeController.h \
    manager/CEYSDDeviceManager.h \
    manager/CThreadWorkerManage.h \
    manager/CTaskThread.h \
    manager/CTaskInfoManager.h \
    manager/CMessageManager.h \
    manager/CIMUDeviceManager.h \
    manager/CFrameSyncManager.h \
    model/CVideoDeviceModel.h \
    model/CCameraPropertyModel.h \
    model/PreviewOptions.h \
    model/CIMUModel.h \
    model/CVideoDeviceModelFactory.h \
    model/CImageDataModel.h \
    model/RegisterReadWriteOptions.h \
    model/ModeConfigOptions.h \
    model/DepthFilterOptions.h \
    model/module/CVideoDeviceModel_8029.h \
    model/module/CVideoDeviceModel_8036_8052.h \
    model/module/CVideoDeviceModel_8036.h \
    model/module/CVideoDeviceModel_8037.h \
    model/module/CVideoDeviceModel_8038.h \
    model/module/CVideoDeviceModel_8040_8054.h \
    model/module/CVideoDeviceModel_8040.h \
    model/module/CVideoDeviceModel_8051.h \
    model/module/CVideoDeviceModel_8052.h \
    model/module/CVideoDeviceModel_8053_8059.h \
    model/module/CVideoDeviceModel_8053.h \
    model/module/CVideoDeviceModel_8054.h \
    model/module/CVideoDeviceModel_8060.h \
    model/module/CVideoDeviceModel_8062.h \
    model/module/CVideoDeviceModel_8063.h \
    model/module/CVideoDeviceModel_8073.h \
    model/module/CVideoDeviceModel_8076_8077.h \
    model/module/CVideoDeviceModel_8081.h \
    model/module/CVideoDeviceModel_8083.h \
    model/module/CVideoDeviceModel_ColorWithDepth.h \
    model/module/CVideoDeviceModel_Kolor.h \
    model/module/CVideoDeviceModel_Iris.h \
    model/module/CVideoDeviceModel_IVY3.h \
    model/module/CVideoDeviceModel_IVY4.h \
    model/module/CVideoDeviceModel_Hypatia.h \
    model/module/CVideoDeviceModel_Hypatia2.h \
    model/module/CVideoDeviceModel_Nora.h \
    model/module/CVideoDeviceModel_Grap.h \
    utility/PlyWriter.h \
    utility/utDisplayMetrics.h \
    utility/utImageProcessingUtility.h \
    utility/ColorPaletteGenerator.h \
    utility/PlyFilter.h \
    utility/RegisterSettings.h \
    utility/ModeConfig.h \
    utility/IMUData.h \
    utility/DepthFusionHelper.h \
    utility/OpenGLShaderCore.h \
    utility/FrameGrabber.h \
    utility/FPSCalculator.h \
    utility/ArcBall.h \
    utility/ArcBallMatrix.h \
    utility/OGLBasic.h \
    utility/sqlite3/sqlite3.h \
    utility/sqlite3/sqlite3ext.h \
    utility/hidapi/hidapi.h \
    utility/IMU/IMU_Filter/IMUFilter_AHRS.h \
    utility/IMU/IMU_Filter/IMUFilter_Complementary.h \
    utility/IMU/IMU_Filter/IMUFilter.h \
    utility/IMU/IMU_Filter/StatelessOrientation.h \
    utility/IMU/IMU_Filter/IMUFilter_EYS3D_AHRS.h \
    utility/IMU/IMU_Filter/quaternion.h \
    utility/IMU/IMU_Filter/Quaternion1.h \
    utility/IMU/IMU_Calibration/CIMUCalibration.h \
    utility/ModuleCalibParWrite/ModuleCalibParWrite.h \
    view/mainwindow.h \
    view/CVideoDeviceDialog.h \
    view/CVideoDevicePreviewWidget.h \
    view/CEYSDUIView.h \
    view/CPreviewDialog.h \
    view/CMessageDialog.h \
    view/CVideoDeviceRegisterWidget.h \
    view/CVideoDeviceCameraPropertyWidget.h \
    view/CVideoDeviceIMUWidget.h \
    view/CVideoDeviceAudoWidget.h \
    view/CPointCloudViewerWidget.h \
    view/CPointCloudViewerDialog.h \
    view/CVideoDeviceDepthAccuracyWidget.h \
    view/CIMUDataViewerWidget.h \
    view/CVideoDeviceDepthFilterWidget.h \
    view/CSelfCalibrationWidget.h \
    view/CSparseModeWidget.h


SOURCES += \
    main.cpp \
    controller/CVideoDeviceController.cpp \
    controller/CRegisterReadWriteController.cpp \
    controller/CCameraPropertyController.cpp \
    controller/CIMUDataController.cpp \
    controller/CDepthAccuracyController.cpp \
    controller/CDepthFilterController.cpp \
    controller/CImageProcessController.cpp \
    controller/CSelfcalibrationController.cpp \
    controller/CSparseModeController.cpp \
    manager/CEYSDDeviceManager.cpp \
    manager/CThreadWorkerManage.cpp \
    manager/CTaskThread.cpp \
    manager/CTaskInfoManager.cpp \
    manager/CMessageManager.cpp \
    manager/CIMUDeviceManager.cpp \
    manager/CFrameSyncManager.cpp \
    model/CVideoDeviceModel.cpp \
    model/CCameraPropertyModel.cpp \
    model/PreviewOptions.cpp \
    model/CIMUModel.cpp \
    model/CVideoDeviceModelFactory.cpp \
    model/CImageDataModel.cpp \
    model/RegisterReadWriteOptions.cpp \
    model/ModeConfigOptions.cpp \
    model/DepthFilterOptions.cpp \
    model/module/CVideoDeviceModel_8029.cpp \
    model/module/CVideoDeviceModel_8036_8052.cpp \
    model/module/CVideoDeviceModel_8036.cpp \
    model/module/CVideoDeviceModel_8037.cpp \
    model/module/CVideoDeviceModel_8038.cpp \
    model/module/CVideoDeviceModel_8040_8054.cpp \
    model/module/CVideoDeviceModel_8040.cpp \
    model/module/CVideoDeviceModel_8051.cpp \
    model/module/CVideoDeviceModel_8052.cpp \
    model/module/CVideoDeviceModel_8053_8059.cpp \
    model/module/CVideoDeviceModel_8053.cpp \
    model/module/CVideoDeviceModel_8054.cpp \
    model/module/CVideoDeviceModel_8060.cpp \
    model/module/CVideoDeviceModel_8062.cpp \
    model/module/CVideoDeviceModel_8063.cpp \
    model/module/CVideoDeviceModel_8073.cpp \
    model/module/CVideoDeviceModel_8076_8077.cpp \
    model/module/CVideoDeviceModel_8081.cpp \
    model/module/CVideoDeviceModel_8083.cpp \
    model/module/CVideoDeviceModel_ColorWithDepth.cpp \
    model/module/CVideoDeviceModel_Kolor.cpp \
    model/module/CVideoDeviceModel_Iris.cpp \
    model/module/CVideoDeviceModel_IVY3.cpp \
    model/module/CVideoDeviceModel_IVY4.cpp \
    model/module/CVideoDeviceModel_Hypatia.cpp \
    model/module/CVideoDeviceModel_Hypatia2.cpp \
    model/module/CVideoDeviceModel_Nora.cpp \
    model/module/CVideoDeviceModel_Grap.cpp \
    utility/PlyWriter.cpp \
    utility/utDisplayMetrics.cpp \
    utility/utImageProcessingUtility.cpp \
    utility/ColorPaletteGenerator.cpp \
    utility/PlyFilter.cpp \
    utility/RegisterSettings.cpp \
    utility/ModeConfig.cpp \
    utility/IMUData.cpp \
    utility/DepthFusionHelper.cpp \
    utility/FrameGrabber.cpp \
    utility/FPSCalculator.cpp \
    utility/ArcBall.cpp \
    utility/ArcBallMatrix.cpp \
    utility/sqlite3/sqlite3.c \
    utility/hidapi/hid_udev.c \
    utility/IMU/IMU_Filter/IMUFilter_AHRS.cpp \
    utility/IMU/IMU_Filter/IMUFilter_Complementary.cpp \
    utility/IMU/IMU_Filter/StatelessOrientation.cpp \
    utility/IMU/IMU_Filter/IMUFilter_EYS3D_AHRS.cpp \
    utility/IMU/IMU_Filter/quaternion.cpp \
    utility/IMU/IMU_Filter/Quaternion1.c \
    utility/IMU/IMU_Calibration/CIMUCalibration.cpp \
    utility/ModuleCalibParWrite/ModuleCalibParWrite.cpp \
    view/mainwindow.cpp \
    view/CVideoDeviceDialog.cpp \
    view/CVideoDevicePreviewWidget.cpp \
    view/CEYSDUIView.cpp \
    view/CPreviewDialog.cpp \
    view/CMessageDialog.cpp \
    view/CVideoDeviceRegisterWidget.cpp \
    view/CVideoDeviceCameraPropertyWidget.cpp \
    view/CVideoDeviceIMUWidget.cpp \
    view/CVideoDeviceAudoWidget.cpp \
    view/CPointCloudViewerWidget.cpp \
    view/CPointCloudViewerDialog.cpp \
    view/CVideoDeviceDepthAccuracyWidget.cpp \
    view/CIMUDataViewerWidget.cpp \
    view/CVideoDeviceDepthFilterWidget.cpp \
    view/CSelfCalibrationWidget.cpp \
    view/CSparseModeWidget.cpp


RESOURCES += \
    resource/resource.qrc

DEFINES += UAC_X86_SUPPORTED
