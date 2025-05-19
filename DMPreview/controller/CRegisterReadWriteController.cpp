#include "CRegisterReadWriteController.h"
#include "RegisterReadWriteOptions.h"
#include "CThreadWorkerManage.h"
#include "CTaskInfoManager.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceModel.h"
#include "eSPDI.h"
#include <iostream>
#include <sstream>
#include <QMessageBox>

CRegisterReadWriteController::CRegisterReadWriteController(CVideoDeviceModel *pVideoDeviceModel):
m_pVideoDeviceModel(pVideoDeviceModel),
m_pRegisterTaskInfo(nullptr),
m_SwapBytesOrder(false)
{
    m_pRegisterReadWriteOptions = new RegisterReadWriteOptions();
}

int  CRegisterReadWriteController::GetCurrentCameraIndex() {
    return m_pRegisterReadWriteOptions->GetCurrentDeviceIndex();
}

void CRegisterReadWriteController::SetCurrentCameraIndex(int nIndex) {
    m_pRegisterReadWriteOptions->SetCurrentDeviceIndex(nIndex);
}

std::map<std::string, int> CRegisterReadWriteController::GetCurrentDeviceMap() {
    auto deviceSelInfoList = GetVideoDeviceModel()->GetDeviceSelInfo();
    auto deviceInfoList = GetVideoDeviceModel()->GetDeviceInformation();

    std::map<std::string, int> deviceDescriptionMap;
    for(int i = 0; i < deviceSelInfoList.size(); i++) {
        std::stringstream pidString;
        pidString << "PID_" << std::hex << std::uppercase << deviceInfoList[i].deviceInfomation.wPID;
        std::string deviceDescriptionString = "";
        deviceDescriptionString.append(pidString.str());
        deviceDescriptionString.append("_");
        deviceDescriptionString.append(deviceInfoList[i].sFWVersion);
        deviceDescriptionString.append("_");
        deviceDescriptionString.append(deviceInfoList[i].sSerialNumber);

        deviceDescriptionMap[std::move(deviceDescriptionString)] = deviceSelInfoList[i]->index;
    }

    return std::move(deviceDescriptionMap);
}

CRegisterReadWriteController::~CRegisterReadWriteController()
{
    if (m_pRegisterReadWriteOptions) delete m_pRegisterReadWriteOptions;
}

int CRegisterReadWriteController::StartReadRegister()
{
    if (m_pRegisterReadWriteOptions->IsPerodicRead()){
        StartPeriodicRead();
    }else{
        ReadRegister();
    }

    return APC_OK;
}

int CRegisterReadWriteController::StartPeriodicRead()
{
    if (m_pRegisterTaskInfo) CThreadWorkerManage::GetInstance()->RemoveTask(m_pRegisterTaskInfo);

    m_bPeriodicReadRunning = true;
    SetLastestReadTime(QTime::currentTime());

    if (m_pRegisterReadWriteOptions->IsSaveLog()){
        QDate date = QDate::currentDate();
        QTime time = QTime::currentTime();
        sprintf(m_pLogFileName, "RegisterLog%04d%02d%02d%02d%02d%02d.txt", date.year(), date.month(), date.day(),
                time.hour(), time.minute(), time.second());

        char buf[512] = {0};
        sprintf(buf, "../out/ReadRegisterLog/%s" , m_pLogFileName);
        m_pLogFile = fopen(buf, "wt");
    }

    m_pRegisterTaskInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::REGISTER_PERIODIC_READ, this);
    CThreadWorkerManage::GetInstance()->AddTask(m_pRegisterTaskInfo);

    return APC_OK;
}

int CRegisterReadWriteController::StopPeriodicRead()
{
    CThreadWorkerManage::GetInstance()->RemoveTask(m_pRegisterTaskInfo);
    m_pRegisterTaskInfo = nullptr;

    if (m_pLogFile){
        fclose(m_pLogFile);
        m_pLogFile = nullptr;
        QString sLogInfo;
        sLogInfo.sprintf("Logs are saved to \n \"../out/ReadRegisterLog/%s.\"", m_pLogFileName);
        QMessageBox::information(NULL, "Register Logs", sLogInfo, QMessageBox::Yes);
    }

    m_bPeriodicReadRunning = false;
    return APC_OK;
}

int CRegisterReadWriteController::LogRegister()
{
    FILE *pFile = GetLogFile();
    if (!pFile) return APC_NullPtr;

    fprintf(pFile, "-----------------------------------------------\n");

    QDate date = QDate::currentDate();
    QTime time = QTime::currentTime();
    fprintf(pFile, "[Time] %d-%d-%d %d:%d:%d'%d\" \n", date.year(), date.month(), date.day(),
                                                    time.hour(), time.minute(), time.second(), time.msec());

    QString type = "NONE";
    switch (m_pRegisterReadWriteOptions->GetType()){
        case RegisterReadWriteOptions::IC2:
            type = "IC2";
            break;
        case RegisterReadWriteOptions::ASIC:
            type = "ASIC";
            break;
        case RegisterReadWriteOptions::FW:
            type = "FW";
            break;
        default:
            break;
    }

    fprintf(pFile, "[Type] %s \n", type.toLocal8Bit().data());

    if (EOF != m_pRegisterReadWriteOptions->GetSlaveID()){
        fprintf(pFile, "[ID] 0x%x \n", m_pRegisterReadWriteOptions->GetSlaveID());
    }

    fprintf(pFile, "[Size] Address_%dByte | Value_%dByte \n",
            FG_Address_1Byte == m_pRegisterReadWriteOptions->GetAddressSize() ? 1 : 2,
            FG_Value_1Byte == m_pRegisterReadWriteOptions->GetValueSize() ? 1 : 2);

    QString sensorMode;
    switch (m_pRegisterReadWriteOptions->GetSensorMode()){
        case SENSOR_A:
            sensorMode = "Sensor 1";
            break;
        case SENSOR_B:
            sensorMode = "Sensor 2";
            break;
        case SENSOR_C:
            sensorMode = "Sensor 3";
            break;
        case SENSOR_D:
            sensorMode = "Sensor 4";
            break;
        case SENSOR_BOTH:
            sensorMode = "Sensor all";
            break;
        default:
            break;
    }

    fprintf(pFile, "[Sensor] %s \n", sensorMode.toLocal8Bit().data());

    for (int i = 0 ; i < REGISTER_REQUEST_MAX_COUNT ; ++i){
        int address = m_pRegisterReadWriteOptions->GetRequestAddress(i);
        int value = m_pRegisterReadWriteOptions->GetRequestValue(i);
        if (EOF == address || EOF == value) continue;

        fprintf(pFile, "[Address, Value] 0x%04x, 0x%04x \n", address, value);
    }

    fprintf(pFile, "-----------------------------------------------\n");
    fflush(pFile);

    return APC_OK;
}

int CRegisterReadWriteController::ReadRegister()
{
    for (int i = 0 ; i < REGISTER_REQUEST_MAX_COUNT ; ++i){
        if (EOF == m_pRegisterReadWriteOptions->GetRequestAddress(i)) continue;

        m_pRegisterReadWriteOptions->SetRequestValue(i, EOF);

        void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
        std::vector<DEVSELINFO *> deviceSelfInfo = m_pVideoDeviceModel->GetDeviceSelInfo();
        DEVSELINFO *selectedDevSelInfo = deviceSelfInfo.at(GetCurrentCameraIndex());
        int ret;
        unsigned short value;
        switch (m_pRegisterReadWriteOptions->GetType()){
            case RegisterReadWriteOptions::IC2:
            {
                ret = APC_GetSensorRegister( pEYSDI, selectedDevSelInfo,
                                                 m_pRegisterReadWriteOptions->GetSlaveID(),
                                                 m_pRegisterReadWriteOptions->GetRequestAddress(i),
                                                 &value,
                                                 m_pRegisterReadWriteOptions->GetAddressSize() | m_pRegisterReadWriteOptions->GetValueSize(),
                                                 m_pRegisterReadWriteOptions->GetSensorMode());
                break;
            }
            case RegisterReadWriteOptions::ASIC:
            {
                ret = APC_GetHWRegister( pEYSDI, selectedDevSelInfo,
                                             m_pRegisterReadWriteOptions->GetRequestAddress(i),
                                             &value,
                                             m_pRegisterReadWriteOptions->GetAddressSize() | m_pRegisterReadWriteOptions->GetValueSize());
                break;
            }
            case RegisterReadWriteOptions::FW:
            {
                ret = APC_GetFWRegister( pEYSDI, selectedDevSelInfo,
                                             m_pRegisterReadWriteOptions->GetRequestAddress(i),
                                             &value,
                                             m_pRegisterReadWriteOptions->GetAddressSize() | m_pRegisterReadWriteOptions->GetValueSize());
                break;
            }
            default:
                return APC_NotSupport;
        }

        if (APC_OK == ret)
        {
            if (m_pRegisterReadWriteOptions->GetValueSize() == FG_Value_2Byte && m_SwapBytesOrder) {
                int bHigh = (value & 0xff) << 8;
                int bLow = (value & 0xff00) >> 8;
                value = bHigh | bLow;
            }
            qDebug() << "ReadRegister: " << value;
            m_pRegisterReadWriteOptions->SetRequestValue(i, value);
        }else{
            m_pRegisterReadWriteOptions->SetRequestValue(i, 0xff);
        }
    }

    LogRegister();
    SetLastestReadTime(QTime::currentTime());

    return APC_OK;
}

int CRegisterReadWriteController::WriteRegister()
{
    for (int i = 0 ; i < REGISTER_REQUEST_MAX_COUNT ; ++i){
        if (EOF == m_pRegisterReadWriteOptions->GetRequestAddress(i)) continue;
        if (EOF == m_pRegisterReadWriteOptions->GetRequestValue(i)) continue;

        void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
        std::vector<DEVSELINFO *> deviceSelfInfo = m_pVideoDeviceModel->GetDeviceSelInfo();
        int ret;
        switch (m_pRegisterReadWriteOptions->GetType()){
            case RegisterReadWriteOptions::IC2:
            {
                ret = APC_SetSensorRegister( pEYSDI, deviceSelfInfo[0],
                                                 m_pRegisterReadWriteOptions->GetSlaveID(),
                                                 m_pRegisterReadWriteOptions->GetRequestAddress(i),
                                                 m_pRegisterReadWriteOptions->GetRequestValue(i),
                                                 m_pRegisterReadWriteOptions->GetAddressSize() | m_pRegisterReadWriteOptions->GetValueSize(),
                                                 m_pRegisterReadWriteOptions->GetSensorMode());
                break;
            }
            case RegisterReadWriteOptions::ASIC:
            {
                ret = APC_SetHWRegister( pEYSDI, deviceSelfInfo[0],
                                             m_pRegisterReadWriteOptions->GetRequestAddress(i),
                                             m_pRegisterReadWriteOptions->GetRequestValue(i),
                                             m_pRegisterReadWriteOptions->GetAddressSize() | m_pRegisterReadWriteOptions->GetValueSize());
                break;
            }
            case RegisterReadWriteOptions::FW:
            {
                ret = APC_SetFWRegister( pEYSDI, deviceSelfInfo[0],
                                             m_pRegisterReadWriteOptions->GetRequestAddress(i),
                                             m_pRegisterReadWriteOptions->GetRequestValue(i),
                                             m_pRegisterReadWriteOptions->GetAddressSize() | m_pRegisterReadWriteOptions->GetValueSize());
                break;
            }
            default:
                return APC_NotSupport;
        }
        if (ret != APC_OK)
        {
            m_pRegisterReadWriteOptions->SetRequestValue(i, 0xff);
        }
    }

    m_pVideoDeviceModel->Update();

    return APC_OK;
}
