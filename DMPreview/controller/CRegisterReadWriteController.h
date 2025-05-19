#ifndef CREGISTERREADWRITECONTROLLER_H
#define CREGISTERREADWRITECONTROLLER_H
#include <stdio.h>
#include <QTime>
#include "RegisterReadWriteOptions.h"
#include <QDebug>

class CVideoDeviceModel;
class CTaskInfo;
class CRegisterReadWriteController
{
public:
    CRegisterReadWriteController(CVideoDeviceModel *pVideoDeviceModel);
    ~CRegisterReadWriteController();

    CVideoDeviceModel *GetVideoDeviceModel(){ return m_pVideoDeviceModel; }
    RegisterReadWriteOptions *GetRegisterReadWriteOptions(){ return m_pRegisterReadWriteOptions; }

    FILE *GetLogFile(){ return m_pLogFile; }

    QTime GetLastestReadTime(){ return m_lastestReadTime; }
    void SetLastestReadTime(QTime time){ m_lastestReadTime = time; }

    int GetRealPeriodTimeMs(){ return m_nRealPeriodTimeMs; }
    void SetRealPeroidTimeMs(int nMs){ m_nRealPeriodTimeMs = nMs; }

    bool IsPeriodicReadRunning(){ return m_bPeriodicReadRunning; }
    void SetPeriodicReadRunning(bool bIsRunning){ m_bPeriodicReadRunning = bIsRunning; }

    int StartReadRegister();
    int StartPeriodicRead();
    int StopPeriodicRead();
    int LogRegister();

    int ReadRegister();
    int WriteRegister();

    bool GetSwapBytesOrderStatus() { return m_SwapBytesOrder; }
    void SetSwapBytesOrderStatus(bool swapBytesOrder) { m_SwapBytesOrder = swapBytesOrder; }

    std::map<std::string, int> GetCurrentDeviceMap();
    int  GetCurrentCameraIndex();
    void SetCurrentCameraIndex(int nIndex);

private:
    CVideoDeviceModel *m_pVideoDeviceModel;
    RegisterReadWriteOptions *m_pRegisterReadWriteOptions;
    CTaskInfo       *m_pRegisterTaskInfo;

    bool m_bPeriodicReadRunning = false;
    QTime m_lastestReadTime;
    int m_nRealPeriodTimeMs = 0;

    FILE *m_pLogFile = nullptr;
    char m_pLogFileName[256] = {0};
    bool m_SwapBytesOrder;
};

#endif // CREGISTERREADWRITECONTROLLER_H
