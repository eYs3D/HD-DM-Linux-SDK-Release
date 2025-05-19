#include "CVideoDeviceModel_IVY4.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_IVY4::CVideoDeviceModel_IVY4(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

std::vector<std::string> CVideoDeviceModel_IVY4::GetGainRegisterValueStringListByDeviceSelInfo(DEVSELINFO *devSelInfo) {
    if (!devSelInfo) {
        qDebug() << "GetGainRegisterValueStringListByDeviceSelInfo nullptr \n";
        return std::vector<std::string>();
    }

    SENSORMODE_INFO nSensorMode = SENSOR_BOTH;
    constexpr int kSensorRegisterFlag = FG_Address_2Byte | FG_Value_1Byte;
    constexpr int kSensorSlaveAddress = 0x6C;  // Black PCB layout would be 0x20

    std::vector<std::pair<unsigned short, unsigned short>> gainRegisters;
    std::vector<std::string> resultStdStringList;

    // Analog gain
    gainRegisters.push_back(std::make_pair(0x3e09, 0x00));
    // Digital gain
    gainRegisters.push_back(std::make_pair(0x3e06, 0x00));
    // Digital fine gain
    gainRegisters.push_back(std::make_pair(0x3e07, 0x00));

    for (auto it : gainRegisters) {
        int ret = APC_GetSensorRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(), devSelInfo,
                                        kSensorSlaveAddress, it.first, &it.second, kSensorRegisterFlag, nSensorMode);
        char buff[100];
        snprintf(buff, sizeof(buff), "Address 0x%x Value 0x%x", it.first, it.second);
        std::string buffAsStdStr(buff);
        resultStdStringList.push_back(std::move(buffAsStdStr));
    }

    return std::move(resultStdStringList);
}

std::vector<std::string> CVideoDeviceModel_IVY4::GetGainRegisterValueStringList() {
    return std::move(GetGainRegisterValueStringListByDeviceSelInfo(m_deviceSelInfo[0]));
}
