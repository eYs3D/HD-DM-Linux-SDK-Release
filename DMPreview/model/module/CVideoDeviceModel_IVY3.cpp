#include "CVideoDeviceModel_IVY3.h"
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"

CVideoDeviceModel_IVY3::CVideoDeviceModel_IVY3(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

std::vector<std::string> CVideoDeviceModel_IVY3::GetGainRegisterValueStringListByDeviceSelInfo(DEVSELINFO *devSelInfo) {
    if (!devSelInfo) {
        qDebug() << "GetGainRegisterValueStringListByDeviceSelInfo nullptr \n";
        return std::vector<std::string>();
    }

    SENSORMODE_INFO nSensorMode = SENSOR_BOTH;
    constexpr int kSensorRegisterFlag = FG_Address_2Byte | FG_Value_1Byte;
    constexpr int kSensorSlaveAddress = 0x20;

    std::vector<std::pair<unsigned short, unsigned short>> gainRegisters;
    std::vector<std::string> resultStdStringList;

    gainRegisters.push_back(std::make_pair(0x044D, 0x0));
    gainRegisters.push_back(std::make_pair(0x0450, 0x0));
    gainRegisters.push_back(std::make_pair(0x0451, 0x0));
    gainRegisters.push_back(std::make_pair(0x0452, 0x0));
    gainRegisters.push_back(std::make_pair(0x0453, 0x0));
    gainRegisters.push_back(std::make_pair(0x0454, 0x0));
    gainRegisters.push_back(std::make_pair(0x0455, 0x0));
    gainRegisters.push_back(std::make_pair(0x0456, 0x0));
    gainRegisters.push_back(std::make_pair(0x0457, 0x0));

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

std::vector<std::string> CVideoDeviceModel_IVY3::GetGainRegisterValueStringList() {
    return std::move(GetGainRegisterValueStringListByDeviceSelInfo(m_deviceSelInfo[0]));
}
