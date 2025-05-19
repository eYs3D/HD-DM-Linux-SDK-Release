#ifndef EP3META_JSONFILESERIALIZER_H
#define EP3META_JSONFILESERIALIZER_H

#include <iostream>
#include <fstream>
#include "../json/nlohmann/json.hpp"
#include <cstring>
#include <string>
#include <sys/stat.h>
/**
 * This class helps storing json into file.
 */
class JsonFileSerializer {
private:
    nlohmann::json mJson;
    static constexpr size_t kIndentSize = 4;
public:
    JsonFileSerializer() = default;

    static constexpr const char* kJsonFileSuffix = ".json";
    static constexpr const char* kCurrentFolder =  "./";
    static constexpr const char* kKeyAnalogGain = "analogGain";
    static constexpr const char* kKeyDigitalGain = "digitalGain";
    static constexpr const char* kKeyFrameCount = "frameCount";
    static constexpr const char* kKeyExposureTimeMs = "exposureTimeMs";
    static constexpr const char* kKeyLuminousMean = "YMean";
    static constexpr const char* kKeyOtherData = "otherData";

    template<typename T>
    void cacheIn(const char* key, T& value) {
        mJson[key] = value;
    }

    bool serializeToFile(const std::string& filename) {
        try {
            std::ofstream file(filename, std::ios::out | std::ios::trunc);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return false;
            }

            file << std::setw(kIndentSize) << mJson << std::endl;
            file.flush();
            file.close();
            mJson.clear();

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }

    inline static char separator() {
        return '/';
    }

    static bool hasSeparator(const std::string& folderName) {
        std::size_t pos = folderName.rfind(separator());
        return pos != std::string::npos;
    }

    static bool createFolder(const std::string& folderName) {
        if (mkdir(folderName.c_str(), 0755) == 0) {
            return true;
        } else {
            if (errno == EEXIST) {
                // Directory already exists
                return false;
            } else {
                // Handle other errors
                std::cerr << "Error creating folder "<< folderName << std::endl;
                return false;
            }
        }
    }

    static std::string CStringToStdString(const char* cstr, size_t length) {
        if (!cstr) {
            return std::string();
        }

        size_t len = std::strlen(cstr);
        if (length < len) {
            len = length;
        }

        return std::string(cstr, len);
    }

    static std::string makeStorePath(std::string&& path) {
        return !path.empty() ? JsonFileSerializer::hasSeparator(path) ?
            path : path + JsonFileSerializer::separator():
            JsonFileSerializer::kCurrentFolder;
    }
};

#endif //EP3META_JSONFILESERIALIZER_H
