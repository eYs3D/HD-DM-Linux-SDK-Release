#ifndef EP3META_PROTOCOLZEROPARSER_H
#define EP3META_PROTOCOLZEROPARSER_H

#include <cstdio>
#include <cstdint>
#include <bitset>
#include <map>
#include "eSPDI.h"

class MetaDataPayloadParser {
public:
    typedef struct {
        uint8_t protocolVersion;
        uint16_t frameCount;
        uint8_t analogGain;    /* Range 1X ~ 16X */
        uint8_t digitalGain;   /* Range 1X ~ 3.938X */
        uint16_t exposureRows; /* Exposure rows x Row Time = Exposure Time */
        uint8_t yAverage;      /* Average YUYV's Ysum divided by Ysampling points */
        uint16_t otherData;
    } ProtoZeroData;

    static const double GetDigitalGainMappingValue(uint8_t key) {
        static const std::map<uint8_t, double> sDigitalGainMap = {
                {0, 1.000}, {10, 1.313}, {20, 1.625}, {30, 1.938}, {40, 2.500}, {50, 3.125}, {60, 3.750},
                {1, 1.031}, {11, 1.344}, {21, 1.656}, {31, 1.969}, {41, 2.563}, {51, 3.188}, {61, 3.813},
                {2, 1.063}, {12, 1.375}, {22, 1.688}, {32, 2.000}, {42, 2.625}, {52, 3.250}, {62, 3.875},
                {3, 1.094}, {13, 1.406}, {23, 1.719}, {33, 2.063}, {43, 2.688}, {53, 3.313}, {63, 3.938},
                {4, 1.125}, {14, 1.438}, {24, 1.750}, {34, 2.125}, {44, 2.750}, {54, 3.375},
                {5, 1.156}, {15, 1.469}, {25, 1.781}, {35, 2.188}, {45, 2.813}, {55, 3.438},
                {6, 1.188}, {16, 1.500}, {26, 1.813}, {36, 2.250}, {46, 2.875}, {56, 3.500},
                {7, 1.219}, {17, 1.531}, {27, 1.844}, {37, 2.313}, {47, 2.938}, {57, 3.563},
                {8, 1.250}, {18, 1.563}, {28, 1.875}, {38, 2.375}, {48, 3.000}, {58, 3.625},
                {9, 1.281}, {19, 1.594}, {29, 1.906}, {39, 2.438}, {49, 3.063}, {59, 3.688},
        };

        auto it = sDigitalGainMap.find(key);
        if (it != sDigitalGainMap.end()) {
            return it->second;
        }

        fprintf(stderr, "Error Packet DG\n");
        return 0;
    }

    static uint8_t GetAnalogGainMappingValue(uint8_t key) {
        static const std::map<uint8_t, uint8_t> sAnalogGainMap = {
                {1, 1}, {2, 2}, {3, 4}, {4, 8}, {5, 16}
        };

        auto it = sAnalogGainMap.find(key);
        if (it != sAnalogGainMap.end()) {
            return it->second;
        }

        fprintf(stderr, "Error Packet AG\n");
        return 0;
    }

    static int parseProtocolZeroData(APC_META_DATA *payload, ProtoZeroData* parsed) {
        if (!payload || !parsed) {
            fprintf(stderr, "Parameter null.\n");
            return APC_NullPtr;
        }

        if (!payload->payloadSize) {
            fprintf(stderr, "Received error meta data.\n");
            return APC_NullPtr;
        }

        uint32_t startData = __bswap_constant_32(*((uint32_t*) (&payload->payload[0])));
        uint32_t endData = __bswap_constant_32(*((uint32_t*) (&payload->payload[4])));

        parsed->protocolVersion = startData >> 29u;
        parsed->frameCount = __bswap_16((startData >> 13u) & 0xFFFFu);
        parsed->analogGain = (startData >> 10u) & 0x7u;
        parsed->digitalGain = (startData >> 4u) & 0x3Fu;
        parsed->exposureRows = ((startData & 0x0Fu) << 8u) | ((endData >> 24u) & 0xFFu);
        parsed->yAverage = ((endData >> 16u) & 0xFFu);
        parsed->otherData = (endData >> 1u) & 0x7FFFu;

        fprintf(stderr, "[eys3d_meta_hex] PV:%02x FC:%d AG:%02x DG:%02x EXP:%02x yAVG=%02x OT%04x Parity:%x\n",
                parsed->protocolVersion,
                parsed->frameCount,
                parsed->analogGain,
                parsed->digitalGain,
                parsed->exposureRows,
                parsed->yAverage,
                parsed->otherData,
                endData & 0x1u);

        return APC_OK;
    }

};


#endif //EP3META_PROTOCOLZEROPARSER_H
