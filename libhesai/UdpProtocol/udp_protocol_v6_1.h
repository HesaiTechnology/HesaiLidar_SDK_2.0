/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/

#ifndef HSLIDARXTV1_H
#define HSLIDARXTV1_H

#include <udp_protocol_header.h>
namespace hesai
{
namespace lidar
{
#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

struct HsLidarXTV1BodyAzimuth {
    uint16_t m_u16Azimuth;

    uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

    void Print() const {
        printf("HsLidarXTV1BodyAzimuth: azimuth:%u\n", GetAzimuth());
    }
} PACKED;

struct HsLidarXTV1BodyChannelData {
    uint16_t m_u16Distance;
    uint8_t m_u8Reflectivity;
    uint8_t m_u8Confidence;

    uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
    uint8_t GetReflectivity() const { return m_u8Reflectivity; }
    uint8_t GetConfidenceLevel() const { return m_u8Confidence; }

    void Print() const {
        printf("HsLidarXTV1BodyChannelData:\n");
        printf("Dist:%u, Reflectivity: %u, confidenceLevel:%d\n", GetDistance(),
               GetReflectivity(), GetConfidenceLevel());
    }
} PACKED;

struct HsLidarXTV1Tail {
    // shutdown flag, bit 0
    static const uint8_t kShutdown = 0x01;

    // return mode
    static const uint8_t kStrongestReturn = 0x37;
    static const uint8_t kLastReturn = 0x38;
    static const uint8_t kDualReturn = 0x39;

    ReservedInfo1 m_reservedInfo1;
    ReservedInfo2 m_reservedInfo2;
    ReservedInfo3 m_reservedInfo3;
    uint8_t m_u8Shutdown;
    uint8_t m_u8ReturnMode;
    uint16_t m_u16MotorSpeed;
    uint8_t m_u8UTC[6];
    uint32_t m_u32Timestamp;
    uint8_t m_u8FactoryInfo;
    uint32_t m_u32SeqNum;

    uint8_t GetStsID0() const { return m_reservedInfo1.GetID(); }
    uint16_t GetData0() const { return m_reservedInfo1.GetData(); }

    uint8_t GetStsID1() const { return m_reservedInfo2.GetID(); }
    uint16_t GetData1() const { return m_reservedInfo2.GetData(); }

    uint8_t HasShutdown() const { return m_u8Shutdown & kShutdown; }

    uint8_t GetStsID2() const { return m_reservedInfo3.GetID(); }
    uint16_t GetData2() const { return m_reservedInfo3.GetData(); }

    uint16_t GetMotorSpeed() const { return little_to_native(m_u16MotorSpeed); }

    uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }

    uint64_t GetMicroLidarTimeU64() const {
        if (m_u8UTC[0] != 0) {
            struct tm t = {0};
            t.tm_year = m_u8UTC[0] + 100;
            if (t.tm_year >= 200) {
                t.tm_year -= 100;
            }
            t.tm_mon = m_u8UTC[1] - 1;
            t.tm_mday = m_u8UTC[2] + 1;
            t.tm_hour = m_u8UTC[3];
            t.tm_min = m_u8UTC[4];
            t.tm_sec = m_u8UTC[5];
            t.tm_isdst = 0;
#ifdef _MSC_VER
  TIME_ZONE_INFORMATION tzi;
  GetTimeZoneInformation(&tzi);
  long int timezone =  tzi.Bias * 60;
#endif
            return (mktime(&t) - timezone - 86400) * 1000000 + GetTimestamp();
        }
        else {
            uint32_t utc_time_big = *(uint32_t*)(&m_u8UTC[0] + 2);
            uint64_t unix_second = big_to_native(utc_time_big);
            return unix_second * 1000000 + GetTimestamp();
        }
    }

    uint8_t GetReturnMode() const { return m_u8ReturnMode; }
    bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
    bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
    bool IsDualReturn() const { return m_u8ReturnMode == kDualReturn; }

    uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }

    uint8_t GetUTCData(uint8_t index) const {
        return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
    }

    uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }

    void Print() const {
        printf("HsLidarXTV1Tail:\n");
        printf(
                "sts0:%d, data0:%d, sts1:%d, data1:%d, sts2:%d, data2:%d, shutDown:%d, motorSpeed:%u, "
                "timestamp:%u, return_mode:0x%02x, factoryInfo:0x%02x, utc:%u %u "
                "%u %u %u %u, seqNum: %u\n",
                GetStsID0(), GetData0(), GetStsID1(), GetData1(), GetStsID2(), GetData2(), HasShutdown(),
                GetMotorSpeed(), GetTimestamp(), GetReturnMode(), GetFactoryInfo(),
                GetUTCData(0), GetUTCData(1), GetUTCData(2), GetUTCData(3),
                GetUTCData(4), GetUTCData(5), GetSeqNum());
    }

} PACKED;

struct HsLidarXTV1Header {
    static const uint8_t kSequenceNum = 0x01;
    static const uint8_t kDistUnit = 0x04;
    static const uint8_t kFirstBlockLastReturn = 0x01;
    static const uint8_t kFirstBlockStrongestReturn = 0x02;

    uint8_t m_u8LaserNum;
    uint8_t m_u8BlockNum;
    uint8_t m_u8EchoCount;
    uint8_t m_u8DistUnit;
    uint8_t m_u8EchoNum;
    uint8_t m_u8Status;

    uint8_t GetLaserNum() const { return m_u8LaserNum; }
    uint8_t GetBlockNum() const { return m_u8BlockNum; }
    float GetDistUnit() const { return m_u8DistUnit / 1000.f; }
    uint8_t GetEchoCount() const { return m_u8EchoCount; }
    bool IsFirstBlockLastReturn() const {
        return m_u8EchoCount == kFirstBlockLastReturn;
    }
    bool IsFirstBlockStrongestReturn() const {
        return m_u8EchoCount == kFirstBlockStrongestReturn;
    }
    uint8_t GetEchoNum() const { return m_u8EchoNum; }
    bool HasSeqNum() const { return m_u8Status & kSequenceNum; }

    uint16_t GetPacketSize() const {
        return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HsLidarXTV1Header) +
               sizeof(HsLidarXTV1Tail) +
                (sizeof(HsLidarXTV1BodyAzimuth) +
                 sizeof(HsLidarXTV1BodyChannelData) * GetLaserNum()) *
                GetBlockNum();
    }

    void Print() const {
        printf("HsLidarXTV1Header:\n");
        printf(
                "laserNum:%02u, block_num:%02u, DistUnit:%g, EchoCnt:%02u, "
                "EchoNum:%02u, HasSeqNum:%d\n",
                GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
                GetEchoNum(), HasSeqNum());
    }
} PACKED;
#ifdef _MSC_VER
#pragma pack(pop)
#endif
}  // namespace lidar
}  // namespace hesai
#endif
