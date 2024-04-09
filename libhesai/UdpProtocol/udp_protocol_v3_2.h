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
#ifndef HS_LIDAR_QT_V2_H
#define HS_LIDAR_QT_V2_H

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


struct HS_LIDAR_BODY_AZIMUTH_QT_V2 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_QT_V2: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_QT_V2 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint8_t m_u8Confidence;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  uint8_t GetConfidenceLevel() const { return m_u8Confidence; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_QT_V2:\n");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%u\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_QT_V2:\n");
    printf("Dist:%u, Reflectivity: %u, n", GetDistance(), GetReflectivity());
  }
} PACKED;

struct HS_LIDAR_BODY_CRC_QT_V2 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_BODY_CRC_QT_V2:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_FUNCTION_SAFETY {
  uint8_t m_u8FSVersion;
  uint8_t m_u8State;
  uint8_t m_u8Code;
  uint16_t m_u16OutCode;
  uint8_t m_u8Reserved[8];
  HS_LIDAR_BODY_CRC_QT_V2 m_crc;
} PACKED;

struct HS_LIDAR_TAIL_QT_V2 {
  // shutdown flag, bit 0
  static const uint8_t kShutdown = 0x01;

  // return mode
  static const uint8_t kFirstReturn = 0x33;
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kLastAndStrongestReturn = 0x39;
  static const uint8_t kFirstAndLastReturn = 0x3b;
  static const uint8_t kFirstAndStrongestReturn = 0x3c;
  static const uint8_t kStrongestAndSecondReturn = 0x3e;
  static const uint8_t kSecondReturn = 0x34;
  static const uint8_t kFirstAndSecondReturn = 0x3a;

  ReservedInfo1 m_reservedInfo1;
  uint16_t m_reservedInfo2;
  uint8_t m_u8ModeFlag;
  ReservedInfo3 m_reservedInfo3;
  uint16_t m_u16AzimuthFlag;
  uint8_t m_u8WorkingMode;
  uint8_t m_u8ReturnMode;
  uint16_t m_u16MotorSpeed;
  uint8_t m_u8UTC[6];
  uint32_t m_u32Timestamp;
  uint8_t m_u8FactoryInfo;

  uint8_t GetStsID1() const { return m_reservedInfo1.GetID(); }
  uint16_t GetData1() const { return m_reservedInfo1.GetData(); }
  uint8_t GetStsID3() const { return m_reservedInfo3.GetID(); }
  uint16_t GetData3() const { return m_reservedInfo3.GetData(); }
  uint8_t GetModeFlag() const { return m_u8ModeFlag; }
  uint16_t GetMotorSpeed() const { return little_to_native(m_u16MotorSpeed); }
  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }
  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsLastAndStrongestReturn() const {
    return m_u8ReturnMode == kLastAndStrongestReturn;
  }
  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }
  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }

  int64_t GetMicroLidarTimeU64() const {
    if (m_u8UTC[0] != 0) {
			struct tm t = {0};
			t.tm_year = m_u8UTC[0] + 100;
			if (t.tm_year >= 200) {
				t.tm_year -= 100;
			}
			t.tm_mon = m_u8UTC[1] - 1;
			t.tm_mday = m_u8UTC[2];
			t.tm_hour = m_u8UTC[3];
			t.tm_min = m_u8UTC[4];
			t.tm_sec = m_u8UTC[5];
			t.tm_isdst = 0;
#ifdef _MSC_VER
  TIME_ZONE_INFORMATION tzi;
  GetTimeZoneInformation(&tzi);
  long int timezone =  tzi.Bias * 60;
#endif
			return (mktime(&t) - timezone) * 1000000 + GetTimestamp();
		}
		else {
      uint32_t utc_time_big = *(uint32_t*)(&m_u8UTC[0] + 2);
      int64_t unix_second = ((utc_time_big >> 24) & 0xff) |
              ((utc_time_big >> 8) & 0xff00) |
              ((utc_time_big << 8) & 0xff0000) |
              ((utc_time_big << 24));
      return unix_second * 1000000 + GetTimestamp();
		}
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_QT_V2:\n");
    printf(
        "sts1:%d, data1:%d, motorSpeed:%u, timestamp:%u, return_mode:0x%02x, "
        "factoryInfo:0x%02x, utc:%u %u %u %u %u %u,\n",
        GetStsID1(), GetData1(), GetMotorSpeed(), GetTimestamp(),
        GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1),
        GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }

} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_QT_V2 {
  uint32_t m_u32SeqNum;

  uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
  static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void CalPktLoss(uint32_t &u32StartSeqNum, uint32_t &u32LastSeqNum, uint32_t &u32LossCount, 
        uint32_t &u32StartTime, uint32_t &u32TotalLossCount, uint32_t &u32TotalStartSeqNum) const {
    // bool print = false;
    if (u32StartSeqNum == 0) {
      u32LossCount = 0;
      u32TotalLossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
      u32LastSeqNum = m_u32SeqNum;
      u32TotalStartSeqNum = m_u32SeqNum;
      return;
    }
    if (m_u32SeqNum - u32LastSeqNum > 1) {
      u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
      u32TotalLossCount += (m_u32SeqNum - u32LastSeqNum - 1);
      // print = true;
      // if (m_u32SeqNum - u32LastSeqNum - 1 > 1000)
      // printf("%d,  %u, %u\n", m_u32SeqNum - u32LastSeqNum - 1, u32LastSeqNum,
      // m_u32SeqNum);
    }

    // print log every 1s
    if (u32LossCount != 0 && GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount,
             m_u32SeqNum - u32StartSeqNum); 
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
    }

    u32LastSeqNum = m_u32SeqNum;
  }

  // void CalPktLoss(uint32_t &u32StartSeqNum, uint32_t &u32LastSeqNum, uint32_t &u32LossCount, uint32_t &u32StartTime) const {
  //   // bool print = false;
  //   if (m_u32SeqNum - u32LastSeqNum > 1) {
  //     u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
  //     // print = true;
  //   }
  //   if (GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
  //     printf("pkt loss freq: %u/%u\n", u32LossCount,
  //            m_u32SeqNum - u32StartSeqNum);
  //     u32LossCount = 0;
  //     u32StartTime = GetMicroTickCount();
  //     u32StartSeqNum = m_u32SeqNum;
  //   }

  //   u32LastSeqNum = m_u32SeqNum;
  // }

  void CalPktLoss() const {
    static uint32_t u32StartSeqNum = m_u32SeqNum;
    static uint32_t u32LastSeqNum = m_u32SeqNum;
    static int32_t u32LossCount = 0;
    static uint32_t u32StartTime = GetMicroTickCount();
    // bool print = false;
    if (m_u32SeqNum - u32LastSeqNum > 1) {
      u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
      // print = true;
      // if (m_u32SeqNum - u32LastSeqNum - 1 > 1000)
      // printf("%d,  %u, %u\n", m_u32SeqNum - u32LastSeqNum - 1, u32LastSeqNum,
      // m_u32SeqNum);
    }

    // print log every 10s
    if (GetMicroTickCount() - u32StartTime >= 10 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount,
             m_u32SeqNum - u32StartSeqNum);
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
    }

    u32LastSeqNum = m_u32SeqNum;
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_SEQ_NUM_QT_V2:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_TAIL_CRC_QT_V2 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_TAIL_CRC_QT_V2:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITY_QT_V2 {
  uint8_t m_u8Signature[32];

  uint8_t GetSignatureData(uint8_t index) const {
    return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_CYBER_SECURITY_QT_V2:\n");
    for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
      printf("Signature%d:%d, ", i, GetSignatureData(i));
    printf("\n");
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITYQ_QT_V2 {
  uint8_t m_u8Signature[32];
} PACKED;

struct HS_LIDAR_HEADER_QT_V2 {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kFunctionSafety = 0x04;
  static const uint8_t kCyberSecurity = 0x08;
  static const uint8_t kConfidenceLevel = 0x10;
  static const uint8_t kSlope = 0x20;
  static const uint8_t kSelfDefine = 0x40;

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
  double GetDistUnit() const { return m_u8DistUnit / 1000.f; }
  uint8_t GetEchoCount() const { return m_u8EchoCount; }
  bool IsFirstBlockLastReturn() const {
    return m_u8EchoCount == kFirstBlockLastReturn;
  }
  bool IsFirstBlockStrongestReturn() const {
    return m_u8EchoCount == kFirstBlockStrongestReturn;
  }
  uint8_t GetEchoNum() const { return m_u8EchoNum; }
  bool HasSeqNum() const { return m_u8Status & kSequenceNum; }
  bool HasFunctionSafety() const { return m_u8Status & kFunctionSafety; }
  bool HasCyberSecurity() const { return m_u8Status & kCyberSecurity; }
  bool HasConfidenceLevel() const { return m_u8Status & kConfidenceLevel; }
  bool HasSlope() const { return m_u8Status & kSlope; }
  bool HasSelfDefine() const { return m_u8Status & kSelfDefine; }

  uint16_t GetPacketSize() const {
    return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_QT_V2) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
            (HasConfidenceLevel()
                 ? sizeof(HS_LIDAR_BODY_CHN_UNIT_QT_V2)
                 : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2)) *
                GetLaserNum()) *
               GetBlockNum() +
           sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
           (HasFunctionSafety() ? sizeof(HS_LIDAR_FUNCTION_SAFETY) : 0) +
           sizeof(HS_LIDAR_TAIL_QT_V2) +
           (HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_QT_V2) : 0) +
           sizeof(HS_LIDAR_TAIL_CRC_QT_V2) +
           (HasCyberSecurity() ? sizeof(HS_LIDAR_CYBER_SECURITY_QT_V2) : 0);
  }

  void Print() const {
    printf("HS_LIDAR_HEADER_QT_V2:\n");
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
