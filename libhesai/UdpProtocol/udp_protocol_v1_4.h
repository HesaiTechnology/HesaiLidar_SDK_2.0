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
#ifndef HS_LIDAR_ME_V4_H
#define HS_LIDAR_ME_V4_H

#include <udp_protocol_header.h>
#include <ctime>
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

struct HS_LIDAR_BODY_AZIMUTH_ME_V4 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_ME_V4: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4:\n");
    printf("Dist:%u, Reflectivity: %u\n", GetDistance(), GetReflectivity());
  }
  void PrintMixData() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4:\n");
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_ME_V4 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint8_t m_u8Confidence;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  uint8_t GetConfidenceLevel() const { return m_u8Confidence; }


  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_ME_V4:\n");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%d\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
  void PrintMixData() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_ME_V4:\n");
  }
} PACKED;

struct HS_LIDAR_BODY_CRC_ME_V4 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_BODY_CRC_ME_V4:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITY_ME_V4 {
  uint8_t m_u8Signature[32];

  uint8_t GetSignatureData(uint8_t index) const {
    return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_CYBER_SECURITY_ME_V4:\n");
    for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
      printf("Signature%d:%d, ", i, GetSignatureData(i));
    printf("\n");
  }
} PACKED;

struct HS_LIDAR_FUNC_SAFETY_ME_V4 {
  static const uint8_t kLidarStateShiftBits = 5;
  static const uint8_t kLidarStateMask = 0x07;
  static const uint8_t kRollingCntShiftBits = 0;
  static const uint8_t kRollingCntMask = 0x07;
  static const uint8_t kFaultTypeCurrent = 0x08;
  static const uint8_t kFaultTypeHistory = 0x10;
  static const uint8_t kFaultNumShiftBits = 4;
  static const uint8_t kFaultNumMask = 0x0F;
  static const uint8_t kFaultIDShiftBits = 0;
  static const uint8_t kFaultIDMask = 0x0F;

  uint8_t m_u8Version;
  uint8_t m_u8Status;
  uint8_t m_u8FaultInfo;
  uint16_t m_u16FaultCode;
  uint8_t m_u8Reserved[8];
  uint32_t m_u32Crc;

  uint8_t GetVersion() const { return m_u8Version; }

  uint8_t GetLidarState() const {
    return (m_u8Status >> kLidarStateShiftBits) & kLidarStateMask;
  }
  bool IsHistoryFault() const { return m_u8Status & kFaultTypeHistory; }
  bool IsCurrentFault() const { return m_u8Status & kFaultTypeCurrent; }

  uint8_t GetRollingCnt() const {
    return (m_u8Status >> kRollingCntShiftBits) & kRollingCntMask;
  }

  uint8_t GetFaultNum() const {
    return (m_u8FaultInfo >> kFaultNumShiftBits) & kFaultNumMask;
  }
  uint8_t GetFaultID() const {
    return (m_u8FaultInfo >> kFaultIDShiftBits) & kFaultIDMask;
  }

  uint16_t GetFaultCode() const { return little_to_native(m_u16FaultCode); }

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_FUNC_SAFETY_ME_V4:\n");
    printf("ver:%u, lidarState:%u, rollingCnt:%u, isCurrFault:%u, "
           "faultNum:%u, faultID:%u, faultCode:0x%04x, crc:0x%08x\n",
           GetVersion(), GetLidarState(), GetRollingCnt(), IsCurrentFault(),
           GetFaultNum(), GetFaultID(), GetFaultCode(), GetCrc());
  }
} PACKED;

struct HS_LIDAR_TAIL_ME_V4 {
  // shutdown flag, bit 0
  static const uint8_t kHighPerformanceMode = 0x00;
  static const uint8_t kShutdown = 0x01;
  static const uint8_t kStandardMode = 0x02;
  static const uint8_t kEnergySavingMode = 0x03;

  // return mode
  static const uint8_t kFirstReturn = 0x33;
  static const uint8_t kSecondReturn = 0x34;
  static const uint8_t kThirdReturn = 0x35;
  static const uint8_t kFourthReturn = 0x36;
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;
  static const uint8_t kFirstSecondReturn = 0x3a;
  static const uint8_t kStongestFirstReturn = 0x3b;

  ReservedInfo1 m_reservedInfo1;
  ReservedInfo2 m_reservedInfo2;
  ReservedInfo3 m_reservedInfo3;
  uint16_t m_u16AzimuthFlag;
  uint8_t m_u8RunningMode;
  uint8_t m_u8ReturnMode;
  uint16_t m_u16MotorSpeed;
  uint8_t m_u8UTC[6];
  uint32_t m_u32Timestamp;
  uint8_t m_u8FactoryInfo;

  uint8_t GetStsID0() const { return m_reservedInfo1.GetID(); }
  uint16_t GetData0() const { return m_reservedInfo1.GetData(); }
  uint8_t GetStsID1() const { return m_reservedInfo2.GetID(); }
  uint16_t GetData1() const { return m_reservedInfo2.GetData(); }
  uint8_t GetStsID2() const { return m_reservedInfo3.GetID(); }
  uint16_t GetData2() const { return m_reservedInfo3.GetData(); }

  uint8_t HasShutdown() const { return m_u8RunningMode == kShutdown; }
  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  uint16_t GetMotorSpeed() const { return little_to_native(m_u16MotorSpeed); }
  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }
  uint8_t getOperationMode() const { return m_u8RunningMode & 0x0f; }
  uint8_t getAngleState(int blockIndex) const {
        return (m_u16AzimuthFlag >> (2 * (7 - blockIndex))) & 0x03;
  }

  bool IsFirstReturn() const { return m_u8ReturnMode == kFirstReturn; }
  bool IsSecondReturn() const { return m_u8ReturnMode == kSecondReturn; }
  bool IsThirdReturn() const { return m_u8ReturnMode == kThirdReturn; }
  bool IsFourthReturn() const { return m_u8ReturnMode == kFourthReturn; }
  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsDualReturn() const { return m_u8ReturnMode == kDualReturn; }
  bool IsFirstSecondReturn() const {
    return m_u8ReturnMode == kFirstSecondReturn;
  }
  bool IsStongestFirstReturn() const {
    return m_u8ReturnMode == kStongestFirstReturn;
  }
  int64_t GetMicroLidarTimeU64() const {
    if (m_u8UTC[0] != 0) {
			struct tm t = {0};
			t.tm_year = m_u8UTC[0];
			if (t.tm_year < 70) {
				return 0;
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
			return (mktime(&t) - timezone - 86400) * 1000000 + GetTimestamp() ;
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

  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }
  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_ME_V4:\n");
    printf("sts0:%d, data0:%d, sts1:%d, data1:%d, sts2:%d, data2:%d, "
           "shutDown:%d, motorSpeed:%u, timestamp:%u, return_mode:0x%02x, "
           "factoryInfo:0x%02x, utc:%u %u %u %u %u %u\n",
           GetStsID0(), GetData0(), GetStsID1(), GetData1(), GetStsID2(),
           GetData2(), HasShutdown(), GetMotorSpeed(), GetTimestamp(),
           GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1),
           GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }
} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_ME_V4 {
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
  //     // if (m_u32SeqNum - u32LastSeqNum - 1 > 1000)
  //     // printf("%d,  %u, %u\n", m_u32SeqNum - u32LastSeqNum - 1, u32LastSeqNum,
  //     // m_u32SeqNum);
  //   }

  //   // print log every 1s
  //   if (GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
  //     printf("pkt loss freq: %u/%u\n", u32LossCount,
  //            m_u32SeqNum - u32StartSeqNum);
  //     u32LossCount = 0;
  //     u32StartTime = GetMicroTickCount();
  //     u32StartSeqNum = m_u32SeqNum;
  //   }

  //   u32LastSeqNum = m_u32SeqNum;
  // }

  void Print() const { 
    printf("HS_LIDAR_TAIL_SEQ_NUM_ME_V4:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_TAIL_IMU_ME_V4 {
  int16_t m_i16IMUTemperature;
  uint16_t m_u16IMUAccelUnit;
  uint16_t m_u16IMUAngVelUnit;
  uint32_t m_u32IMUTimeStamp;
  int16_t m_i16IMUXAccel;
  int16_t m_i16IMUYAccel;
  int16_t m_i16IMUZAccel;
  int16_t m_i16IMUXAngVel;
  int16_t m_i16IMUYAngVel;
  int16_t m_i16IMUZAngVel;

  int16_t GetIMUTemperature() const {
    return little_to_native(m_i16IMUTemperature);
  }
  double GetIMUAccelUnit() const {
    return little_to_native(m_u16IMUAccelUnit) / 1000.f;
  }
  double GetIMUAngVelUnit() const {
    return little_to_native(m_u16IMUAngVelUnit) / 1000.f;
  }
  uint32_t GetIMUTimestamp() const {
    return little_to_native(m_u32IMUTimeStamp);
  }
  double GetIMUXAccel() const {
    return little_to_native(m_i16IMUXAccel) * GetIMUAccelUnit();
  }
  double GetIMUYAccel() const {
    return little_to_native(m_i16IMUYAccel) * GetIMUAccelUnit();
  }
  double GetIMUZAccel() const {
    return little_to_native(m_i16IMUZAccel) * GetIMUAccelUnit();
  }
  double GetIMUXAngVel() const {
    return little_to_native(m_i16IMUXAngVel) * GetIMUAngVelUnit();
  }
  double GetIMUYAngVel() const {
    return little_to_native(m_i16IMUYAngVel) * GetIMUAngVelUnit();
  }
  double GetIMUZAngVel() const {
    return little_to_native(m_i16IMUZAngVel) * GetIMUAngVelUnit();
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_IMU_ME_V4:\n");
    printf("Temp:%u, AccelUnit:%f, AngVelUnit:%f, TimeStamp:%u, XAccel:%f, "
           "YAccel:%f, ZAccel:%f, XAngVel:%f, YAngVel:%f, ZAngVel:%f\n",
           GetIMUTemperature(), GetIMUAccelUnit(), GetIMUAngVelUnit(),
           GetIMUTimestamp(), GetIMUXAccel(), GetIMUYAccel(), GetIMUZAccel(),
           GetIMUXAngVel(), GetIMUYAngVel(), GetIMUZAngVel());
  }
} PACKED;

struct HS_LIDAR_TAIL_CRC_ME_V4 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_TAIL_CRC_ME_V4:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_HEADER_ME_V4 {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kFunctionSafety = 0x04;
  static const uint8_t kCyberSecurity = 0x08;
  static const uint8_t kConfidenceLevel = 0x10;
  static const uint8_t kDistUnit = 0x04;

  static const uint8_t kSingleReturn = 0x00;
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
  uint8_t GetEchoNum() const { return m_u8EchoNum; }
  bool HasSeqNum() const { return m_u8Status & kSequenceNum; }
  bool HasIMU() const { return m_u8Status & kIMU; }
  bool HasFuncSafety() const { return m_u8Status & kFunctionSafety; }
  bool HasCyberSecurity() const { return m_u8Status & kCyberSecurity; }
  bool HasConfidenceLevel() const { return m_u8Status & kConfidenceLevel; }
  bool IsFirstBlockLastReturn() const {
    return m_u8EchoCount == kFirstBlockLastReturn;
  }
  bool IsFirstBlockStrongestReturn() const {
    return m_u8EchoCount == kFirstBlockStrongestReturn;
  }

  uint16_t GetPacketSize() const {
    return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_ME_V4) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
            (HasConfidenceLevel()
                 ? sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4)
                 : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4)) *
                GetLaserNum()) *
               GetBlockNum() +
           sizeof(HS_LIDAR_BODY_CRC_ME_V4) +
           (HasFuncSafety() ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0) +
           (HasCyberSecurity() ? sizeof(HS_LIDAR_CYBER_SECURITY_ME_V4) : 0) +
           sizeof(HS_LIDAR_TAIL_ME_V4) + sizeof(HS_LIDAR_TAIL_CRC_ME_V4) +
           (HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ME_V4) : 0) +
           (HasIMU() ? sizeof(HS_LIDAR_TAIL_IMU_ME_V4) : 0);
  }

  void Print() const {
    printf("HS_LIDAR_HEADER_ME_V4:\n");
    printf("laserNum:%02u, block_num:%02u, DistUnit:%g, EchoCnt:%02u, "
           "EchoNum:%02u, HasSeqNum:%d, HasIMU:%d, "
           "HasFuncSafety:%d, HasCyberSecurity:%d, HasConfidence:%d\n",
           GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
           GetEchoNum(), HasSeqNum(), HasIMU(), HasFuncSafety(),
           HasCyberSecurity(), HasConfidenceLevel());
  }
} PACKED;
#ifdef _MSC_VER
#pragma pack(pop)
#endif
}  // namespace lidar
}  // namespace hesai
#endif
