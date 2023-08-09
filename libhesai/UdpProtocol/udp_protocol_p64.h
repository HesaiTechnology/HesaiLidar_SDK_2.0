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

#ifndef HSLIDARP64_H
#define HSLIDARP64_H
#include <udp_protocol_header.h>
#include <cstdint>
#include <cstdio>
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

struct HS_LIDAR_L64_Header {
  // 0xFFEE 2bytes
  uint16_t m_u16Sob;
  uint8_t m_u8LaserNum;
  uint8_t m_u8BlockNum;
  // return mode 1 byte  when dual return 0-Single Return ,
  // 1-The first block is the 1 st return. 2-The first block
  // is the 2 nd return
  uint8_t m_u8ReturnType;
  uint8_t m_u8DistUnit;
  uint8_t m_u8Reserved0;
  uint8_t m_u8Reserved1;

  HS_LIDAR_L64_Header()
      : m_u16Sob(0),
        m_u8LaserNum(0),
        m_u8BlockNum(0),
        m_u8ReturnType(0),
        m_u8DistUnit(0),
        m_u8Reserved0(0),
        m_u8Reserved1(0) {}

  uint8_t GetLaserNum() const { return m_u8LaserNum; }
  uint8_t GetBlockNum() const { return m_u8BlockNum; }
  double GetDistUnit() const { return m_u8DistUnit / 1000.f; }
  uint8_t GetReturnType() const { return m_u8ReturnType; }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_L64 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
} PACKED;

struct HS_LIDAR_BODY_AZIMUTH_L64 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_L64: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_L64 {
  uint32_t m_u32SeqNum;

  uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
  static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void Print() const {
    printf("HS_LIDAR_TAIL_SEQ_NUM_L64:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_TAIL_L64 {
  ReservedInfo1 m_reservedInfo1;
  uint8_t m_u8Reserved0[2];
  uint8_t m_u8Shutdown;
  uint16_t m_u16ErrorCode;
  uint16_t m_u16MotorSpeed;
  uint32_t m_u32Timestamp;
  uint8_t m_u8ReturnMode;
  uint8_t m_u8FactoryInfo;
  uint8_t m_u8UTC[6];

  uint16_t GetMotorSpeed() const { return little_to_native(m_u16MotorSpeed); }
  uint8_t GetStatusID() const { return m_reservedInfo1.GetID(); }
  uint16_t GetStatusData() const { return m_reservedInfo1.GetData(); }
  uint16_t GetErrorCode() const { return little_to_native(m_u16ErrorCode); }

  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }

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

} PACKED;
#ifdef _MSC_VER
#pragma pack(pop)
#endif
}  // namespace lidar
}  // namespace hesai
#endif  // INTENSITYRETEST_HSLIDARP64_H
