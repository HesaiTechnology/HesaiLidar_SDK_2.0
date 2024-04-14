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
#ifndef HS_LIDAR_FT_V2_H
#define HS_LIDAR_FT_V2_H

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

struct HS_LIDAR_BODY_CHN_UNIT_FT_V2 {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t background;
  uint8_t confidence;

  uint16_t GetDistance() const { return little_to_native(distance); }
  uint8_t GetReflectivity() const { return reflectivity; }
  uint8_t GetConfidenceLevel() const { return confidence; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_FT_V2:\n");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%d\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
} PACKED;

struct HS_LIDAR_TAIL_FT_V2 {
  // shutdown flag, bit 0
  static const uint8_t kShutdown = 0x01;

  // return mode
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;

  ReservedInfo1 m_reservedInfo1;
  ReservedInfo2 m_reservedInfo2;
  uint8_t status_mode;
  uint16_t column_id;
  uint8_t frame_flag;
  uint8_t high_temperature_shutdown_flag;
  uint8_t return_mode;
  uint16_t frame_duration;
  uint8_t utc[6];
  uint32_t timestamp;
  uint8_t factory;
  uint32_t sequence_num;

  uint8_t GetStsID0() const { return m_reservedInfo1.GetID(); }
  uint16_t GetData0() const { return m_reservedInfo1.GetData(); }

  uint8_t GetStsID1() const { return m_reservedInfo2.GetID(); }
  uint16_t GetData1() const { return m_reservedInfo2.GetData(); }


  uint16_t GetMotorSpeed() const { return little_to_native(frame_duration); }

  uint32_t GetTimestamp() const { return little_to_native(timestamp); }

  uint8_t GetReturnMode() const { return return_mode; }

  uint8_t GetUTCData(uint8_t index) const {
    return utc[index < sizeof(utc) ? index : 0];
  }

  uint32_t GetSeqNum() const { return little_to_native(sequence_num); }

  void CalPktLoss(uint32_t &u32StartSeqNum, uint32_t &u32LastSeqNum, uint32_t &u32LossCount, 
        uint32_t &u32StartTime, uint32_t &u32TotalLossCount, uint32_t &u32TotalStartSeqNum) const {
    // bool print = false;
    if (u32StartSeqNum == 0) {
      u32LossCount = 0;
      u32TotalLossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = sequence_num;
      u32LastSeqNum = sequence_num;
      u32TotalStartSeqNum = sequence_num;
      return;
    }
    if (sequence_num - u32LastSeqNum > 1) {
      u32LossCount += (sequence_num - u32LastSeqNum - 1);
      u32TotalLossCount += (sequence_num - u32LastSeqNum - 1);
      // print = true;
      // if (sequence_num - u32LastSeqNum - 1 > 1000)
      // printf("%d,  %u, %u\n", sequence_num - u32LastSeqNum - 1, u32LastSeqNum,
      // sequence_num);
    }

    // print log every 1s
    if (u32LossCount != 0 && GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount,
             sequence_num - u32StartSeqNum);
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = sequence_num;
    }

    u32LastSeqNum = sequence_num;
  }

  // void CalPktLoss(uint32_t &u32StartSeqNum, uint32_t &u32LastSeqNum, uint32_t &u32LossCount, uint32_t &u32StartTime) const {
  //   // bool print = false;
  //   if (sequence_num - u32LastSeqNum > 1) {
  //     u32LossCount += (sequence_num - u32LastSeqNum - 1);
  //     // print = true;
  //     // if (sequence_num - u32LastSeqNum - 1 > 1000)
  //     // printf("%d,  %u, %u\n", sequence_num - u32LastSeqNum - 1, u32LastSeqNum,
  //     // sequence_num);
  //   }

  //   // print log every 1s
  //   if (GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
  //     printf("pkt loss freq: %u/%u\n", u32LossCount,
  //            sequence_num - u32StartSeqNum);
  //     u32LossCount = 0;
  //     u32StartTime = GetMicroTickCount();
  //     u32StartSeqNum = sequence_num;
  //   }

  //   u32LastSeqNum = sequence_num;
  // }
  static uint32_t GetSeqNumSize() { return sizeof(sequence_num); }


  void CalPktLoss() const {
    static uint32_t u32StartSeqNum = GetSeqNum();
    static uint32_t u32LastSeqNum = GetSeqNum();
    static uint32_t u32LossCount = 0;
    static uint32_t u32StartTime = GetMicroTickCount();

    if (GetSeqNum() - u32LastSeqNum - 1 > 0) {
      u32LossCount += (GetSeqNum() - u32LastSeqNum - 1);
    }

    // print log every 10s
    if (GetMicroTickCount() - u32StartTime >= 10 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount, 
          GetSeqNum() - u32StartSeqNum);
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = GetSeqNum();
    }

    u32LastSeqNum = GetSeqNum();
  }
  int64_t GetMicroLidarTimeU64() const {
    if (utc[0] != 0) {
			struct tm t = {0};
			t.tm_year = utc[0] + 100;
			if (t.tm_year >= 200) {
				t.tm_year -= 100;
			}
			t.tm_mon = utc[1] - 1;
			t.tm_mday = utc[2];
			t.tm_hour = utc[3];
			t.tm_min = utc[4];
			t.tm_sec = utc[5];
			t.tm_isdst = 0;
#ifdef _MSC_VER
  TIME_ZONE_INFORMATION tzi;
  GetTimeZoneInformation(&tzi);
  long int timezone =  tzi.Bias * 60;
#endif
			return (mktime(&t) - timezone) * 1000000 + GetTimestamp();
		}
		else {
      uint32_t utc_time_big = *(uint32_t*)(&utc[0] + 2);
      int64_t unix_second = ((utc_time_big >> 24) & 0xff) |
              ((utc_time_big >> 8) & 0xff00) |
              ((utc_time_big << 8) & 0xff0000) |
              ((utc_time_big << 24));
      return unix_second * 1000000 + GetTimestamp();
		}

  }

  void Print() const {
    printf("HS_LIDAR_TAIL_FT_V2:\n");
    printf(
        "sts0:%d, data0:%d, sts1:%d, data1:%d, motorSpeed:%u, "
        "timestamp:%u, return_mode:0x%02x, utc:%u %u "
        "%u %u %u %u, seqNum: %u\n",
        GetStsID0(), GetData0(), GetStsID1(), GetData1(),
        GetMotorSpeed(), GetTimestamp(), GetReturnMode(),
        GetUTCData(0), GetUTCData(1), GetUTCData(2), GetUTCData(3),
        GetUTCData(4), GetUTCData(5), GetSeqNum());
  }

} PACKED;

struct HS_LIDAR_HEADER_FT_V2 {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kDistUnit = 0x04;
  static const uint8_t kFirstBlockLastReturn = 0x01;
  static const uint8_t kFirstBlockStrongestReturn = 0x02;
  uint16_t total_column;
  uint16_t total_row;
  uint8_t column_resolution;
  uint8_t row_resolution;
  uint8_t echo;
  uint8_t dist_unit;
  uint8_t block_index;
  uint16_t channel_num;
  uint8_t reserved[8];

  uint16_t GetChannelNum() const { return channel_num; }
  double GetDistUnit() const { return dist_unit / 1000.f; }
  uint8_t GetEchoCount() const { return echo; }
  bool IsFirstBlockLastReturn() const {
    return echo == kFirstBlockLastReturn;
  }
  bool IsFirstBlockStrongestReturn() const {
    return echo == kFirstBlockStrongestReturn;
  }

  // uint16_t GetPacketSize() const {
  //   uint16_t u16TailDelta =
  //       (HasSeqNum() ? 0 : HS_LIDAR_TAIL_FT_V2::GetSeqNumSize()) +
  //       (HasIMU() ? 0 : HS_LIDAR_TAIL_FT_V2::GetIMUSize());

  //   return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_FT_V2) +
  //          sizeof(HS_LIDAR_TAIL_FT_V2) +
  //          (sizeof(HS_LIDAR_BODY_AZIMUTH_FT_V2) +
  //           sizeof(HS_LIDAR_BODY_CHN_UNIT_FT_V2) * GetColumnNum()) *
  //              GetBlockNum() -
  //          u16TailDelta;
  // }

  // void Print() const {
  //   printf("HS_LIDAR_HEADER_FT_V2:\n");
  //   printf(
  //       "laserNum:%02u, block_num:%02u, DistUnit:%g, EchoCnt:%02u, "
  //       "EchoNum:%02u, HasSeqNum:%d, HasIMU:%d\n",
  //       GetColumnNum(), GetRowNum(), GetDistUnit(), GetEchoCount(),
  //       GetEchoNum(), HasSeqNum(), HasIMU());
  // }
} PACKED;
#ifdef _MSC_VER
#pragma pack(pop)
#endif
}  // namespace lidar
}  // namespace hesai
#endif
