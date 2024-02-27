#ifndef HS_LIDAR_ET_V5_H
#define HS_LIDAR_ET_V5_H

#include "plat_utils.h"
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

    // Body
    // body_unit
    struct HS_LIDAR_BODY_LASTER_UNIT_ET_V5 {
      uint16_t distance;
      uint8_t reflectivity;

      uint16_t GetDistance() const { return little_to_native(distance); }
      uint8_t GetReflectivity() const { return reflectivity; }
      void Print() const {
        printf("HS_LIDAR_BODY_CHN_UNIT_FT_V2:\n");
        printf("Distance:%u, Reflectivity: %u\n", GetDistance(), GetReflectivity() );
      }
    } PACKED;

    //Body
    // The first three fields of seq
    struct HS_LIDAR_BODY_SEQ3_ET_V5 {
      int16_t horizontalAngle;
      int16_t verticalAngle;
      uint8_t confidence;
      
      int16_t GetHorizontalAngle() const { return little_to_native(horizontalAngle); }
      int16_t GetVerticalAngle() const { return little_to_native(verticalAngle); }
      uint8_t GetConfidence() const { return confidence; }
    } PACKED;

    // Body
    // crc
    struct HS_LIDAR_BODY_CRC_ET_V5 {
      uint32_t m_u32Crc;

      uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

      void Print() const {
        printf("HS_LIDAR_BODY_CRC_ET_V5:\n");
        printf("crc:0x%08x\n", GetCrc());
      }
    } PACKED;

    // Tail
    // the first nine fields of tail
    struct HS_LIDAR_TAIL_ET_V5{
      
      static const uint8_t kShutDown = 0x01;
      static const uint8_t kFirstReturn = 0x33;
      static const uint8_t kStrongestReturn = 0x37;
      static const uint8_t kLastReturn = 0x38;
      static const uint8_t kDualReturn = 0x39;
      static const uint8_t kFactoryInfo = 0x42;
      ReservedInfo1 m_reservedInfo1;
      ReservedInfo2 m_reservedInfo2;
      ReservedInfo3 m_reservedInfo3;
      uint8_t m_u8FrameID;
      uint8_t m_u8ShutDown;
      uint8_t m_u8EchoMode;
      uint8_t m_u8UTC[6];
      uint32_t m_u32Timestamp;
      uint8_t m_u8FactoryInfo;
      
      
      uint8_t GetStsID0() const { return m_reservedInfo1.GetID(); }
      uint16_t GetData0() const { return m_reservedInfo1.GetData(); }

      uint8_t GetStsID1() const { return m_reservedInfo2.GetID(); }
      uint16_t GetData1() const { return m_reservedInfo2.GetData(); }

      uint8_t GetStsID2() const { return m_reservedInfo3.GetID(); }
      uint16_t GetData2() const { return m_reservedInfo3.GetData(); }

      uint8_t GetFrameID() const { return m_u8FrameID; }

      uint8_t HasShutdown() const { return m_u8ShutDown & kShutDown; }

      uint8_t GetReturnMode() const { return m_u8EchoMode; }

      bool IsLastReturn() const { return m_u8EchoMode == kLastReturn; }
      bool IsStrongestReturn() const { return m_u8EchoMode == kStrongestReturn; }
      bool IsDualReturn() const { return m_u8EchoMode == kDualReturn; }
      bool IsFirstReturn() const { return m_u8EchoMode == kFirstReturn; }

      uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }
      uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }

      uint8_t GetUTCData(uint8_t index) const {
        return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
      }
      
      // get us time
      int64_t GetMicroLidarTimeU64() const {
        if (m_u8UTC[0] != 0) {
          struct tm t = {0};
          t.tm_year = m_u8UTC[0];
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
        printf("HS_LIDAR_TAIL_ET_V5:\n");
        printf(
            "sts0:%d, data0:%d, sts1:%d, data1:%d, shutDown:%d"
            "timestamp:%u, return_mode:0x%02x, factoryInfo:0x%02x, utc:%u %u "
            "%u %u %u %u\n",
            GetStsID0(), GetData0(), GetStsID1(), GetData1(), HasShutdown(),
            GetTimestamp(), GetReturnMode(), GetFactoryInfo(),
            GetUTCData(0), GetUTCData(1), GetUTCData(2), GetUTCData(3),
            GetUTCData(4), GetUTCData(5));
      }
    } PACKED;

    // Tail
    // seqnum
    struct HS_LIDAR_TAIL_SEQ_NUM_ET_V5 {
      uint32_t m_u32SeqNum;
      uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
      static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }
      
      void CalPktLoss(uint32_t &u32StartSeqNum, uint32_t &u32LastSeqNum, uint32_t &u32LossCount, uint32_t &u32StartTime) const {
        // bool print = false;
        if (m_u32SeqNum - u32LastSeqNum > 1) {
          u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
          // print = true;
        }
      
       if (GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
          printf("pkt loss freq: %u/%u\n", u32LossCount, m_u32SeqNum - u32StartSeqNum);
          u32LossCount = 0;
          u32StartTime = GetMicroTickCount();
          u32StartSeqNum = m_u32SeqNum;
        }
        u32LastSeqNum = m_u32SeqNum;
      }
      
      void Print() const {
        printf("HS_LIDAR_TAIL_SEQ_NUM_ET_V5:\n");
        printf("seqNum: %u\n", GetSeqNum());
      }
    } PACKED;

    // Tail
    // crc
    struct HS_LIDAR_TAIL_CRC_ET_V5 {
      uint32_t m_u32Crc;
      uint32_t GetCrc() const { return little_to_native(m_u32Crc); }
      void Print() const {
        printf("HS_LIDAR_TAIL_CRC_ET_V5:\n");
        printf("crc:0x%08x\n", GetCrc());
      }
    } PACKED;

    // Cyber Security (optional)
    struct HS_LIDAR_CYBER_SECURITY_ET_V5 {
      uint8_t m_u8Signature[32];
      uint8_t GetSignatureData(uint8_t index) const {
        return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
      }
      void Print() const {
        printf("HS_LIDAR_CYBER_SECURITY_ET_V5:\n");
        for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
          printf("Signature%d:%d, ", i, GetSignatureData(i));
        printf("\n");
      }
    } PACKED;

    // Header
    struct HS_LIDAR_HEADER_ET_V5 {
      static const uint8_t kLaserNum = 0x40;
      static const uint8_t kBlockNum = 0x04;
      static const uint8_t kFirstBlockLastReturn = 0x01;
      static const uint8_t kFirstBlockStrongestReturn = 0x02;
      static const uint8_t kDistUnit = 0x05;
      static const uint8_t kSeqNum = 0x08;
      static const uint8_t kSequenceNum = 0x01;     // bit0
      static const uint8_t kIMU = 0x02;             // bit1
      static const uint8_t kFunctionSafety = 0x04;  // bit2
      static const uint8_t kCyberSecurity = 0x08;   // bit3
      static const uint8_t kConfidenceLevel = 0x10; // bit4
      static const uint8_t kWeightFactor = 0x20;    // bit5

      uint8_t m_u8LaserNum;     
      uint8_t m_u8BlockNum;     
      uint8_t m_u8EchoCount;    
      uint8_t m_u8DistUnit;     
      uint8_t m_u8EchoNum;      
      uint8_t m_u8SeqNum;       
      uint8_t m_u8Reserved2; 

     
      uint8_t GetLaserNum() const { return m_u8LaserNum; }
      uint8_t GetBlockNum() const { return m_u8BlockNum; }
      double GetDistUnit() const { return m_u8DistUnit / 1000.f; }
      uint8_t GetEchoCount() const { return m_u8EchoCount; }
      uint8_t GetSeqNum() const { return m_u8SeqNum; }

      bool IsFirstBlockLastReturn() const {
        return m_u8EchoCount == kFirstBlockLastReturn;
      }
      bool IsFirstBlockStrongestReturn() const {
        return m_u8EchoCount == kFirstBlockStrongestReturn;
      }
      uint8_t GetEchoNum() const { return m_u8EchoNum; }
      bool HasSeqNum() const { return m_u8Reserved2 & kSequenceNum; }
      bool HasIMU() const { return  m_u8Reserved2 & kIMU; }
      bool HasFuncSafety() const { return  m_u8Reserved2& kFunctionSafety; }
      bool HasCyberSecurity() const { return  m_u8Reserved2 & kCyberSecurity; }
      bool HasConfidenceLevel() const { return  m_u8Reserved2 & kConfidenceLevel; }
      bool HasWeightFactor() const { return  m_u8Reserved2 & kWeightFactor ; }

      uint16_t GetPacketSize() const {
        return (sizeof(struct HS_LIDAR_PRE_HEADER) + sizeof(struct HS_LIDAR_HEADER_ET_V5) + 
                (sizeof(uint16_t) + (sizeof(struct HS_LIDAR_BODY_LASTER_UNIT_ET_V5) * (m_u8LaserNum / m_u8SeqNum) + 5) * m_u8SeqNum) * m_u8BlockNum +
                sizeof(struct HS_LIDAR_BODY_CRC_ET_V5) + 
                sizeof(struct HS_LIDAR_TAIL_ET_V5) + sizeof(struct HS_LIDAR_TAIL_SEQ_NUM_ET_V5) + sizeof(struct HS_LIDAR_TAIL_CRC_ET_V5) + 
                sizeof(struct HS_LIDAR_CYBER_SECURITY_ET_V5)
               );
      }
      void Print() const {
        printf("HS_LIDAR_HEADER_ET_V5:\n");
        printf(
        "laserNum:%02u, block_num:%02u, DistUnit:%g, EchoCnt:%02u, "
        "EchoNum:%02u, HasSeqNum:%d, HasIMU:%d, "
        "HasFuncSafety:%d, HasCyberSecurity:%d, HasConfidence:%d\n",
        GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
        GetEchoNum(), HasSeqNum(), HasIMU(), HasFuncSafety(),
        HasCyberSecurity(), HasConfidenceLevel());
      }
    } PACKED;
  } // namespace lidar
} // namespace hesai

#endif 