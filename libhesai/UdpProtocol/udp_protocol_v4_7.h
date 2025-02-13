/*
This document is an encapsulation of the UDP4.7 protocol, 
defining protocol-related data structures and some operational functions.
*/

#ifndef HS_LIDAR_ST_V7_H
#define HS_LIDAR_ST_V7_H

#include <udp_protocol_header.h>
#include "plat_utils.h"

#define SEC_LEN  24
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

struct HS_LIDAR_BODY_AZIMUTH_ST_V7 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_ST_V7: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7 {
  uint8_t m_u8FineAzimuth;

  uint8_t GetFineAzimuth() const { return m_u8FineAzimuth; }

  void Print() const {
    printf("HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7: FineAzimuth:%u\n",
           GetFineAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_NNIT_ST_V7 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint8_t m_u8Confidence;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  uint8_t GetConfidenceLevel() const { return m_u8Confidence; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_NNIT_ST_V7:\n");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%d\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
} PACKED;


struct HS_LIDAR_BODY_CRC_ST_V7 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_BODY_CRC_ST_V7:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_TAIL_ST_V7 {
  static const uint8_t kShutdown = 0x01;
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;

  ReservedInfo1 m_reservedInfo1;
  uint8_t m_u8UReserved0[7];
  uint8_t m_u8FrameID;
  uint8_t m_u8UReserved1[7];
  int16_t m_i16MotorSpeed;
  uint32_t m_u32Timestamp;
  uint8_t m_u8ReturnMode;
  uint8_t m_u8FactoryInfo;
  uint8_t m_u8UTC[6];

  uint8_t GetStsID0() const { return m_reservedInfo1.GetID(); }
  uint16_t GetData0() const { return m_reservedInfo1.GetData(); }

  uint8_t GetFrameID() const { return m_u8FrameID; }

  int16_t GetMotorSpeed() const { return little_to_native(m_i16MotorSpeed); }

  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }

  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsDualReturn() const { return m_u8ReturnMode == kDualReturn; }

  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }

  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }
  uint64_t GetMicroLidarTimeU64() const {
    if (m_u8UTC[0] != 0) {
			struct tm t = {0};
			t.tm_year = m_u8UTC[0];
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
      return (mktime(&t) - timezone - 86400) * 1000000 + GetTimestamp() ;
		}
		else {
      uint32_t utc_time_big = *(uint32_t*)(&m_u8UTC[0] + 2);
      uint64_t unix_second = big_to_native(utc_time_big);
      return unix_second * 1000000 + GetTimestamp();
		}

  }
  void Print() const {
    printf("HS_LIDAR_TAIL_ST_V7:\n");
    printf(
        "sts0:%d, data0:%d, frameID:%d, motorSpeed:%d, "
        "timestamp:%u, return_mode:0x%02x, factoryInfo:0x%02x, utc:%u %u "
        "%u %u %u %u\n",
        GetStsID0(), GetData0(), GetFrameID(), GetMotorSpeed(), 
        GetTimestamp(), GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1), 
        GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }

} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_ST_V7 {
  uint32_t m_u32SeqNum;
  uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
  static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void Print() const {
    printf("HS_LIDAR_TAIL_SEQ_NUM_ST_V7:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_TAIL_CRC_ST_V7 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_TAIL_CRC_ST_V7:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITY_ST_V7 {
  uint8_t m_u8Signature[32];

  uint8_t GetSignatureData(uint8_t index) const {
    return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_CYBER_SECURITY_ST_V7:\n");
    for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
      printf("Signature%d:%d, ", i, GetSignatureData(i));
    printf("\n");
  }
} PACKED;

struct HS_LIDAR_E2E_HEADER_ST_V7 {
  uint16_t m_u16Length;
  uint16_t m_u16Counter;
  uint32_t m_u32DataID;
  uint32_t m_u32CRC;

  uint16_t GetLength() const { return big_to_native(m_u16Length); }
  uint16_t GetCounter() const { return big_to_native(m_u16Counter); }
  uint32_t GetDataID() const { return big_to_native(m_u32DataID); }
  uint32_t GetCRC() const { return big_to_native(m_u32CRC); }

  uint16_t GetE2ECrcSize() const {
    return GetLength() - sizeof(HS_LIDAR_CYBER_SECURITY_ST_V7) - sizeof(uint32_t);
  }

  void Print() const {
    printf("HS_LIDAR_E2E_HEADER_ST_V7:\n");
    printf("Length:%u, Counter:%u, DateID:%u, CRC:0x%08x\n",
        GetLength(), GetCounter(), GetDataID(), GetCRC());
  }
} PACKED;

struct HS_LIDAR_HEADER_ST_V7 {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kFunctionSafety = 0x04;
  static const uint8_t kCyberSecurity = 0x08;
  static const uint8_t kConfidenceLevel = 0x10;

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
  bool HasIMU() const { return m_u8Status & kIMU; }
  bool HasFuncSafety() const { return m_u8Status & kFunctionSafety; }
  bool HasCyberSecurity() const { return m_u8Status & kCyberSecurity; }
  bool HasConfidenceLevel() const { return m_u8Status & kConfidenceLevel; }

  uint16_t GetPacketSize() const {
    return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_ST_V7) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7) +
            sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7) +
            sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V7) * GetLaserNum()) *
               GetBlockNum() +
           sizeof(HS_LIDAR_BODY_CRC_ST_V7) + sizeof(HS_LIDAR_TAIL_ST_V7) +
           (HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ST_V7) : 0) +
           sizeof(HS_LIDAR_E2E_HEADER_ST_V7) +
           (HasCyberSecurity() ? sizeof(HS_LIDAR_CYBER_SECURITY_ST_V7) : 0);
  }

  uint16_t GetE2ECrcSize() const {
    return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_ST_V7) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7) +
            sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7) +
            sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V7) * GetLaserNum()) *
               GetBlockNum() +
           sizeof(HS_LIDAR_BODY_CRC_ST_V7) + sizeof(HS_LIDAR_TAIL_ST_V7) +
           (HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ST_V7) : 0) +
           sizeof(HS_LIDAR_E2E_HEADER_ST_V7) - sizeof(uint32_t);
  }

  void Print() const {
    printf("HS_LIDAR_HEADER_ST_V7:\n");
    printf(
        "laserNum:%02u, block_num:%02u, DistUnit:%g, EchoCnt:%02u, "
        "EchoNum:%02u, HasSeqNum:%d, HasIMU:%d, "
        "HasFuncSafety:%d, HasCyberSecurity:%d, HasConfidence:%d\n",
        GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
        GetEchoNum(), HasSeqNum(), HasIMU(), HasFuncSafety(),
        HasCyberSecurity(), HasConfidenceLevel());
  }
} PACKED;

struct FaultMessageVersion4_7 {
 public:
  uint8_t version_info;
  uint8_t utc_time[6];
  uint32_t time_stamp;
  uint8_t operate_state;
  uint8_t fault_state;
  uint8_t total_fault_code_num;
  uint8_t fault_code_id;
  uint32_t fault_code;
  uint8_t time_division_multiplexing[15];
  uint8_t fault_indicate[9];
  uint8_t software_version[3];
  uint8_t reversed[17];
  HS_LIDAR_E2E_HEADER_ST_V7 E2Eheader;
  uint8_t cycber_security[SEC_LEN];
  uint32_t GetTimestamp() const { return little_to_native(time_stamp); }
  uint64_t GetMicroLidarTimeU64() const {
    if (utc_time[0] != 0) {
			struct tm t = {0};
			t.tm_year = utc_time[0];
			if (t.tm_year >= 200) {
				t.tm_year -= 100;
			}
			t.tm_mon = utc_time[1] - 1;
			t.tm_mday = utc_time[2] + 1;
			t.tm_hour = utc_time[3];
			t.tm_min = utc_time[4];
			t.tm_sec = utc_time[5];
			t.tm_isdst = 0;
#ifdef _MSC_VER
  TIME_ZONE_INFORMATION tzi;
  GetTimeZoneInformation(&tzi);
  long int timezone =  tzi.Bias * 60;
#endif
      return (mktime(&t) - timezone - 86400) * 1000000 + GetTimestamp() ;
		}
		else {
      uint32_t utc_time_big = *(uint32_t*)(&utc_time[0] + 2);
      uint64_t unix_second = big_to_native(utc_time_big);
      return unix_second * 1000000 + GetTimestamp();
		}
  }
  void ParserFaultMessage(FaultMessageInfo &fault_message_info) {
    fault_message_info.fault_parse_version = 0x47;
    fault_message_info.version = version_info;
    memcpy(fault_message_info.utc_time, utc_time, sizeof(utc_time));
    fault_message_info.timestamp = GetTimestamp();
    fault_message_info.total_time = static_cast<double>(GetMicroLidarTimeU64()) / 1000000.0;
    fault_message_info.operate_state = operate_state;
    fault_message_info.fault_state = fault_state;
    fault_message_info.total_faultcode_num = total_fault_code_num;
    fault_message_info.faultcode_id = fault_code_id;
    fault_message_info.faultcode = fault_code;
    fault_message_info.union_info.fault4_7.tdm_data_indicate = time_division_multiplexing[0];
    memcpy(fault_message_info.union_info.fault4_7.time_division_multiplexing, time_division_multiplexing + 1, sizeof(time_division_multiplexing) - 1);
    fault_message_info.union_info.fault4_7.internal_fault_id = fault_indicate[0];
    memcpy(fault_message_info.union_info.fault4_7.fault_indicate, fault_indicate + 1, sizeof(fault_indicate) - 1);
    fault_message_info.union_info.fault4_7.customer_id = software_version[0];
    fault_message_info.union_info.fault4_7.software_version = software_version[1];
    fault_message_info.union_info.fault4_7.iteration_version = software_version[2];
    memcpy(fault_message_info.union_info.fault4_7.reversed, reversed, sizeof(reversed));
  }
} PACKED;
#ifdef _MSC_VER
#pragma pack(pop)
#endif
}  // namespace lidar
}  // namespace hesai
#endif