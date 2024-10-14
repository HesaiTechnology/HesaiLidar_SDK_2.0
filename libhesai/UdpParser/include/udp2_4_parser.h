/*
  This file encapsulates the data types for ET radar correction files and defines a class for a parser(UDP2.4 protocol).
*/
#ifndef UDP2_4_PARSER_H
#define UDP2_4_PARSER_H

#include "general_parser.h"
#include "lidar_types.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
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
    static constexpr int ET_MAX_CHANNEL_NUM_24 = 512;
    struct ETCorrections_V4 {
      uint8_t delimiter[2];
      uint8_t major_version;
      uint8_t min_version;
      uint8_t reserved1;
      uint8_t reserved2;
      uint8_t channel_number;
      uint8_t mirror_number_reserved3;
      uint16_t angle_division;
      int16_t apha;
      int16_t beta;
      int16_t gamma;
      float azimuths[ET_MAX_CHANNEL_NUM_24];
      float elevations[ET_MAX_CHANNEL_NUM_24];
      int16_t raw_azimuths[ET_MAX_CHANNEL_NUM_24];
      int16_t raw_elevations[ET_MAX_CHANNEL_NUM_24];
      uint8_t SHA_value[32];
    };

    struct ETCorrections_v4_Header {
    uint8_t delimiter[2];
    uint8_t major_version;
    uint8_t min_version;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t channel_number;
    uint8_t mirror_number_reserved3;
    uint16_t angle_division;
    int16_t apha;
    int16_t beta;
    int16_t gamma;
  } PACKED;
#ifdef _MSC_VER
#pragma pack(pop)
#endif
  // class Udp2_4Parser
  // parsers packets and computes points for ET25
  // you can parser the upd or pcap packets using the DocodePacket fuction
  // you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
  template<typename T_Point>
  class Udp2_4Parser : public GeneralParser<T_Point> {
    public:
      Udp2_4Parser();
      virtual ~Udp2_4Parser();
      int16_t GetVecticalAngle(int channel); 
      // determine whether frame splitting is needed
      bool IsNeedFrameSplit(uint16_t nowid);
      // get lidar correction file from local file,and pass to udp parser 
      virtual void LoadCorrectionFile(std::string correction_path);
      virtual int LoadCorrectionString(char *correction_string);
      int LoadCorrectionString_csv(std::string lidar_correction_file);
      // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
      virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket); 
      // compute xyzi of points from decoded packet
      // param packet is the decoded packet; xyzi of points after computed is puted in frame    
      virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index);
      ETCorrections_V4 m_ET_corrections;
    protected:
      bool get_correction_file_;
      int last_frameid_ = -1;
  };
  } // namespace lidar
} // namespace hesai
#include "udp2_4_parser.cc"
#endif // UDP2_4_PARSER_H