/*
  This file encapsulates the data types for ET radar correction files and defines a class for an ET parser.
*/

#ifndef UDP2_5_PARSER_H
#define UDP2_5_PARSER_H

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
  
  static constexpr int ET_MAX_CHANNEL_NUM = 512;

  struct ETCorrections {
    uint8_t delimiter[2];
    uint8_t major_version;
    uint8_t min_version;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t channel_number;
    uint8_t mirror_nummber_reserved3;
    uint16_t angle_division;
    int16_t apha;
    int16_t beta;
    int16_t gamma;
    float azimuths[ET_MAX_CHANNEL_NUM];
    float elevations[ET_MAX_CHANNEL_NUM];
    int16_t raw_azimuths[ET_MAX_CHANNEL_NUM];
    int16_t raw_elevations[ET_MAX_CHANNEL_NUM];
    // SHA-256_value
    uint8_t SHA_value[32];
    
  };

  struct ETCorrectionsHeader {
    uint8_t delimiter[2];
    uint8_t major_version;
    uint8_t min_version;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t channel_number;
    uint8_t mirror_nummber_reserved3;
    uint16_t angle_division;
    int16_t apha;
    int16_t beta;
    int16_t gamma;
  };


  // class Udp2_5Parser
  // parsers packets and computes points for ET25
  // you can parser the upd or pcap packets using the DocodePacket fuction
  // you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
  template<typename T_Point>
  class Udp2_5Parser : public GeneralParser<T_Point> {
    public:
      Udp2_5Parser();
      virtual ~Udp2_5Parser();
      
      // 
      int16_t GetVecticalAngle(int channel); 

      // determine whether frame splitting is needed
      bool IsNeedFrameSplit(uint16_t frame_id);


      // get lidar correction file from local file,and pass to udp parser 
      virtual void LoadCorrectionFile(std::string correction_path);
      virtual int LoadCorrectionString(char *correction_string);
      int LoadCorrectionDatData(char *correction_string);
      int LoadCorrectionCsvData(char *correction_string);
  
      // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
      // udp_packet is the origin udp packet, output is the decoded packet
      virtual int DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket);  

      // compute xyzi of points from decoded packet
      // param packet is the decoded packet; xyzi of points after computed is puted in frame    
      virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet);
      ETCorrections corrections_;
      

    // 类维护的成员变量 
    protected:
      // int view_mode_;           
      // std::string pcap_file_;
      // std::string lidar_correction_file_;
      bool get_correction_file_;
      int last_frameid_ = 0;
  };



}  // namespace lidar
}  // namespace hesai

#include "udp2_5_parser.cc"


#endif // UDP2_5_PARSER_H