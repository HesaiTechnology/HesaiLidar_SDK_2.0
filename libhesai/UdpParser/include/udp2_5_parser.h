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
    ETCorrectionsHeader():angle_division(1)
    {
      major_version = 0;
      min_version = 0;
    }
  };
  struct ETCorrections {
    struct ETCorrectionsHeader header;
    float azimuths[ET_MAX_CHANNEL_NUM];
    float elevations[ET_MAX_CHANNEL_NUM];
    int16_t raw_azimuths[ET_MAX_CHANNEL_NUM];
    int16_t raw_elevations[ET_MAX_CHANNEL_NUM];
    int16_t elevation_adjust[3000];
    int16_t azimuth_adjust[3000];
    uint8_t azimuth_adjust_interval;
    uint8_t elevation_adjust_interval;
    // SHA-256_value
    uint8_t SHA_value[32];
    float getAziAdjustV2(float azi, float ele) const{
        float azimuth_fov = 120.0f;
        float elevation_fov = 25.0f;
        float adjust_interval_resolution = 0.5f;
        int azimuth_offset_num = int(azimuth_fov / (azimuth_adjust_interval * adjust_interval_resolution) + 1);
        int elevation_offset_num = int(elevation_fov / (elevation_adjust_interval * adjust_interval_resolution) + 1);
        int offset_index1 = int((azi + azimuth_fov / 2) /  (azimuth_adjust_interval * adjust_interval_resolution));      //azi dimension
        int offset_index2 = int((ele + elevation_fov / 2) /  (elevation_adjust_interval * adjust_interval_resolution));      //ele dimension
        if (offset_index1 >= (azimuth_offset_num - 1)  || offset_index2 >= (elevation_offset_num - 1)) return 0;
        if (offset_index1 < 0  || offset_index2 < 0) return 0;
        float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
        float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - ele - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
        float offset1 = coefficient1 * azimuth_adjust[offset_index1  + offset_index2 * azimuth_offset_num] + (1 - coefficient1) * azimuth_adjust[offset_index1 + 1 + offset_index2 * azimuth_offset_num];
        float offset2 = coefficient1 * azimuth_adjust[offset_index1 + (offset_index2 + 1) * azimuth_offset_num] + (1 - coefficient1) * azimuth_adjust[offset_index1 + 1 + (offset_index2 + 1) * azimuth_offset_num];
        return (coefficient2 * offset1 + (1 - coefficient2) * offset2) / header.angle_division;
    }
    float getEleAdjustV2(float azi, float ele) const{
        float azimuth_fov = 120.0f;
        float elevation_fov = 25.0f;
        float adjust_interval_resolution = 0.5f;
        unsigned int azimuth_offset_num = int(azimuth_fov / (azimuth_adjust_interval * adjust_interval_resolution) + 1);
        unsigned int elevation_offset_num = int(elevation_fov / (elevation_adjust_interval * adjust_interval_resolution) + 1);
        int offset_index1 = int((azi + azimuth_fov / 2) /  (azimuth_adjust_interval * adjust_interval_resolution));      //azi dimension
        int offset_index2 = int((ele + elevation_fov / 2) /  (elevation_adjust_interval * adjust_interval_resolution));      //ele dimension
        if (offset_index1 >= (int)(azimuth_offset_num - 1)  || offset_index2 >= (int)(elevation_offset_num - 1)) return 0;
        if (offset_index1 < 0  || offset_index2 < 0) return 0;
        float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
        float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - ele - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
        float offset1 = coefficient1 * elevation_adjust[offset_index1  + offset_index2 * azimuth_offset_num] + (1 - coefficient1) * elevation_adjust[offset_index1 + 1 + offset_index2 * azimuth_offset_num];
        float offset2 = coefficient1 * elevation_adjust[offset_index1 + (offset_index2 + 1) * azimuth_offset_num] + (1 - coefficient1) * elevation_adjust[offset_index1 + 1 + (offset_index2 + 1) * azimuth_offset_num];
        return (coefficient2 * offset1 + (1 - coefficient2) * offset2) / header.angle_division;
    }
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

      // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
      virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);

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
