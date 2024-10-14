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
#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif
  static constexpr int ET_MAX_CHANNEL_NUM = 512;

  struct ETCorrectionsHeader_V1V2 {
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
    ETCorrectionsHeader_V1V2():delimiter(), reserved1(), reserved2(), channel_number(), 
    mirror_number_reserved3(), angle_division(1), apha(), beta(), gamma()
    {
      major_version = 0;
      min_version = 0;
    }
  } PACKED;
  struct ETCorrectionsHeader_V3_4 {
    uint8_t delimiter[2];
    uint8_t major_version;
    uint8_t min_version;
    uint8_t reserved1;
    uint8_t reserved2;
    uint16_t channel_number;
    uint8_t mirror_nummber;
    uint8_t turn_number_per_frame;
    uint16_t angle_division;
    int16_t apha;
    int16_t beta;
    ETCorrectionsHeader_V3_4():delimiter(), reserved1(), reserved2(), channel_number(), 
    mirror_nummber(), turn_number_per_frame(), angle_division(1), apha(), beta()
    {
      major_version = 0;
      min_version = 0;
    }
  } PACKED;
  struct ETCorrectionsHeader {
    uint8_t delimiter[2];
    uint8_t major_version;
    uint8_t min_version;
    uint8_t reserved1;
    uint8_t reserved2;
    uint16_t channel_number;
    uint8_t mirror_number_reserved3;
    uint16_t angle_division;
    int16_t apha;
    int16_t beta;
    int16_t gamma;
    ETCorrectionsHeader():angle_division(1)
    {
      major_version = 0;
      min_version = 0;
    }
    void getDataFromV1V2(ETCorrectionsHeader_V1V2 &header) {
      delimiter[0] = header.delimiter[0];
      delimiter[1] = header.delimiter[1];
      major_version = header.major_version;
      min_version = header.min_version;
      reserved1 = header.reserved1;
      reserved2 = header.reserved2;
      channel_number = header.channel_number;
      mirror_number_reserved3 = header.mirror_number_reserved3;
      angle_division = header.angle_division;
      apha = header.apha;
      beta = header.beta;
      gamma = header.gamma;
    }
    void getDataFromV3_4(ETCorrectionsHeader_V3_4 &header) {
      delimiter[0] = header.delimiter[0];
      delimiter[1] = header.delimiter[1];
      major_version = header.major_version;
      min_version = header.min_version;
      reserved1 = header.reserved1;
      reserved2 = header.reserved2;
      channel_number = header.channel_number;
      mirror_number_reserved3 = header.mirror_nummber;
      angle_division = header.angle_division;
      apha = header.apha;
      beta = header.beta;
    }
  } PACKED;
  struct ETCorrections {
    struct ETCorrectionsHeader header;
    uint8_t min_version;
    uint8_t turn_number_per_frame;
    int16_t gamma[8];
    float gamma_f[8];
    float azimuths[ET_MAX_CHANNEL_NUM];
    float elevations[ET_MAX_CHANNEL_NUM];
    int16_t raw_azimuths[ET_MAX_CHANNEL_NUM];
    int16_t raw_elevations[ET_MAX_CHANNEL_NUM];
    int16_t elevation_adjust[3000];
    int16_t azimuth_adjust[3000];
    float elevation_adjust_f[3000];
    float azimuth_adjust_f[3000];
    int16_t elevation_offset_delta[8];
    int16_t azimuth_offset_delta[8];
    float elevation_offset_delta_f[8];
    float azimuth_offset_delta_f[8];
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
    float getAziAdjustV4(float azi, float ele, int mirror_id) const {
        const float azimuth_fov = 120.0f;
        const float elevation_fov = 3.2f;
        const float adjust_interval_resolution = 0.1f;
        // T * (120 / H / 0.1 + 1) * (3.2 / V / 0.1 + 1)
        int azimuth_offset_num = int(azimuth_fov / (azimuth_adjust_interval * adjust_interval_resolution) + 1);
        int elevation_offset_num = int(elevation_fov / (elevation_adjust_interval * adjust_interval_resolution) + 1);
        int offset_index1 = int((azi + azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution));      //azi dimension
        int offset_index2 = int((ele + elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution));      //ele dimension
        if (offset_index1 >= (azimuth_offset_num - 1) || offset_index2 >= (elevation_offset_num - 1)) return 0;
        if (offset_index1 < 0  || offset_index2 < 0) return 0;
        float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
        float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - ele - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
        // az_offset * ele_offset * m_id
        auto mr_offset = azimuth_offset_num * elevation_offset_num * mirror_id;
        auto az_index1 = offset_index1 + offset_index2 * azimuth_offset_num + mr_offset;
        auto az_index2 = offset_index1 + (offset_index2 + 1) * azimuth_offset_num + mr_offset;
        // azimuth_adjust
        float offset1 = coefficient1 * azimuth_adjust[az_index1] + (1 - coefficient1) * azimuth_adjust[az_index1 + 1];
        float offset2 = coefficient1 * azimuth_adjust[az_index2] + (1 - coefficient1) * azimuth_adjust[az_index2 + 1];
        return (coefficient2 * offset1 + (1 - coefficient2) * offset2) / header.angle_division;
    }
    float getEleAdjustV4(float azi, float ele, int mirror_id) const { 
        const float azimuth_fov = 120.0f;
        const float elevation_fov = 3.2f;
        const float adjust_interval_resolution = 0.1f;
        // T * (120 / H / 0.1 + 1) * (3.2 / V / 0.1 + 1)
        int azimuth_offset_num = int(azimuth_fov / (azimuth_adjust_interval * adjust_interval_resolution) + 1);
        int elevation_offset_num = int(elevation_fov / (elevation_adjust_interval * adjust_interval_resolution) + 1);
        int offset_index1 = int((azi + azimuth_fov / 2) /  (azimuth_adjust_interval * adjust_interval_resolution));      //azi dimension
        int offset_index2 = int((ele + elevation_fov / 2) /  (elevation_adjust_interval * adjust_interval_resolution));      //ele dimension
        if (offset_index1 >= (azimuth_offset_num - 1)  || offset_index2 >= (elevation_offset_num - 1)) return 0;
        if (offset_index1 < 0  || offset_index2 < 0) return 0;
        float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
        float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - ele - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
        // az_offset * ele_offset * m_id
        auto mr_offset = azimuth_offset_num * elevation_offset_num * mirror_id;
        auto az_index1 = offset_index1 + offset_index2 * azimuth_offset_num + mr_offset;
        auto az_index2 = offset_index1 + (offset_index2 + 1) * azimuth_offset_num + mr_offset;
        // elevation_adjust
        float offset1 = coefficient1 * elevation_adjust[az_index1] + (1 - coefficient1) * elevation_adjust[az_index1 + 1];
        float offset2 = coefficient1 * elevation_adjust[az_index2] + (1 - coefficient1) * elevation_adjust[az_index2 + 1];
        return (coefficient2 * offset1 + (1 - coefficient2) * offset2) / header.angle_division;
    }
  };

#ifdef _MSC_VER
#pragma pack(pop)
#endif

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

      // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
      virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);

      // compute xyzi of points from decoded packet
      // param packet is the decoded packet; xyzi of points after computed is puted in frame    
      virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index);
      ETCorrections corrections_;      

    // 类维护的成员变量 
    protected:
      // int view_mode_;           
      // std::string pcap_file_;
      // std::string lidar_correction_file_;
      bool get_correction_file_;
      int last_frameid_ = -1;
  };



}  // namespace lidar
}  // namespace hesai

#include "udp2_5_parser.cc"


#endif // UDP2_5_PARSER_H
