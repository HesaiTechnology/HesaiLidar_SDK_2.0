#ifndef UDP4_3_PARSER_GPU_H_
#define UDP4_3_PARSER_GPU_H_
#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include "general_parser_gpu.h"

namespace hesai
{
namespace lidar
{
struct Corrections {
  struct Header {
    uint8_t delimiter[2];
    uint8_t version[2];
    uint8_t max_channel_num;
    uint8_t nmirrors;
    uint8_t nframes;
    uint8_t frame_config[8];
    uint8_t resolution;
  } header;
};

struct CorrectionsV1_3 : Corrections {
  uint16_t mirror_azi_begins[3];
  uint16_t mirror_azi_ends[3];
  int16_t channel_azimuths[kMaxPointsNumPerPacket];
  int16_t channel_elevations[kMaxPointsNumPerPacket];
  int8_t dazis[36000];
  int8_t deles[36000];
  uint8_t SHA256[32];
};

struct CorrectionsV1_5 : Corrections {
  uint32_t mirror_azi_begins[3];
  uint32_t mirror_azi_ends[3];
  int32_t channel_azimuths[128];
  int32_t channel_elevations[128];
  int8_t dazis[128 * 180];
  int8_t deles[128 * 180];
  uint8_t SHA256[32];
};

// class Udp4_3ParserGpu
// computes points for PandarAT128
// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class Udp4_3ParserGpu: public GeneralParserGpu<T_Point>{

 private:
  // corrections
  bool corrections_loaded_;
  Corrections::Header corrections_header;
  float raw_azimuth_begin;
  float mirror_azi_begins[3];
  float mirror_azi_ends[3];
  int32_t* channel_azimuths_cu_;
  int32_t* channel_elevations_cu_;
  int8_t* dazis_cu;
  int8_t* deles_cu;
  uint8_t* raw_fine_azimuths_cu;
  float* raw_azimuths_cu_;
  uint16_t* raw_distances_cu_;
  uint8_t* raw_reflectivities_cu_;
  uint32_t* mirror_azi_begins_cu;
  uint32_t* mirror_azi_ends_cu;
  uint64_t* raw_sensor_timestamp_cu_;

 public:
  Udp4_3ParserGpu();
  ~Udp4_3ParserGpu();

  // compute xyzi of points from decoded packet， use gpu device
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  virtual int LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  
};
}
}

#include "udp4_3_parser_gpu.cu"
#endif  // UDP4_3_PARSER_GPU_H_
