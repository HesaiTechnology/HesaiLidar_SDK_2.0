#ifndef Udp1_4_PARSER_GPU_H_
#define Udp1_4_PARSER_GPU_H_
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

#ifndef M_PI
#define M_PI 3.1415926535898
#endif

namespace hesai
{
namespace lidar
{
// class Udp1_4ParserGpu
// computes points for Pandar128
// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class Udp1_4ParserGpu: public GeneralParserGpu<T_Point>{
 private:
  float* channel_azimuths_cu_;
  float* channel_elevations_cu_;
  float* raw_azimuths_cu_;
  uint16_t* raw_distances_cu_;
  uint8_t* raw_reflectivities_cu_;
  uint64_t* raw_sensor_timestamp_cu_;
 public:
  Udp1_4ParserGpu();
  ~Udp1_4ParserGpu();

  // compute xyzi of points from decoded packet， use gpu device
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  virtual int LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  bool corrections_loaded_;
  
};
}
}
#include "udp1_4_parser_gpu.cu"
#endif  // Udp1_4_PARSER_GPU_H_
