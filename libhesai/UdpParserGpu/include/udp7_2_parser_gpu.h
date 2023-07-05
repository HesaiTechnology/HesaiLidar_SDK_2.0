#ifndef Udp7_2_PARSER_GPU_H_
#define Udp7_2_PARSER_GPU_H_
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
// class Udp7_2ParserGpu
// computes points for PandarFT120
// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class Udp7_2ParserGpu: public GeneralParserGpu<T_Point>{
 private:
  float* channel_azimuths_cu_;
  float* channel_elevations_cu_;
  float* raw_azimuths_cu_;
  float* raw_elevations_cu;
  uint16_t* raw_distances_cu_;
  uint8_t* raw_reflectivities_cu_;
  uint64_t* raw_sensor_timestamp_cu_;

 public:
  Udp7_2ParserGpu();
  ~Udp7_2ParserGpu();

  // compute xyzi of points from decoded packet， use gpu device
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  virtual int LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  int LoadCorrectionDatData(char *correction_string);
  int LoadCorrectionCsvData(char *correction_string);
  PandarFTCorrections corrections_;
  bool corrections_loaded_;
};
}
}

#include "udp7_2_parser_gpu.cu"
#endif  // Udp7_2_PARSER_GPU_H_
