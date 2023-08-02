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
