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
#ifndef GENERAL_PARSER_GPU_H_
#define GENERAL_PARSER_GPU_H_
#define MAX_LASER_NUM (512)
#include <stdint.h>
#include <iostream>
#include "nvbuffer.h"
#include <iostream>
#include <fstream>
#include <string>
#include <semaphore.h>
#include <list>
#include <vector>
#include "lidar_types.h"
#include "plat_utils.h"
#ifndef M_PI
#define M_PI 3.1415926535898
#endif

#ifndef CIRCLE
#define CIRCLE 36000
#endif

#ifndef HALF_CIRCLE 
#define HALF_CIRCLE 18000
#endif

#define PANDAR_HAS_MEMBER(C, member) has_##member<C>::value
namespace hesai
{
namespace lidar
{

namespace gpu
{
  template <typename T_Point>
  __device__ inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
  {
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
  {
    point.x = value;
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
  {
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
  {
    point.y = value;
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
  {
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
  {
    point.z = value;
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                        const uint8_t& value)
  {
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                      const uint8_t& value)
  {
    point.intensity = value;
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                        const double& value)
  {
  }

  template <typename T_Point>
  __device__ inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                      const double& value)
  {
    point.timestamp = value;
  }

} // namespace gpu

template <typename PointT>
struct PointCloudStruct  {
  uint32_t frame_id = 0;
  int seq = 0;
  uint64_t sensor_timestamp[kMaxPacketNumPerFrame];
  float azimuths[kMaxPacketNumPerFrame * kMaxPointsNumPerPacket];
  uint16_t distances[kMaxPacketNumPerFrame * kMaxPointsNumPerPacket];
  uint8_t reflectivities[kMaxPacketNumPerFrame * kMaxPointsNumPerPacket];
  uint16_t spin_speed[kMaxPacketNumPerFrame];
  float firetimes[kMaxPointsNumPerPacket];
  PointT points[kMaxPacketNumPerFrame * kMaxPointsNumPerPacket];
};
// class GeneralParserGpu
// the GeneralParserGpu class is a base class for computing points with gpu
// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class GeneralParserGpu {
 public:
  GeneralParserGpu();
  ~GeneralParserGpu();
  virtual int LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  virtual void LoadFiretimesFile(std::string firetimes_path);
  virtual int LoadFiretimesString(char *firetimes_string);
  
  // compute xyzi of points from decoded packet， use gpu device
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  void SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw);
  Transform transform_;
  bool corrections_loaded_ = false;
  protected:
  double firetime_correction_[kMaxPointsNumPerPacket];
  MemBufferClass<PointCloudStruct<T_Point>> frame_;
};
}
}
#include "general_parser_gpu.cu"

#endif  // GENERAL_PARSER_GPU_H_
