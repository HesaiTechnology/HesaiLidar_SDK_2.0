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

/*
 * File:       lidar.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: 
 */

#ifndef LIDAR_TYPES_H_
#define LIDAR_TYPES_H_
#include <stdint.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <algorithm>
#include <functional>
#include <memory>
#include <atomic>
#define CHANNEL_NUM 256
#define PACKET_NUM 3600
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

//max point num of one packet, laser_num * block_num <= kMaxPointsNumPerPacket
static constexpr uint16_t kMaxPointsNumPerPacket = 512;
//max packet num of one frame, it means the capacity of frame buffer
static constexpr uint16_t kMaxPacketNumPerFrame = 5000;
//max points num of one frame
static constexpr uint32_t kMaxPointsNumPerFrame = kMaxPointsNumPerPacket * kMaxPacketNumPerFrame;
//half of the max value in degrees, 1 LSB represents 0.01 degree, float type
static constexpr float kHalfCircleFloat = 18000.0f;
//half of the max value in degrees, 1 LSB represents 0.01 degree, int type
static constexpr int kHalfCircleInt = 18000;
//max value in degrees, 1 LSB represents 0.01 degree
static constexpr int kCircle = 36000;
//laser azimuth resolution, 100 units represents 1 degree, int type
static constexpr int kResolutionInt = 100;
//laser azimuth resolution, 100 units represents 1 degree, float type
static constexpr float kResolutionFloat = 100.0f;
//conversion factor between second and micorsecond
static constexpr float kMicrosecondToSecond = 1000000.0f;
static constexpr int kMicrosecondToSecondInt = 1000000;
//the difference between last azimuth and current azimuth must be greater than this angle in split frame function,
//to avoid split frame unsuccessfully
static constexpr uint16_t kSplitFrameMinAngle = 300;
//laser fine azimuth resolution, 1 LSB represents 0.01 / 256 degree, float type
static constexpr float kFineResolutionFloat = 256.0f;
static constexpr float kAllFineResolutionFloat = kResolutionFloat * kFineResolutionFloat;
//laser fine azimuth resolution, 1 LSB represents 0.01 / 256 degree, int type
static constexpr int kFineResolutionInt = 256;
static constexpr int kAllFineResolutionInt = kResolutionInt * kFineResolutionInt;
//synchronize host time with sensor time per kPcapPlaySynchronizationCount packets
static constexpr int kPcapPlaySynchronizationCount = 100;
//min points of one frame for displaying frame message
static constexpr int kMinPointsOfOneFrame = 1000;
//max time interval between two frame
static constexpr int kMaxTimeInterval = 150000;

//length of fault message packet
static constexpr int kFaultMessageLength = 99;

static constexpr int kPacketBufferSize = 36000;
//default udp data max lenth
static const uint16_t kBufSize = 1500;
typedef struct LidarPointXYZI
{
    float x; 
    float y;             
    float z;             
    float intensity;     
} LidarPointXYZI;

typedef struct LidarPointXYZIRT
{
    float x; 
    float y;             
    float z;             
    float intensity;  
    uint16_t ring;
    double timestamp;  
} LidarPointXYZIRT;

typedef struct LidarPointXYZICRT
{
    float x; 
    float y;             
    float z;             
    float intensity;  
    float confidence;  
    uint16_t ring;
    double timestamp;  
} LidarPointXYZICRT;

typedef struct LidarPointRTHI
{
    int theta; 
    int phi;               
    int radius;            
    int intensity;         
} LidarPointRTHI;

typedef struct _LidarDecodeConfig {
    int fov_start;
    int fov_end;

    _LidarDecodeConfig()
    {
      fov_start = -1;
      fov_end = -1;
    }
} LidarDecodeConfig;

#define POINT_DATA_OFFSET               (0)
#define POINT_DATA_LEN                  (sizeof(PointDecodeData) * kMaxPacketNumPerFrame * kMaxPointsNumPerPacket)
#define SENSOR_TIMESTAMP_OFFSET         (POINT_DATA_OFFSET + POINT_DATA_LEN)
#define SENSOR_TIMESTAMP_LEN            (sizeof(uint64_t) * kMaxPacketNumPerFrame)
#define POINTS_OFFSET                   (SENSOR_TIMESTAMP_OFFSET + SENSOR_TIMESTAMP_LEN)

struct PointDecodeData {
  float azimuth;
  float elevation;
  uint16_t distances;
  uint8_t reflectivities;
  uint8_t confidence;
  uint8_t chn_index;
  uint8_t mirror_index;
} PACKED;

template <typename PointT>
class LidarDecodedFrame
{
    public:
    LidarDecodedFrame() {
        total_memory = new uint8_t[sizeof(PointDecodeData) * kMaxPacketNumPerFrame * kMaxPointsNumPerPacket
                                   + sizeof(uint64_t) * kMaxPacketNumPerFrame
                                   + sizeof(PointT) * kMaxPacketNumPerFrame * kMaxPointsNumPerPacket];

        pointData = reinterpret_cast<PointDecodeData* >(total_memory + POINT_DATA_OFFSET);
        sensor_timestamp = reinterpret_cast<uint64_t* >(total_memory + SENSOR_TIMESTAMP_OFFSET);
        points = reinterpret_cast <PointT* >(total_memory + POINTS_OFFSET);

        host_timestamp = 0;
        major_version = 0;
        minor_version = 0;
        return_mode = 0;
        spin_speed = 0;
        points_num = 0;
        packet_num = 0;
        block_num = 0;
        laser_num = 0; 
        per_points_num = 0;
        scan_complete = false;
        distance_unit = 0.0;
        frame_index = 0;
        lidar_state = -1;
        work_mode = -1;
        use_timestamp_type = 0;
    };
    ~LidarDecodedFrame() {
        if (total_memory) {
          delete total_memory;
          total_memory = nullptr;
          sensor_timestamp = nullptr;
          points = nullptr;
          pointData = nullptr;
        }
    }
    LidarDecodedFrame(const LidarDecodedFrame&) = delete;
    LidarDecodedFrame& operator=(const LidarDecodedFrame&) = delete;
    void Update(){
      host_timestamp = 0;
      major_version = 0;
      minor_version = 0;
      return_mode = 0;
      spin_speed = 0;
      points_num = 0;
      packet_num = 0;
      block_num = 0;
      laser_num = 0; 
      per_points_num = 0;
      scan_complete = false;
      distance_unit = 0;
      lidar_state = -1;
      work_mode = -1;
      frame_index++;
    }
    uint64_t host_timestamp;   
    uint64_t* sensor_timestamp = nullptr; 
    uint8_t major_version;
    uint8_t minor_version;
    uint16_t return_mode;
    uint16_t spin_speed;        
    uint32_t points_num; 
    uint32_t packet_num;
    uint8_t* total_memory = nullptr;                  
    PointT* points = nullptr;
    PointDecodeData* pointData = nullptr;
    uint16_t block_num;
    uint16_t laser_num;
    uint32_t per_points_num; 
    bool scan_complete;
    double distance_unit;
    int frame_index;
    int16_t lidar_state;
    int16_t work_mode;
    uint16_t use_timestamp_type;
    LidarDecodeConfig config;
};


struct UdpPacket {
  uint8_t buffer[1500];
  int16_t packet_len;
  bool is_timeout = false;
  uint64_t recv_timestamp;
  uint32_t ip;
  uint16_t port;
  UdpPacket(const uint8_t* data = nullptr, uint32_t sz = 0, uint64_t tm = 0, 
            uint32_t i_ip = 0, uint32_t i_port = 0)
  : packet_len(sz), recv_timestamp(tm), ip(i_ip), port(i_port)
  {
    if(data != nullptr)
      memcpy(buffer, data, packet_len);
  }
};

typedef std::vector<uint8_t> u8Array_t;
typedef std::vector<uint16_t> u16Array_t;
typedef std::vector<uint32_t> u32Array_t;
typedef std::vector<double> doubleArray_t;
typedef std::vector<u8Array_t> u8ArrayArray_t;
typedef std::vector<doubleArray_t> doubleArrayArray_t;
typedef std::vector<u16Array_t> u16ArrayArray_t;
typedef std::vector<std::string> stringArray_t;
typedef std::vector<UdpPacket> UdpFrame_t;
typedef std::vector<UdpFrame_t> UdpFrameArray_t;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

}  // namespace lidar
}  // namespace hesai

 
#endif // LIDAR_TYPES_H_
