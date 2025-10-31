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
 * File:       lidar_type.h
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
#include <array>
#include <cmath>
#include "inner_com.h"
#include "driver_param.h"
#include "logger.h"
#include <fstream>
namespace hesai
{
namespace lidar
{



#pragma pack(push, 1)
struct LidarPointXYZI
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;     
};

struct LidarPointXYZIRT
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;  
    uint16_t ring;
    double timestamp;  
};

struct LidarPointXYZICRT
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;  
    uint8_t confidence;  
    uint16_t ring;
    double timestamp;  
};

struct LidarPointXYZICRTT
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;  
    uint8_t confidence;  
    uint16_t ring;
    uint64_t timeSecond;  
    uint32_t timeNanosecond;  
};

struct LidarPointXYZICWERT
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;  
    uint8_t confidence;  
    uint8_t weightFactor;
    uint8_t envLight;
    uint16_t ring;
    double timestamp;  
};

struct LidarPointRTHI
{
    int theta; 
    int phi;               
    int radius;            
    int intensity;         
};
#pragma pack(pop)

struct LidarImuData {
  double timestamp; 
  double imu_accel_x;
  double imu_accel_y;
  double imu_accel_z;
  double imu_ang_vel_x;
  double imu_ang_vel_y;
  double imu_ang_vel_z;
  bool flag;
  LidarImuData() {
    flag = false;
    timestamp = 0;
    imu_accel_x = 0;
    imu_accel_y = 0;
    imu_accel_z = 1;
    imu_ang_vel_x = -1;
    imu_ang_vel_y = -1;
    imu_ang_vel_z = -1;
  }

  bool isSameImuValue(const LidarImuData& other) const {
    return imu_accel_x == other.imu_accel_x &&
           imu_accel_y == other.imu_accel_y &&
           imu_accel_z == other.imu_accel_z &&
           imu_ang_vel_x == other.imu_ang_vel_x &&
           imu_ang_vel_y == other.imu_ang_vel_y &&
           imu_ang_vel_z == other.imu_ang_vel_z;
  }
};

struct FrameDecodeParam {
  bool pcap_time_synchronization; 
  bool firetimes_flag;
  bool dcf_flag;
  bool distance_correction_flag;
  bool xt_spot_correction;
  bool update_function_safety_flag;
  bool enable_packet_loss_tool_;
  bool enable_packet_timeloss_tool_;
  bool packet_timeloss_tool_continue_;
  bool dirty_mapping_reflectance;
  uint8_t use_timestamp_type;  // 0: point cloud time, other: local time
  uint8_t echo_mode_filter;
  LidarDecodeConfig config;
  TransformParam transform;
  int rotation_flag;
  RemakeConfig remake_config;
  bool et_blooming_filter_flag;
  bool use_cuda;
  float frame_frequency;
  float default_frame_frequency;

  FrameDecodeParam() {
    use_timestamp_type = 0;
    pcap_time_synchronization = false;
    firetimes_flag = false;
    dcf_flag = false;
    distance_correction_flag = false;
    xt_spot_correction = false;
    update_function_safety_flag = false;
    echo_mode_filter = 0;
    rotation_flag = 0;
    enable_packet_loss_tool_ = false;
    enable_packet_timeloss_tool_ = false;
    packet_timeloss_tool_continue_ = false;
    dirty_mapping_reflectance = false;
    et_blooming_filter_flag = false;
    use_cuda = false;
    frame_frequency = -1;
    default_frame_frequency = DEFAULT_MAX_MULTI_FRAME_NUM;
  }
  void UpdateRotation(int rotation) {
    if (abs(rotation_flag) == 10240) return;
    if (abs(rotation_flag) > 128 * 8) rotation_flag /= 128;
    rotation_flag += rotation;
  }
  void Init(const DriverParam& param) {
    use_timestamp_type = param.decoder_param.use_timestamp_type;
    pcap_time_synchronization = param.decoder_param.pcap_play_synchronization;
    firetimes_flag = true;
    dcf_flag = true;
    distance_correction_flag = param.decoder_param.distance_correction_flag;
    xt_spot_correction = param.decoder_param.xt_spot_correction;
    config.fov_start = param.decoder_param.fov_start;
    config.fov_end = param.decoder_param.fov_end;
    transform = param.decoder_param.transform_param;
    enable_packet_loss_tool_ = param.decoder_param.enable_packet_loss_tool;
    enable_packet_timeloss_tool_ = param.decoder_param.enable_packet_timeloss_tool;
    packet_timeloss_tool_continue_ = param.decoder_param.packet_timeloss_tool_continue;
    remake_config = param.decoder_param.remake_config;
    et_blooming_filter_flag = param.decoder_param.et_blooming_filter_flag;
    use_cuda = param.use_gpu;
    update_function_safety_flag = param.decoder_param.update_function_safety_flag;
    echo_mode_filter = param.decoder_param.echo_mode_filter;
    ParseChannelFovFilterPath(param.decoder_param.channel_fov_filter_path, config.channel_fov_filter);
    ParseMultiFovFilterRanges(param.decoder_param.multi_fov_filter_ranges, config.multi_fov_filter_ranges);
    frame_frequency = param.decoder_param.frame_frequency;
    default_frame_frequency = param.decoder_param.default_frame_frequency;
    if (default_frame_frequency == 0) {
      default_frame_frequency = DEFAULT_MAX_MULTI_FRAME_NUM;
      LogError("default_frame_frequency cannot be 0, reset to %f\n", default_frame_frequency);
    }
  }

  int ParseChannelFovFilterPath(std::string file, std::map<int, std::vector<std::pair<int, int>>>& channel_fov_filter) {
    if (file == "") return -1;
    if (!std::ifstream(file).good()) {
      LogError("channel fov file does not exist: %s", file.c_str());
      return -1;
    }
    std::ifstream infile(file);
    if (!infile.is_open()) {
      LogError("Failed to open channel fov filter file: %s", file.c_str());
      return -1;
    }

    std::string line;
    while (std::getline(infile, line)) {
      if (line.empty() || line[0] == '#') continue; // Skip empty lines or comments

      size_t colon_pos = line.find(':');
      if (colon_pos == std::string::npos) continue;

      int channel = std::stoi(line.substr(0, colon_pos));
      std::string ranges_str = line.substr(colon_pos + 1);

      std::vector<std::pair<int, int>> ranges;
      size_t start = 0;
      size_t end = ranges_str.find(';');

      while (end != std::string::npos) {
        std::string range_part = ranges_str.substr(start, end - start);
        size_t bracket_open = range_part.find('[');
        size_t comma = range_part.find(',');
        size_t bracket_close = range_part.find(']');

        if (bracket_open != std::string::npos && comma != std::string::npos && bracket_close != std::string::npos) {
          int low = std::stoi(range_part.substr(bracket_open + 1, comma - bracket_open - 1));
          int high = std::stoi(range_part.substr(comma + 1, bracket_close - comma - 1));
          ranges.emplace_back(low, high);
        }

        start = end + 1;
        end = ranges_str.find(';', start);
      }

      // Handle last range
      std::string range_part = ranges_str.substr(start);
      size_t bracket_open = range_part.find('[');
      size_t comma = range_part.find(',');
      size_t bracket_close = range_part.find(']');

      if (bracket_open != std::string::npos && comma != std::string::npos && bracket_close != std::string::npos) {
        int low = std::stoi(range_part.substr(bracket_open + 1, comma - bracket_open - 1));
        int high = std::stoi(range_part.substr(comma + 1, bracket_close - comma - 1));
        ranges.emplace_back(low, high);
      }

      channel_fov_filter[channel] = ranges;
    }

    infile.close();
    //遍历打印channel_fov_filter
    for (const auto& entry : channel_fov_filter) {
      int channel = entry.first;
      const std::vector<std::pair<int, int>>& ranges = entry.second;

      std::string channel_info = "Channel: " + std::to_string(channel) + ", Ranges: ";
      for (const auto& range : ranges) {
          channel_info += "[" + std::to_string(range.first) + ", " + std::to_string(range.second) + "] ";
      }
      LogInfo(channel_info.c_str());
    }
    return 0;
  }
  int ParseMultiFovFilterRanges(std::string ranges_str, std::vector<std::pair<int, int>>& ranges) {
    if (ranges_str == "") return -1;
    size_t start = 0;
    size_t end = ranges_str.find(';');

    while (end != std::string::npos) {
      std::string range_part = ranges_str.substr(start, end - start);
      size_t bracket_open = range_part.find('[');
      size_t comma = range_part.find(',');
      size_t bracket_close = range_part.find(']');

      if (bracket_open != std::string::npos && comma != std::string::npos && bracket_close != std::string::npos) {
        int low = std::stoi(range_part.substr(bracket_open + 1, comma - bracket_open - 1));
        int high = std::stoi(range_part.substr(comma + 1, bracket_close - comma - 1));
        ranges.emplace_back(low, high);
      }

      start = end + 1;
      end = ranges_str.find(';', start);
    }

    // Handle last range
    std::string range_part = ranges_str.substr(start);
    size_t bracket_open = range_part.find('[');
    size_t comma = range_part.find(',');
    size_t bracket_close = range_part.find(']');

    if (bracket_open != std::string::npos && comma != std::string::npos && bracket_close != std::string::npos) {
      int low = std::stoi(range_part.substr(bracket_open + 1, comma - bracket_open - 1));
      int high = std::stoi(range_part.substr(comma + 1, bracket_close - comma - 1));
      ranges.emplace_back(low, high);
    }
    
    std::string ranges_info = "";
    //遍历打印channel_fov_filter
    for (const auto& range : ranges) {
      ranges_info += "[" + std::to_string(range.first) + ", " + std::to_string(range.second) + "] ";
    }
    LogInfo(ranges_info.c_str());
    
    return 0;
  }
  int IsMultiFrameFrequency() const { 
    if (frame_frequency > 0 && frame_frequency < default_frame_frequency) {
      if (fmodf(default_frame_frequency, frame_frequency) <= 0.0000001) {
        return 1;
      } else {
        LogWarning("default_frame_frequency %lf is not a multiple of %lf, please check", default_frame_frequency, frame_frequency);
        return 0;
      }
    } else {
      return 0;
    }
  }
};

template <typename PointT>
class LidarDecodedFrame
{
    public:
    LidarDecodedFrame(uint16_t maxPacketNum = 5000, uint16_t maxNumPerPacket = 1024) {
        resetMalloc(maxPacketNum, maxNumPerPacket);
        lidar_state = -1;
        work_mode = -1;
        packet_num = 0;
        frame_index = 0;
        block_num = 0;
        laser_num = 0; 
        channel_num = 0;
        per_points_num = 0;
        distance_unit = 0.0;
        return_mode = 0;
        points_num = 0;
        frame_start_timestamp = 0;
        frame_end_timestamp = 0;
        scan_complete = false;
        multi_packet_num = 0;
        multi_points_num = 0;
    };
    ~LidarDecodedFrame() {
        if (total_memory) {
            delete[] total_memory;
            total_memory = nullptr;
            packetData = nullptr;
            points = nullptr;
            funcSafety = nullptr;
        }
        if (valid_points) {
          delete[] valid_points;
          valid_points = nullptr;
        }
        if (point_cloud_raw_data) {
          delete[] point_cloud_raw_data;
          point_cloud_raw_data = nullptr;
        }
        if (multi_frame_buffer) {
          delete[] multi_frame_buffer;
          multi_frame_buffer = nullptr;
          multi_points = nullptr;
        }
    }
    void resetMalloc(uint16_t maxPacketNum, uint16_t maxNumPerPacket) {
      maxPacketPerFrame = maxPacketNum;
      maxPointPerPacket = maxNumPerPacket;
      if (total_memory) {
          delete[] total_memory;
          total_memory = nullptr;
          packetData = nullptr;
          points = nullptr;
          funcSafety = nullptr;
      }
      if (valid_points) {
        delete[] valid_points;
      }
      total_memory = new uint8_t[sizeof(PacketDecodeData) * maxPacketPerFrame
                                   + sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket
                                   + sizeof(FunctionSafety) * maxPacketPerFrame];
      memset(total_memory, 0, sizeof(PacketDecodeData) * maxPacketPerFrame
                                   + sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket
                                   + sizeof(FunctionSafety) * maxPacketPerFrame);
      uint32_t offset = 0;
      packetData = reinterpret_cast<PacketDecodeData* >(total_memory + offset);
      offset += (sizeof(PacketDecodeData) * maxPacketPerFrame);
      points = reinterpret_cast <PointT* >(total_memory + offset);
      offset += (sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket);
      funcSafety = reinterpret_cast <FunctionSafety* >(total_memory + offset);
      offset += (sizeof(FunctionSafety) * maxPacketPerFrame);
      valid_points = new uint32_t[maxPacketPerFrame];
      if (fParam.IsMultiFrameFrequency() == 1) {
        multi_rate = fParam.default_frame_frequency / fParam.frame_frequency;
        int multi_num = multi_rate + 1;
        multi_frame_buffer = new uint8_t[sizeof(PointT) * maxPacketPerFrame * maxPointPerPacket * multi_num];
        multi_points = reinterpret_cast <PointT* >(multi_frame_buffer);
        LogInfo("fParam.frame_frequency: %lf, multi_rate : %d", fParam.frame_frequency, multi_rate);
      }
    }
    LidarDecodedFrame(const LidarDecodedFrame&) = delete;
    LidarDecodedFrame& operator=(const LidarDecodedFrame&) = delete;
    void Update() {
        packet_num = 0;
        points_num = 0;
        frame_start_timestamp = 0;
        frame_end_timestamp = 0;
        scan_complete = false;
        frame_index++;
        std::fill(et_echo_vec.begin(), et_echo_vec.end(), false);
    }
    void clearFuncSafety() {
      memset(funcSafety, 0, sizeof(FunctionSafety) * maxPacketPerFrame);
    }
    uint32_t getPointSize() const {
      return sizeof(PointT);
    }
    void MultiFrameUpdate() {
      multi_packet_num = 0;
      multi_points_num = 0;
      multi_frame_start_timestamp = 0;
      multi_frame_end_timestamp = 0;
      multi_frame_index++;
    }
    uint8_t* total_memory = nullptr; 
    uint32_t maxPacketPerFrame;
    uint32_t maxPointPerPacket;
    // configure
    FrameDecodeParam fParam;

    // frame parameter
    int16_t lidar_state;
    int16_t work_mode;
    uint16_t return_mode;
    uint32_t packet_num; 
    uint32_t* valid_points = nullptr;
    int frame_index;
    uint32_t points_num;
    double frame_start_timestamp;
    double frame_end_timestamp;
    std::string software_version = "xx.xx.xx";
    std::string hardware_version = "xx.xx.xx";
    // package parameter
    PacketDecodeData* packetData = nullptr; 
    FunctionSafety* funcSafety = nullptr;
    uint16_t block_num;
    uint16_t laser_num;  // channel number in point cloud
    uint16_t channel_num; // real channel number, when != 0, mean useful channel number
    uint32_t per_points_num; 
    uint8_t reserved[4];
    double distance_unit;
    bool scan_complete;
    // point parameter
    PointT* points = nullptr;
    //special output
    LidarImuData imu_config;
    // point cloud raw data
    bool frame_init_ = false;
    uint32_t point_cloud_size = 0;
    uint8_t* point_cloud_raw_data = nullptr;
    std::vector<bool> et_echo_vec;
    bool clear_every_frame = false;
    uint8_t *multi_frame_buffer = nullptr;
    PointT* multi_points = nullptr;
    uint32_t multi_packet_num = 0;
    uint32_t multi_points_num = 0;
    double multi_frame_start_timestamp = 0;
    double multi_frame_end_timestamp = 0;
    int multi_frame_index = 0;
    int multi_rate = 1;
};

}  // namespace lidar
}  // namespace hesai

 
#endif // LIDAR_TYPES_H_
