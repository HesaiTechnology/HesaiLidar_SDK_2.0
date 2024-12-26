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
 * File:       general_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente GeneralParser class
*/

#include "general_parser.h"
#include <iomanip> 

using namespace hesai::lidar;
template <typename T_Point>
GeneralParser<T_Point>::GeneralParser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  this->enable_update_monitor_info_ = false;
  this->last_azimuth_ = 0;
  this->last_last_azimuth_ = 0;
  this->total_packet_count_ = 0;
  this->enable_packet_loss_tool_ = false;
  this->enable_packet_timeloss_tool_ = false;
  this->packet_timeloss_tool_continue_ = false;
  this->rotation_flag = 1;
  this->xt_spot_correction = false;
  for (int i = 0; i < CIRCLE; ++i) {
    this->sin_all_angle_[i] = std::sin(i * 2 * M_PI / CIRCLE);
    this->cos_all_angle_[i] = std::cos(i * 2 * M_PI / CIRCLE);
  }
}


template <typename T_Point>
int GeneralParser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  (void)frame;
  (void)packet_index;
  return 0;
}
template <typename T_Point>
void GeneralParser<T_Point>::FrameNumAdd() {
  this->compute_packet_num++;
}
template <typename T_Point>
int GeneralParser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  (void)frame;
  (void)udpPacket;
  return 0;
} 
template <typename T_Point>
void GeneralParser<T_Point>::ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info) {
  (void)udp_packet;
  (void)fault_message_info;
  return;
}

template <typename T_Point>
GeneralParser<T_Point>::~GeneralParser() { LogInfo("release general parser"); }

template <typename T_Point>
void GeneralParser<T_Point>::SetFrameAzimuth(float frame_start_azimuth) {
  this->frame_start_azimuth_ = frame_start_azimuth;
}
template <typename T_Point>
void GeneralParser<T_Point>::EnableUpdateMonitorInfo() {
  this->enable_update_monitor_info_ = true;
}
template <typename T_Point>
void GeneralParser<T_Point>::DisableUpdateMonitorInfo() {
  this->enable_update_monitor_info_ = false;
}
template <typename T_Point>
uint16_t *GeneralParser<T_Point>::GetMonitorInfo1() { return this->monitor_info1_; }
template <typename T_Point>
uint16_t *GeneralParser<T_Point>::GetMonitorInfo2() { return this->monitor_info2_; }
template <typename T_Point>
uint16_t *GeneralParser<T_Point>::GetMonitorInfo3() { return this->monitor_info3_; }
template <typename T_Point>
void GeneralParser<T_Point>::LoadCorrectionFile(std::string correction_path) {
  std::ifstream fin(correction_path);
  if (fin.is_open()) {
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = static_cast<int>(fin.tellg());
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    int ret = LoadCorrectionString(buffer);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local correction file Error");
    } else {
      LogInfo("Parser correction file success!");
    }
  } else {
    LogError("Open correction file failed");
    return;
  }
}
template <typename T_Point>
int GeneralParser<T_Point>::LoadCorrectionString(char *correction_content) {
  std::string correction_content_str = correction_content;
  std::istringstream ifs(correction_content_str);
  std::string line;

  // skip first line "Laser id,Elevation,Azimuth" or "eeff"
  std::getline(ifs, line);  
  float elevation_list[MAX_LASER_NUM], azimuth_list[MAX_LASER_NUM];
  std::vector<std::string> vfirstLine;
  split_string(vfirstLine, line, ',');
  if (vfirstLine[0] == "EEFF" || vfirstLine[0] == "eeff") {
    // skip second line
    std::getline(ifs, line);  
  }

  int lineCount = 0;
  while (std::getline(ifs, line)) {
    std::vector<std::string> vLineSplit;
    split_string(vLineSplit, line, ',');
    // skip error line or hash value line
    if (vLineSplit.size() < 3) {  
      continue;
    } else {
      lineCount++;
    }
    float elevation, azimuth;
    int laserId = 0;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> laserId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elevation;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (laserId > MAX_LASER_NUM || laserId <= 0) {
      LogFatal("laser id is wrong in correction file. laser Id: %d, line: %d", laserId, lineCount);
      continue;
    }
    if (laserId != lineCount) {
      LogWarning("laser id is wrong in correction file. laser Id: %d, line: %d.  continue", laserId, lineCount);
      continue;
    }
    elevation_list[laserId - 1] = elevation;
    azimuth_list[laserId - 1] = azimuth;
  }
  this->elevation_correction_.resize(lineCount);
  this->azimuth_collection_.resize(lineCount);

  for (int i = 0; i < lineCount; ++i) {
    this->elevation_correction_[i] = elevation_list[i];
    this->azimuth_collection_[i] = azimuth_list[i];
  }
  this->get_correction_file_ = true;
  return 0;
}

template <typename T_Point>
void GeneralParser<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  std::ifstream inFile(firetimes_path, std::ios::in);
  if (inFile.is_open()) {
    std::string lineStr;
    //skip first line
    std::getline(inFile, lineStr); 
    int lineCount = 0;
    while (getline(inFile, lineStr)) {
      std::vector<std::string> vLineSplit;
      split_string(vLineSplit, lineStr, ',');
      // skip error line or hash value line
      if (vLineSplit.size() < 2) {  
        continue;
      } else {
        lineCount++;
      }
      float deltTime;
      int laserId = 0;
      std::stringstream ss(lineStr);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> laserId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> deltTime;
      if (laserId > MAX_LASER_NUM || laserId <= 0) {
        LogFatal("laser id is wrong in firetime file. laser Id: %d, line: %d", laserId, lineCount);
        continue;
      }
      firetime_correction_[laserId - 1] = deltTime;
    }
    this->get_firetime_file_ = true;
    LogInfo("Open firetime file success!");
    inFile.close();
    return;
  } else {
    LogWarning("Open firetime file failed");
    this->get_firetime_file_ = false;
    return;
  }
}

template <typename T_Point>
void GeneralParser<T_Point>::EnablePacketLossTool(bool enable) {
  this->enable_packet_loss_tool_ = enable;
}

template <typename T_Point>
void GeneralParser<T_Point>::EnablePacketTimeLossTool(bool enable) {
  this->enable_packet_timeloss_tool_ = enable;
}

template <typename T_Point>
void GeneralParser<T_Point>::PacketTimeLossToolContinue(bool enable) {
  this->packet_timeloss_tool_continue_ = enable;
}

template <typename T_Point>
void GeneralParser<T_Point>::SetLidarType(std::string type) {
  this->lidar_type_ = type;
}

template <typename T_Point>
int GeneralParser<T_Point>::LoadFiretimesString(char *correction_string) {
  (void)correction_string;
  LogInfo("don't load firetimes string");
  return 0;
}

template <typename T_Point>
double GeneralParser<T_Point>::GetFiretimesCorrection(int laserId, double speed) {
  return this->firetime_correction_[laserId] * speed * 6E-6;
}
template <typename T_Point>
void GeneralParser<T_Point>::GetDistanceCorrection(LidarOpticalCenter optical_center, int &azimuth, int &elevation, float &distance, DistanceCorrectionType type) {
  if (distance <= 0.09) return;
  azimuth = (azimuth + CIRCLE) % CIRCLE;
  elevation = (elevation + CIRCLE) % CIRCLE;
  float tx = this->cos_all_angle_[elevation] * this->sin_all_angle_[azimuth];
  float ty = this->cos_all_angle_[elevation] * this->cos_all_angle_[azimuth];
  float tz = this->sin_all_angle_[elevation];
  float d  = distance;
  if (type == GeometricCenter) {
    float B = 2 * tx * optical_center.x + 2 * ty * optical_center.y + 2 * tz * optical_center.z;
    float C = optical_center.x * optical_center.x + optical_center.y * optical_center.y + optical_center.z * optical_center.z - d * d;
    float d_opitcal = std::sqrt(B * B / 4 - C) - B / 2;
    float x = d_opitcal * tx + optical_center.x;
    float y = d_opitcal * ty + optical_center.y;
    float z = d_opitcal * tz + optical_center.z;
    azimuth = int(std::atan(x / y) * kHalfCircleFloat * kFineResolutionFloat / M_PI + CIRCLE) % CIRCLE;
    elevation = int(std::asin(z / d) * kHalfCircleFloat * kFineResolutionFloat / M_PI + CIRCLE) % CIRCLE;
    distance = d;
  } else if (type == OpticalCenter) {
    float x = d * tx + optical_center.x;
    float y = d * ty + optical_center.y;
    float z = d * tz + optical_center.z;
    float d_geometric_center = std::sqrt(x * x + y * y + z * z);
    azimuth = int(std::atan(x / y) * kHalfCircleFloat * kFineResolutionFloat / M_PI + CIRCLE) % CIRCLE;
    elevation = int(std::asin(z / d_geometric_center) * kHalfCircleFloat * kFineResolutionFloat / M_PI + CIRCLE) % CIRCLE;
    distance = d_geometric_center;
  } else {
    // It should never have been executed here.
  }
}

template <typename T_Point>
void GeneralParser<T_Point>::TransformPoint(float& x, float& y, float& z)
{
  // Eigen::AngleAxisd current_rotation_x(transform_.roll, Eigen::Vector3d::UnitX());
  // Eigen::AngleAxisd current_rotation_y(transform_.pitch, Eigen::Vector3d::UnitY());
  // Eigen::AngleAxisd current_rotation_z(transform_.yaw, Eigen::Vector3d::UnitZ());
  // Eigen::Translation3d current_translation(transform_.x, transform_.y,
  //                                          transform_.z);
  // Eigen::Matrix4d trans = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
  // Eigen::Vector4d target_ori(x, y, z, 1);
  // Eigen::Vector4d target_rotate = trans * target_ori;
  // x = target_rotate(0);
  // y = target_rotate(1);
  // z = target_rotate(2);

  float cosa = std::cos(transform_.roll);
  float sina = std::sin(transform_.roll);
  float cosb = std::cos(transform_.pitch);
  float sinb = std::sin(transform_.pitch);
  float cosc = std::cos(transform_.yaw);
  float sinc = std::sin(transform_.yaw);

  float x_ = cosb * cosc * x + (sina * sinb * cosc - cosa * sinc) * y +
              (sina * sinc + cosa * sinb * cosc) * z + transform_.x;
  float y_ = cosb * sinc * x + (cosa * cosc + sina * sinb * sinc) * y +
              (cosa * sinb * sinc - sina * cosc) * z + transform_.y;
  float z_ = -sinb * x + sina * cosb * y + cosa * cosb * z + transform_.z;
  x = x_;
  y = y_;
  z = z_; 
}

template <typename T_Point>
void GeneralParser<T_Point>::SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw) {
  transform_.x = x;
  transform_.y = y;
  transform_.z = z;
  transform_.roll = roll;
  transform_.yaw = yaw;
  transform_.pitch = pitch;
}

template <typename T_Point>
  void GeneralParser<T_Point>::CalPktLoss(uint32_t PacketSeqnum) {
    if (this->enable_packet_loss_tool_ == false) {
      return;
    }
    if (PacketSeqnum > this->seqnum_loss_message_.last_seqnum && this->seqnum_loss_message_.last_seqnum != 0) {
      this->total_packet_count_ += PacketSeqnum - this->seqnum_loss_message_.last_seqnum;
    }
    this->seqnum_loss_message_.is_packet_loss = false;
    if (this->seqnum_loss_message_.start_seqnum == 0) {
      this->seqnum_loss_message_.loss_count = 0;
      this->seqnum_loss_message_.total_loss_count = 0;
      this->seqnum_loss_message_.start_time = GetMicroTickCount();
      this->seqnum_loss_message_.start_seqnum = PacketSeqnum;
      this->seqnum_loss_message_.last_seqnum = PacketSeqnum;
      this->seqnum_loss_message_.total_start_seqnum = PacketSeqnum;
      return;
    }
    if (PacketSeqnum - this->seqnum_loss_message_.last_seqnum > 1) {
      this->seqnum_loss_message_.loss_count += (PacketSeqnum - this->seqnum_loss_message_.last_seqnum - 1);
      this->seqnum_loss_message_.total_loss_count += (PacketSeqnum - this->seqnum_loss_message_.last_seqnum - 1);
      this->seqnum_loss_message_.is_packet_loss = true;
    }
    // print log every 1s
    if (this->seqnum_loss_message_.loss_count != 0 && GetMicroTickCount() - this->seqnum_loss_message_.start_time >= 1 * 1000 * 1000) {
      LogWarning("pkt loss freq: %u/%u", this->seqnum_loss_message_.loss_count,
             PacketSeqnum - this->seqnum_loss_message_.start_seqnum);
      this->seqnum_loss_message_.loss_count = 0;
      this->seqnum_loss_message_.start_time = GetMicroTickCount();
      this->seqnum_loss_message_.start_seqnum = PacketSeqnum;
    }
    this->seqnum_loss_message_.last_seqnum = PacketSeqnum;
  }

template <typename T_Point>
void GeneralParser<T_Point>::CalPktTimeLoss(uint64_t PacketTimestamp) {
  if(this->enable_packet_timeloss_tool_ == false){
    return;
  } 
  if(this->packet_timeloss_tool_continue_ == false && this->time_loss_message_.total_timeloss_count != 0){    
    return;
  }
  if (this->time_loss_message_.start_timestamp == 0) {
    this->time_loss_message_.timeloss_count = 0;
    this->time_loss_message_.total_timeloss_count = 0;
    this->time_loss_message_.timeloss_start_time = GetMicroTickCount();
    this->time_loss_message_.start_timestamp = PacketTimestamp;
    this->time_loss_message_.last_timestamp = PacketTimestamp;
    this->time_loss_message_.total_start_timestamp = PacketTimestamp;
    return;
  }
  // packet time loss reset
  else if(this->seqnum_loss_message_.is_packet_loss){
    LogWarning("pkt time loss freq: %u/%u", this->time_loss_message_.timeloss_count, this->total_packet_count_ - this->time_loss_message_.last_total_package_count);
    this->time_loss_message_.timeloss_count = 0;
    this->time_loss_message_.timeloss_start_time = GetMicroTickCount();
    this->time_loss_message_.start_timestamp = PacketTimestamp;
    this->time_loss_message_.last_timestamp = PacketTimestamp;
    this->time_loss_message_.last_total_package_count = this->total_packet_count_;
    return;
  }
  if (PacketTimestamp <= this->time_loss_message_.last_timestamp) {
    this->time_loss_message_.timeloss_count++;
    this->time_loss_message_.total_timeloss_count++;
  }
  // print log every 1s
  if (this->time_loss_message_.timeloss_count != 0 && GetMicroTickCount() - this->time_loss_message_.timeloss_start_time >= 1 * 1000 * 1000) {
    LogWarning("pkt time loss freq: %u/%u", this->time_loss_message_.timeloss_count,
           this->total_packet_count_ - this->time_loss_message_.last_total_package_count);
    this->time_loss_message_.timeloss_count = 0;
    this->time_loss_message_.timeloss_start_time = GetMicroTickCount();
    this->time_loss_message_.start_timestamp = PacketTimestamp;
    this->time_loss_message_.last_total_package_count = this->total_packet_count_;
  }
  this->time_loss_message_.last_timestamp = PacketTimestamp;    
}

template <typename T_Point>
void GeneralParser<T_Point>::SetOpticalCenterFlag(bool flag) { 
  optical_center.flag = flag; 
  if (flag) LogInfo("Enable distance correction, center(%f %f %f)", optical_center.x, optical_center.y, optical_center.z);
}

template <typename T_Point>
uint32_t GeneralParser<T_Point>::CRCCalc(const uint8_t *bytes, int len, int zeros_num) {
  CRCInit();
    uint32_t i_crc = 0xffffffff;
    for (int i = 0; i < len; i++)
        i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ bytes[i]) & 0xff];
    for (int i = 0; i < zeros_num; i++)
        i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ 0) & 0xff];
    return i_crc;
}

template <typename T_Point>
void GeneralParser<T_Point>::CRCInit() {
    static bool initialized = false;
    if (initialized) {
        return;
    }
    initialized = true;

    uint32_t i, j, k;
    for (i = 0; i < 256; i++) {
        k = 0;
        for (j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1)
            k = (k << 1) ^ (((k ^ j) & 0x80000000) ? 0x04c11db7 : 0);
        m_CRCTable[i] = k;
    }
}