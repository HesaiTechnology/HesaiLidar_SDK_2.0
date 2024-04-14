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
// #include <Eigen/Dense>
// #include<Eigen/Core>
using namespace hesai::lidar;
template <typename T_Point>
GeneralParser<T_Point>::GeneralParser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  this->enable_update_monitor_info_ = false;
  this->start_seqnum_ = 0;
  this->last_seqnum_ = 0;
  this->loss_count_ = 0;
  this->start_time_ = 0;
  this->last_azimuth_ = 0;
  this->last_last_azimuth_ = 0;
  this->total_packet_count_ = 0;
  this->enable_packet_loss_tool_ = false;
  for (int i = 0; i < CIRCLE; ++i) {
    this->sin_all_angle_[i] = std::sin(i * 2 * M_PI / CIRCLE);
    this->cos_all_angle_[i] = std::cos(i * 2 * M_PI / CIRCLE);
  }
}


template <typename T_Point>
int GeneralParser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  return 0;
}
template <typename T_Point>
int GeneralParser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  return 0;
}  
template <typename T_Point>
int GeneralParser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  return 0;
} 

template <typename T_Point>
GeneralParser<T_Point>::~GeneralParser() { printf("release general Parser\n"); }

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
  int ret = 0;
  std::ifstream fin(correction_path);
  if (fin.is_open()) {
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    ret = LoadCorrectionString(buffer);
    if (ret != 0) {
      std::cout << "Parse local correction file Error\n";
    } else {
      std::cout << "Parser correction file success!" << std::endl;
    }
  } else {
    std::cout << "Open correction file failed\n";
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

    if (laserId != lineCount || laserId >= MAX_LASER_NUM) {
      std::cout << "laser id is wrong in correction file. laser Id:"
                  << laserId << ", line" << lineCount << std::endl;
      // return -1;
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
    while (getline(inFile, lineStr)) {
      std::stringstream ss(lineStr);
      std::string index, deltTime;
      std::getline(ss, index, ',');
      std::getline(ss, deltTime, ',');
    }
    this->get_firetime_file_ = true;
    std::cout << "Open firetime file success!" << std::endl;
    inFile.close();
    return;
  } else {
    std::cout << "Open firetime file failed" << std::endl;
    this->get_firetime_file_ = false;
    return;
  }
}

template <typename T_Point>
void GeneralParser<T_Point>::EnablePacketLossTool(bool enable) {
  this->enable_packet_loss_tool_ = enable;
}

template <typename T_Point>
void GeneralParser<T_Point>::SetEnableFireTimeCorrection(bool enable) {
  this->enable_firetime_correction_ = enable;
}
template <typename T_Point>
void GeneralParser<T_Point>::SetEnableDistanceCorrection(bool enable) {
  this->enable_distance_correction_ = enable;
}
template <typename T_Point>
int GeneralParser<T_Point>::LoadFiretimesString(char *correction_string) {
  printf("load firetimes string\n");
  return 0;
}

template <typename T_Point>
double GeneralParser<T_Point>::GetFiretimesCorrection(int laserId, double speed) {
  return this->firetime_correction_[laserId] * speed * 6E-6;
}
template <typename T_Point>
void GeneralParser<T_Point>::GetDistanceCorrection(double &azimuth, double &elevation,
                                          double &distance) {
  printf("get distance correction\n");
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
