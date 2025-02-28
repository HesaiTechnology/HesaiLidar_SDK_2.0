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
 * File:       udp1_8_parser.cc
 * Author:     Chang XingShuo <changxingshuo@hesaitech.com>
 * Description: Implemente Udp1_8Parser class
*/

#include "udp1_8_parser.h"
#include "udp_protocol_v1_8.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp1_8Parser<T_Point>::Udp1_8Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  this->optical_center.setNoFlag(LidarOpticalCenter{-0.00625, 0.010955, 0.003911});
}

template<typename T_Point>
Udp1_8Parser<T_Point>::~Udp1_8Parser() { LogInfo("release general parser"); }

template<typename T_Point>
int Udp1_8Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  for (int blockid = 0; blockid < frame.block_num; blockid++) {
    // T_Point point;
    int elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < frame.laser_num; i++) {
      int point_index = packet_index * frame.per_points_num + blockid * frame.laser_num + i;
      int Azimuth = int(frame.pointData[point_index].azimuth * kFineResolutionFloat);
      float distance = static_cast<float>(frame.pointData[point_index].distances * frame.distance_unit); 
      if (this->get_correction_file_) {
        int azimuth_coll = (int(this->azimuth_collection_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        int elevation_corr = (int(this->elevation_correction_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        if (this->optical_center.flag) {
          azimuth_coll -= 190.0 * kAllFineResolutionFloat;
          GeneralParser<T_Point>::GetDistanceCorrection(this->optical_center, azimuth_coll, elevation_corr, distance, OpticalCenter);
          azimuth_coll += 190.0 * kAllFineResolutionFloat;
        }
        elevation = elevation_corr;
        azimuth = Azimuth + azimuth_coll;
        azimuth = (CIRCLE + azimuth) % CIRCLE;
      } 
      if (frame.config.fov_start != -1 && frame.config.fov_end != -1)
      {
        int fov_transfer = azimuth / 256 / 100;
        if (fov_transfer < frame.config.fov_start || fov_transfer > frame.config.fov_end){//不在fov范围continue
          memset(&frame.points[point_index], 0, sizeof(T_Point));
          continue;
        }
      } 

      float xyDistance = distance * this->cos_all_angle_[(elevation)];
      float x = xyDistance * this->sin_all_angle_[(azimuth)];
      float y = xyDistance * this->cos_all_angle_[(azimuth)];
      float z = distance * this->sin_all_angle_[(elevation)];

      this->TransformPoint(x, y, z);
      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], frame.pointData[point_index].reflectivities);
      setTimestamp(frame.points[point_index], double(frame.sensor_timestamp[packet_index]) / kMicrosecondToSecond);
      setRing(frame.points[point_index], static_cast<uint16_t>(i));
    }
  }
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
int Udp1_8Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF) return -1;
  const HS_LIDAR_PRE_HEADER_JT *pPreHeader =
      reinterpret_cast<const HS_LIDAR_PRE_HEADER_JT *>(udpPacket.buffer);

  const HS_LIDAR_HEADER_JT *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_JT *>(
          udpPacket.buffer + sizeof(HS_LIDAR_PRE_HEADER_JT));
  int zeros_num = (udpPacket.packet_len % 4);
  zeros_num = zeros_num == 0 ? 0 : (4 - zeros_num);
  uint32_t ret = this->CRCCalc(udpPacket.buffer, udpPacket.packet_len - sizeof(uint32_t), zeros_num);
  // point to azimuth of udp start block
  if (pPreHeader->GetDataType() == 1) {
    const HS_LIDAR_BODY_IMU_JT *pBody =
      reinterpret_cast<const HS_LIDAR_BODY_IMU_JT *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_JT));
    const HS_LIDAR_TAIL_JT *pTail = reinterpret_cast<const HS_LIDAR_TAIL_JT *>(
          (const unsigned char *)pBody + sizeof(HS_LIDAR_BODY_IMU_JT));
    if (ret != pTail->GetCrc()) {
        return -1;
    }
    if (frame.use_timestamp_type == 0) {
      frame.imu_config.timestamp = double(pHeader->GetMicroLidarTimeU64()) / kMicrosecondToSecond;
    } else {
      frame.imu_config.timestamp = double(udpPacket.recv_timestamp) / kMicrosecondToSecond;
    }  
    frame.imu_config.imu_accel_x = pBody->GetIMUXAccel();
    frame.imu_config.imu_accel_y = pBody->GetIMUYAccel();
    frame.imu_config.imu_accel_z = pBody->GetIMUZAccel();
    frame.imu_config.imu_ang_vel_x = pBody->GetIMUXAngVel();
    frame.imu_config.imu_ang_vel_y = pBody->GetIMUYAngVel();
    frame.imu_config.imu_ang_vel_z = pBody->GetIMUZAngVel();
    return 1;
  }
  if (pPreHeader->GetDataType() != 0) return -1; 
  const HS_LIDAR_BODY_POINT_JT *pBody =
      reinterpret_cast<const HS_LIDAR_BODY_POINT_JT *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_JT) +
            sizeof(HS_LIDAR_BODY_AZIMUTH_JT) +
           sizeof(HS_LIDAR_BODY_CHN_UNIT_JT) * kLaserNum);
  const HS_LIDAR_BODY_AZIMUTH_JT *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_JT *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_JT));
  const HS_LIDAR_BODY_CHN_UNIT_JT *pChnUnit =
          reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_JT *>(
              (const unsigned char *)pAzimuth +
              sizeof(HS_LIDAR_BODY_AZIMUTH_JT));
  // uint32_t packet_seqnum = pBody->GetSequenceNum();
  // this->CalPktLoss(packet_seqnum);
  const HS_LIDAR_TAIL_JT *pTail = reinterpret_cast<const HS_LIDAR_TAIL_JT *>(
          (const unsigned char *)pBody + sizeof(HS_LIDAR_BODY_POINT_JT));
  if (ret != pTail->GetCrc()) {
      return -1;
  }
  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pHeader->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }   
  uint64_t packet_timestamp = pHeader->GetMicroLidarTimeU64();
  this->CalPktTimeLoss(packet_timestamp);
  frame.host_timestamp = GetMicroTickCountU64();
  frame.lidar_state = pBody->GetLidarState();
  frame.per_points_num = kLaserNum;
  frame.scan_complete = false;
  frame.distance_unit = 4.0 / 1000.0;
  frame.block_num = 1;
  frame.laser_num = kLaserNum;

  int index = frame.packet_num * 1 * kLaserNum;
  uint16_t u16Azimuth = pAzimuth->GetAzimuth();
  for (int i = 0; i < kLaserNum; i++) {
    frame.pointData[index].azimuth = u16Azimuth;
    frame.pointData[index].reflectivities = pChnUnit->GetReflectivity();  
    frame.pointData[index].distances = pChnUnit->GetDistance();
    pChnUnit = pChnUnit + 1;
    index++;
  }
  
  if (IsNeedFrameSplit(u16Azimuth)) {
    frame.scan_complete = true;
  }
  if (u16Azimuth != this->last_azimuth_) {
    this->last_last_azimuth_ = this->last_azimuth_;
    this->last_azimuth_ = u16Azimuth;
  }
  frame.packet_num++;
  return 0;
} 

template<typename T_Point>
bool Udp1_8Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  // Determine frame_start_azimuth_ [0,360)
  if (this->frame_start_azimuth_ < 0.0f || this->frame_start_azimuth_ >= 360.0f) {
    this->frame_start_azimuth_ = 0.0f;
  }
  // The first two packet dont have the information of last_azimuth_  and last_last_azimuth, so do not need split frame
  // The initial value of last_azimuth_ is -1
  // Determine the rotation direction and division
  
  int32_t division = 0;
  // If last_last_azimuth_ != -1，the packet is the third, so we can determine whether the current packet requires framing
  if (this->last_last_azimuth_ != -1) 
  {
    // Get the division
    int32_t division1 = abs(this->last_azimuth_ - this->last_last_azimuth_);
    int32_t division2 = abs(this->last_azimuth_ - azimuth);
    division = division1 > division2 ? division2 : division1 ;
    // Prevent two consecutive packets from having the same angle when causing an error in framing
    if ( division == 0) return false;
    // In the three consecutive angle values, if the angle values appear by the division of the decreasing situation,it must be reversed
    // The same is true for FOV
    if( this->last_last_azimuth_ - this->last_azimuth_ == division || this->last_azimuth_ -azimuth == division)
    {
      this->rotation_flag = -1;
    } else {
      this->rotation_flag = 1;
    }
  } else {
    // The first  and second packet do not need split frame
    return false;
  }
  if (this->rotation_flag == 1) {
    // When an angle jump occurs
    if (this->last_azimuth_- azimuth > division)
    {
      if (uint16_t(this->frame_start_azimuth_ * kResolutionInt) > this->last_azimuth_ || uint16_t(this->frame_start_azimuth_ * kResolutionInt <= azimuth)) {
        return true;
      } 
      return false;
    }  
    if (this->last_azimuth_ < azimuth && this->last_azimuth_ < uint16_t(this->frame_start_azimuth_ * kResolutionInt) 
        && azimuth >= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) {
      return true;
    }
    return false;
  } else {
    if (azimuth - this->last_azimuth_ > division)
    {
      if (uint16_t(this->frame_start_azimuth_ * kResolutionInt) <= this->last_azimuth_ || uint16_t(this->frame_start_azimuth_ * kResolutionInt) > azimuth) {
        return true;
      } 
      return false;
    }  
    if (this->last_azimuth_ > azimuth && this->last_azimuth_ > uint16_t(this->frame_start_azimuth_ * kResolutionInt) 
        && azimuth <= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) {
      return true;
    }
    return false;
  }
}

template <typename T_Point>
void Udp1_8Parser<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
  int type = 0;
  size_t length = lidar_correction_file.length();
  if (length >= 4) {
    std::string extension = lidar_correction_file.substr(length - 4);
    if (extension == ".bin" || extension == ".dat") {
        type = 1; //  .bin
    } else if (extension == ".csv") {
        type = 2; //  .csv
    } else {
        // type = 0; //  wrong
        return;
    }   
  }

  LogInfo("load correction file from local correction now!");
  std::ifstream fin(lidar_correction_file);
  if (fin.is_open()) {
    LogDebug("Open correction file success!");
    fin.seekg(0, std::ios::end);
    int len = static_cast<int>(fin.tellg());
    // return the begin of file
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[len];
    // file --> buffer
    fin.read(buffer, len);
    fin.close();
    int ret = 0;
    if (type == 1) {
      ret = LoadCorrectionString(buffer);
    } else if(type == 2) {
      ret = LoadCorrectionCsvData(buffer);
    } else {
      LogWarning("Invalid suffix name");
      ret = -1;
    }
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local Correction file Error!");
    } else {
      LogInfo("Parse local Correction file Success!!!");
    }
  } else { // open failed
    LogError("Open correction file failed");
    return;
  }
}
template <typename T_Point>
int Udp1_8Parser<T_Point>::LoadCorrectionCsvData(char *correction_content) {
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
int Udp1_8Parser<T_Point>::LoadCorrectionString(char *correction_content) {
  int16_t *ptr = (int16_t *)correction_content;
  this->azimuth_collection_.resize(16);
  this->elevation_correction_.resize(16);
  for (int i = 0; i < 16; i++) {
    this->azimuth_collection_[i] = *ptr * 0.01;
    ptr++;
  }
  for (int i = 0; i < 16; i++) {
    this->elevation_correction_[i] = *ptr * 0.01;
    ptr++;
  }
  LogInfo("load correction string success");
  this->get_correction_file_ = true;
  return 0;
}