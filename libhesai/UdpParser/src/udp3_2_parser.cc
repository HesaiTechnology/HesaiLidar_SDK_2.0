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
 * File:       udp3_2_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp3_2Parser class
*/

#include "udp3_2_parser.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp3_2Parser<T_Point>::Udp3_2Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  this->optical_center.setNoFlag(LidarOpticalCenter{0.0072, 0.0354, 0});
}

template<typename T_Point>
Udp3_2Parser<T_Point>::~Udp3_2Parser() { LogInfo("release general parser"); }

template<typename T_Point>
int Udp3_2Parser<T_Point>::LoadFiretimesString(char *firetimes_string) {
  std::string firetimes_content_str = firetimes_string;
  std::istringstream fin(firetimes_content_str);
  std::string line;
  // first line sequence,chn id,firetime/us
  if (std::getline(fin, line)) {  
    LogInfo("Parse Lidar firetime now...");
  }
  std::vector<std::string> firstLine;
  split_string(firstLine, line, ',');
  if (firstLine[0] == "EEFF" || firstLine[0] == "eeff") {
    std::array<std::array<float, HS_LIDAR_QT128_LASER_NUM>,
               HS_LIDAR_QT128_LOOP_NUM>
        firetimes;
    firetimes[0].fill(0);
    firetimes[1].fill(0);
    firetimes[2].fill(0);
    firetimes[3].fill(0);
    std::getline(fin, line);
    std::vector<std::string> loopNumLine;
    split_string(loopNumLine, line, ',');
    int loopNum = atoi(loopNumLine[3].c_str());
    if (loopNum > HS_LIDAR_QT128_LOOP_NUM) {
      LogError("firetimes loop num is error : %d", loopNum);
      return -1;
    }
    std::getline(fin, line);
    for (int i = 0; i < HS_LIDAR_QT128_LASER_NUM; i++) {
      std::getline(fin, line);
      std::vector<std::string> ChannelLine;
      split_string(ChannelLine, line, ',');
      for (int j = 0; j < loopNum; j++) {
        if ((int)ChannelLine.size() == loopNum * 2) {
          int laserId = atoi(ChannelLine[j * 2].c_str()) - 1;
          if (laserId >= 0 && laserId < HS_LIDAR_QT128_LASER_NUM)
            firetimes[j][laserId] = std::stof(ChannelLine[j * 2 + 1].c_str());
          else if (std::stof(ChannelLine[j * 2 + 1].c_str()) != 0)
            LogWarning("firetimes laserId is invalid : %d", laserId);
        } else {
          LogError("loop num is not equal to the first channel line");
          return -1;
        }
      }
    }
    qt128_firetime_ = firetimes;
    this->get_firetime_file_ = true;
  } else {
    LogError("firetime file delimiter is wrong");
    return -1;
  }
  return 0;
}
template<typename T_Point>
void Udp3_2Parser<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  std::ifstream fin(firetimes_path);
  if (fin.is_open()) {
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = static_cast<int>(fin.tellg());
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    int ret = LoadFiretimesString(buffer);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local firetimes file Error");
    }
  } else {
    LogError("Open firetimes file failed");
    return;
  }
}
template<typename T_Point>
int Udp3_2Parser<T_Point>::LoadChannelConfigString(char *channel_config_content) {
  std::string channel_config_content_str = channel_config_content;
  pandarQT_channel_config_.m_bIsChannelConfigObtained = false;
  std::istringstream ifs(channel_config_content_str);
  std::string line;

  std::getline(ifs, line);
  std::vector<std::string> versionLine;
  split_string(versionLine, line, ',');
  if (versionLine[0] == "EEFF" || versionLine[0] == "eeff") {
    pandarQT_channel_config_.major_version =
        std::stoi(versionLine[1].c_str());
    pandarQT_channel_config_.min_version = std::stoi(versionLine[2].c_str());
  } else {
    LogError("channel config file delimiter is wrong");
    return -1;
  }
  std::getline(ifs, line);
  std::vector<std::string> channelNumLine;
  split_string(channelNumLine, line, ',');
  pandarQT_channel_config_.laser_num = std::stoi(channelNumLine[1].c_str());
  pandarQT_channel_config_.m_u8BlockNum = std::stoi(channelNumLine[3].c_str());
  if (pandarQT_channel_config_.laser_num <= 0 ||
      pandarQT_channel_config_.m_u8BlockNum <= 0) {
    LogError("LaserNum: %u BlockNum: %u", pandarQT_channel_config_.laser_num
                , pandarQT_channel_config_.m_u8BlockNum);
    return -1;
  }
  std::getline(ifs, line);
  std::vector<std::string> firstChannelLine;
  split_string(firstChannelLine, line, ',');
  int loop_num = firstChannelLine.size();
  pandarQT_channel_config_.m_vChannelConfigTable.resize(loop_num);

  for (int i = 0; i < loop_num; i++) {
    pandarQT_channel_config_.m_vChannelConfigTable[i].resize(
        pandarQT_channel_config_.laser_num);
  }
  for (int i = 0; i < pandarQT_channel_config_.laser_num; i++) {
    std::getline(ifs, line);
    std::vector<std::string> ChannelLine;
    split_string(ChannelLine, line, ',');
    for (int j = 0; j < loop_num; j++) {
      if (ChannelLine.size() == loop_num) {
        pandarQT_channel_config_.m_vChannelConfigTable[j][i] =
            std::stoi(ChannelLine[j].c_str());
      } else {
        LogError("loop num is not equal to the first channel line");
        return -1;
      }
    }
  }
  std::getline(ifs, line);
  pandarQT_channel_config_.m_sHashValue = line;
  pandarQT_channel_config_.m_bIsChannelConfigObtained = true;
  return 0;
}
template<typename T_Point>
void Udp3_2Parser<T_Point>::LoadChannelConfigFile(std::string channel_config_path) {
  std::ifstream fin(channel_config_path);
  if (fin.is_open()) {
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = static_cast<int>(fin.tellg());
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    int ret = LoadChannelConfigString(buffer);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local channel congfig file Error");
    }
  } else {
    LogError("Open channel congfig file failed");
    return;
  }
}
template<typename T_Point>
double Udp3_2Parser<T_Point>::GetFiretimesCorrection(int laserId, double speed,
                                             int loopIndex) {
  return qt128_firetime_[loopIndex][laserId] * speed * 6E-6;
}

template<typename T_Point>
int Udp3_2Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  for (int blockid = 0; blockid < frame.block_num; blockid++) {
    // T_Point point;
    int elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < frame.laser_num; i++) {
      int point_index = packet_index * frame.per_points_num + blockid * frame.laser_num + i;
      float distance = static_cast<float>(frame.pointData[point_index].distances * frame.distance_unit);
      int Azimuth = int(frame.pointData[point_index].azimuth * kFineResolutionFloat);
      azimuth = Azimuth;
      if (this->get_correction_file_) {
        int azimuth_coll = (int(this->azimuth_collection_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        int elevation_corr = (int(this->elevation_correction_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        if (this->optical_center.flag) {
          GeneralParser<T_Point>::GetDistanceCorrection(this->optical_center, azimuth_coll, elevation_corr, distance, GeometricCenter);
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
bool Udp3_2Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
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
    // When an angle jump occurs, it maybe 359.9-0 or 39.9-40-10.0(consired FOV)
    if (this->last_azimuth_- azimuth > division)
    {
      if (uint16_t(this->frame_start_azimuth_ * kResolutionInt) > this->last_azimuth_ || uint16_t(this->frame_start_azimuth_ * kResolutionInt <= azimuth)) {
        return true;
      } 
      return false;
    }
    // No angle jump occurs, the frame_start_azimuth must betwen last_azimuth and azimuth  
    if (this->last_azimuth_ < azimuth && this->last_azimuth_ < uint16_t(this->frame_start_azimuth_ * kResolutionInt) 
        && azimuth >= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) {
      return true;
    }
    return false;
  } else {
    if (azimuth - this->last_azimuth_ > division)
    {
      if (uint16_t(this->frame_start_azimuth_ * kResolutionInt) <= this->last_azimuth_ || uint16_t(this->frame_start_azimuth_ * kResolutionInt > azimuth)) {
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

template<typename T_Point>
int Udp3_2Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF) return -1;
  const HS_LIDAR_HEADER_QT_V2 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_QT_V2 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  frame.per_points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.scan_complete = false;
  frame.block_num = pHeader->GetBlockNum();
  frame.laser_num = pHeader->GetLaserNum();  
  frame.distance_unit = pHeader->GetDistUnit();      
  int index = frame.packet_num * pHeader->GetBlockNum() * pHeader->GetLaserNum();

  if (pHeader->HasConfidenceLevel()) {
    const HS_LIDAR_BODY_AZIMUTH_QT_V2 *pAzimuth =
        reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2));
    
    const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *pChnUnit =
        reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *>(
            (const unsigned char *)pAzimuth +
            sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));

    const HS_LIDAR_TAIL_QT_V2 *pTail =
        reinterpret_cast<const HS_LIDAR_TAIL_QT_V2 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) +
            (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
             sizeof(HS_LIDAR_BODY_CHN_UNIT_QT_V2) * pHeader->GetLaserNum()) *
                pHeader->GetBlockNum() +
            sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
            (pHeader->HasFunctionSafety() ? sizeof(HS_LIDAR_FUNCTION_SAFETY)
                                          : 0));
    if (pHeader->HasSeqNum()) {
      const HS_LIDAR_TAIL_SEQ_NUM_QT_V2 *pTailSeqNum =
        reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_QT_V2 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) +
            (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
             sizeof(HS_LIDAR_BODY_CHN_UNIT_QT_V2) * pHeader->GetLaserNum()) *
                pHeader->GetBlockNum() +
            sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
            (pHeader->HasFunctionSafety() ? sizeof(HS_LIDAR_FUNCTION_SAFETY)
                                          : 0) +
                                          sizeof(HS_LIDAR_TAIL_QT_V2));
      uint32_t packet_seqnum = pTailSeqNum->m_u32SeqNum;
      this->CalPktLoss(packet_seqnum);
    }
    uint64_t packet_timestamp = pTail->GetMicroLidarTimeU64();
    this->CalPktTimeLoss(packet_timestamp);  
    frame.host_timestamp = GetMicroTickCountU64();              
    if (frame.use_timestamp_type == 0) {
      frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
    } else {
      frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
    }                    
    this->spin_speed_ = pTail->GetMotorSpeed();
    frame.spin_speed = pTail->GetMotorSpeed();
    frame.return_mode = pTail->GetReturnMode();
    uint16_t u16Azimuth = 0;
    for (size_t i = 0; i < pHeader->GetBlockNum(); i++) {
      // point to channel unit addr
      u16Azimuth = pAzimuth->GetAzimuth();
      // auto elevation = 0;
      pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));

      // point to next block azimuth addr
      pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_QT_V2) * pHeader->GetLaserNum());
      int loopIndex = (pTail->GetModeFlag() +
                    (i / ((pTail->GetReturnMode() < 0x39) ? 1 : 2)) + 1) % 2;
      for (int j = 0; j < pHeader->GetLaserNum(); j++) {
        int laserId = (pHeader->HasSelfDefine() &&
               pandarQT_channel_config_.m_bIsChannelConfigObtained &&
               i < pandarQT_channel_config_.m_vChannelConfigTable[loopIndex].size())
                  ? pandarQT_channel_config_.m_vChannelConfigTable[loopIndex][j] - 1
                  : j;
        if (this->get_firetime_file_) {
          frame.pointData[index].azimuth = u16Azimuth + this->rotation_flag * GetFiretimesCorrection(
                                    laserId, pTail->GetMotorSpeed(), loopIndex) * kResolutionFloat;
        } else {
          frame.pointData[index].azimuth = u16Azimuth;
        }
        frame.pointData[index].distances = pChnUnit->GetDistance();
        frame.pointData[index].reflectivities = pChnUnit->GetReflectivity();
        frame.pointData[index].confidence = pChnUnit->GetConfidenceLevel();
        pChnUnit = pChnUnit + 1;  
        index++;  
      }
    }
    if (IsNeedFrameSplit(u16Azimuth)) {
      frame.scan_complete = true;
    }
    if (u16Azimuth != this->last_azimuth_) {
      this->last_last_azimuth_ = this->last_azimuth_;
      this->last_azimuth_ = u16Azimuth; 
    } 
  } else {
    const HS_LIDAR_BODY_AZIMUTH_QT_V2 *pAzimuth =
        reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2));       

    const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2 *pChnUnit =
        reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2 *>(
            (const unsigned char *)pAzimuth +
            sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));

    const HS_LIDAR_TAIL_QT_V2 *pTail =
        reinterpret_cast<const HS_LIDAR_TAIL_QT_V2 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) +
                    (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
                     sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2) *
                         pHeader->GetLaserNum()) *
                        pHeader->GetBlockNum() +
                    sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
                    (pHeader->HasFunctionSafety()
                ? sizeof(HS_LIDAR_FUNCTION_SAFETY)
                : 0));
    if (frame.use_timestamp_type == 0) {
      frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
    } else {
      frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
    }
    this->spin_speed_ = pTail->GetMotorSpeed();
    frame.spin_speed = pTail->GetMotorSpeed();
    frame.return_mode = pTail->GetReturnMode();
    uint16_t u16Azimuth = 0;
    for (size_t i = 0; i < pHeader->GetBlockNum(); i++) {
      // point to channel unit addr
      pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));
      u16Azimuth = pAzimuth->GetAzimuth(); 
      // point to next block azimuth addr
      pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2) *
              pHeader->GetLaserNum());
      int loopIndex = (pTail->GetModeFlag() +
                    (i / ((pTail->GetReturnMode() < 0x39) ? 1 : 2)) + 1) % 2;
      for (int j = 0; j < pHeader->GetLaserNum(); j++) {
        int laserId =(pHeader->HasSelfDefine() &&
               pandarQT_channel_config_.m_bIsChannelConfigObtained &&
               i < pandarQT_channel_config_.m_vChannelConfigTable[loopIndex].size())
                  ? pandarQT_channel_config_.m_vChannelConfigTable[loopIndex][j] - 1
                  : j;
        if (this->get_firetime_file_) {
          frame.pointData[index].azimuth = u16Azimuth + this->rotation_flag * GetFiretimesCorrection(
                                    laserId, pTail->GetMotorSpeed(), loopIndex) * kResolutionFloat;
        } else {
          frame.pointData[index].azimuth = u16Azimuth;
        }
        frame.pointData[index].distances = pChnUnit->GetDistance();
        frame.pointData[index].reflectivities = pChnUnit->GetReflectivity(); 
        pChnUnit = pChnUnit + 1;  
        index++;
      }
    }
    if (IsNeedFrameSplit(u16Azimuth)) {
      frame.scan_complete = true;
    }
    if (u16Azimuth != this->last_azimuth_) {
      this->last_last_azimuth_ = this->last_azimuth_;
      this->last_azimuth_ = u16Azimuth;  
    }
  }
  frame.packet_num++;
  return 0;
}