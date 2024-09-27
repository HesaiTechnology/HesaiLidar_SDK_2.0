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
 * File:       udp1_4_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp1_4Parser class
*/

#include "udp1_4_parser.h"
#include "udp_protocol_v1_4.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp1_4Parser<T_Point>::Udp1_4Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  distance_correction_para_a_ = 1;
  distance_correction_para_b_ = 0.012; 
  distance_correction_para_h_ = 0.04; 
  distance_correction_para_c_ = std::sqrt(distance_correction_para_b_ * distance_correction_para_b_ + distance_correction_para_h_  * distance_correction_para_h_); 
  distance_correction_para_d_ = std::atan(distance_correction_para_b_  / distance_correction_para_h_); 
  use_frame_start_azimuth_ = true;
}

template<typename T_Point>
Udp1_4Parser<T_Point>::~Udp1_4Parser() { LogInfo("release general parser"); }

template<typename T_Point>
void Udp1_4Parser<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  try {
    std::ifstream inFile(firetimes_path, std::ios::in);
    if (inFile.is_open()) {
      int uselessLine = 0;
      int count = 0;
      bool is_OT = false;
      std::string lineStr;
      while (std::getline(inFile, lineStr)) {
        std::stringstream ss(lineStr);
        std::string str;
        std::vector<std::string> strList;
        strList.reserve(20);
        while(std::getline(ss, str, ',')){
          strList.push_back(str);
        }
        if (uselessLine == 0 && strList[0] == "Operational State") {
          is_OT = true;
        } else if(uselessLine == 0) {
          std::string dist;
          for (auto e : strList[0]) {
            if (e == '.' || (e >= '0' && e <= '9')) {
              dist.push_back(e);
            }
          }
          if (dist.size() != 0) {
            section_distance = std::stod(dist);
          } 
        }
        if (is_OT == false) {
          if(uselessLine < 3) {
            uselessLine++;
            continue;
          }
          if (lineStr[lineStr.size() - 1] == '\n') {
            lineStr = lineStr.substr(lineStr.size() - 1);
          }
          if (strList.size() < 17) {
            LogError("invalid firetime input file!(list)");
            this->get_firetime_file_ = false;
            return;
          }
          int idx = std::stoi(strList[0]) - 1;
          if (idx >= kLaserNum || idx < 0) {
            LogFatal("laser id is wrong in correction file. laser Id: %d", idx);
            continue;
          }
          for (int i = 1; i <= 15; i += 2) {
            int a = std::stoi(strList[i]);
            int b = std::stoi(strList[i + 1]);
            firetime_section_values[idx].section_values[i / 2].firetime[0] = a;
            firetime_section_values[idx].section_values[i / 2].firetime[1] = b;
          }
        } else {
          if(uselessLine < 2) {
            uselessLine++;
            continue;
          }
          if (lineStr[lineStr.size() - 1] == '\n') {
            lineStr = lineStr.substr(lineStr.size() - 1);
          }
          if (strList.size() < 7) {
            LogError("invalid firetime input file!(list)");
            this->get_firetime_file_ = false;
            return;
          }
          int idx = std::stoi(strList[0]) - 1;
          if (idx >= kLaserNum || idx < 0) {
            LogFatal("laser id is wrong in correction file. laser Id: %d", idx);
            continue;
          }
          for (int i = 1; i <= 6; i++) {
            float a = 0;
            if (strList[i] != "-") {
              a = std::stof(strList[i]) * 1000;
            }
            firetime_section_values[idx].section_values[i - 1].firetime[0] = a;
            firetime_section_values[idx].section_values[i - 1].firetime[1] = a;
          }
        }
        count++;
        if (count > 128) {
          LogError("invalid firetime input file!(count)");
          this->get_firetime_file_ = false;
          return;
        }
      }
    } else {
      LogWarning("Open firetime file failed");
      this->get_firetime_file_ = false;
      return;
    }
    this->get_firetime_file_ = true;
    LogInfo("Open firetime file success!");
  } catch (const std::exception &e) {
    LogFatal("load firetimes error: %s", e.what());
    this->get_firetime_file_ = false;
  }
  return;
}

template<typename T_Point>
double Udp1_4Parser<T_Point>::GetFiretimesCorrection(int laserId, double speed, uint8_t optMode, uint8_t angleState, float dist){
  int idx = 0;
  switch (optMode)
  {
  case 0:
    idx = angleState;
    break;
  case 2:
    idx = 4 + angleState;
    break;
  case 3:
    idx = 6 + angleState;
    break;

  default:
    return 0.0;
    break;
  }
  int k = (dist >= section_distance)? 0 : 1;
  return firetime_section_values[laserId].section_values[idx].firetime[k] * speed * 6E-9;
}

template<typename T_Point>
int Udp1_4Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  for (int blockid = 0; blockid < frame.block_num; blockid++) {
    // T_Point point;
    int elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < frame.laser_num; i++) {
      int point_index = packet_index * frame.per_points_num + blockid * frame.laser_num + i;
      int Azimuth = frame.pointData[point_index].azimuth * kFineResolutionFloat;
      float distance = frame.pointData[point_index].distances * frame.distance_unit;  
      if (this->get_correction_file_) {
        int azimuth_coll = (int(this->azimuth_collection_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        int elevation_corr = (int(this->elevation_correction_[i] * kAllFineResolutionFloat) + CIRCLE) % CIRCLE;
        if (this->enable_distance_correction_) {
          GetDistanceCorrection(azimuth_coll, elevation_corr, distance, GeometricCenter);
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
      setConfidence(frame.points[point_index], frame.pointData[point_index].confidence);
      setTimestamp(frame.points[point_index], double(frame.sensor_timestamp[packet_index]) / kMicrosecondToSecond);
      setRing(frame.points[point_index], i);
    }
  }
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
bool Udp1_4Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  // Determine frame_start_azimuth_ [0,360)
  if (this->frame_start_azimuth_ < 0.0f || this->frame_start_azimuth_ >= 360.0f) {
    this->frame_start_azimuth_ = 0.0f;
  }
  // The first two packet dont have the information of last_azimuth_  and last_last_azimuth, so do not need split frame
  // The initial value of last_azimuth_ is -1
  // Determine the rotation direction and division
  
  uint16_t division = 0;
  // If last_last_azimuth_ != -1，the packet is the third, so we can determine whether the current packet requires framing
  if (this->last_last_azimuth_ != -1) 
  {
    // Get the division
    uint16_t division1 = abs(this->last_azimuth_ - this->last_last_azimuth_);
    uint16_t division2 = abs(this->last_azimuth_ - azimuth);
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

template<typename T_Point>
int Udp1_4Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
{
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF) return -1;
  const HS_LIDAR_HEADER_ME_V4 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_ME_V4 *>(
          udpPacket.buffer + sizeof(HS_LIDAR_PRE_HEADER));

  // point to azimuth of udp start block
  const HS_LIDAR_BODY_AZIMUTH_ME_V4 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4));
  if(pHeader->HasFuncSafety()) {
    const auto *function_savety_ptr = reinterpret_cast<const HS_LIDAR_FUNC_SAFETY_ME_V4 *>(
      (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) +
      (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + 
      (pHeader->HasConfidenceLevel()
              ? sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4)
              : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4)) *
            pHeader->GetLaserNum()) *
            pHeader->GetBlockNum() +
        sizeof(HS_LIDAR_BODY_CRC_ME_V4)
    );
    frame.lidar_state = function_savety_ptr->GetLidarState();
  } else {
    frame.lidar_state = -1;
  }

  const auto *pTail = reinterpret_cast<const HS_LIDAR_TAIL_ME_V4 *>(
      (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) +
      (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
       (pHeader->HasConfidenceLevel()
            ? sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4)
            : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4)) *
           pHeader->GetLaserNum()) *
          pHeader->GetBlockNum() +
      sizeof(HS_LIDAR_BODY_CRC_ME_V4) +
      (pHeader->HasFuncSafety() ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0));
  if (pHeader->HasSeqNum()) {
    const HS_LIDAR_TAIL_SEQ_NUM_ME_V4 *pTailSeqNum =
        reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ME_V4 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) +
            (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
             (pHeader->HasConfidenceLevel()
                  ? sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4)
                  : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4)) *
                 pHeader->GetLaserNum()) *
                pHeader->GetBlockNum() +
            sizeof(HS_LIDAR_BODY_CRC_ME_V4) +
            (pHeader->HasFuncSafety() ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4)
                                      : 0) +
            sizeof(HS_LIDAR_TAIL_ME_V4));
    
    //skip decode packet if enable packet_loss_tool
    uint32_t packet_seqnum = pTailSeqNum->m_u32SeqNum;
    this->CalPktLoss(packet_seqnum);
  }

  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }    
  uint64_t packet_timestamp = pTail->GetMicroLidarTimeU64();
  this->CalPktTimeLoss(packet_timestamp);
  frame.host_timestamp = GetMicroTickCountU64();
  this->spin_speed_ = pTail->m_u16MotorSpeed;
  this->is_dual_return_= pTail->IsDualReturn();
  frame.spin_speed = pTail->GetMotorSpeed();
  frame.work_mode = pTail->m_u8RunningMode;
  frame.return_mode = pTail->GetReturnMode();
  frame.per_points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.scan_complete = false;
  frame.distance_unit = pHeader->GetDistUnit();
  frame.block_num = pHeader->GetBlockNum();
  frame.laser_num = pHeader->GetLaserNum();
  
  int index = frame.packet_num * pHeader->GetBlockNum() * pHeader->GetLaserNum();
  if (this->enable_update_monitor_info_) {
    this->monitor_info1_[pTail->m_reservedInfo1.m_u8ID] =
        pTail->m_reservedInfo1.m_u16Sts;
    this->monitor_info2_[pTail->m_reservedInfo2.m_u8ID] =
        pTail->m_reservedInfo2.m_u16Sts;
    this->monitor_info3_[pTail->m_reservedInfo3.m_u8ID] = pTail->m_reservedInfo3.m_u16Sts;
  }
  uint16_t u16Azimuth = 0;
  uint8_t optMode = pTail->getOperationMode();
  for (int i = 0; i < pHeader->GetBlockNum(); i++) {
    uint8_t angleState = pTail->getAngleState(i);
    u16Azimuth = pAzimuth->GetAzimuth();
    // point to channel unit addr
    if (pHeader->HasConfidenceLevel()) {
      const HS_LIDAR_BODY_CHN_UNIT_ME_V4 *pChnUnit =
          reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_ME_V4 *>(
              (const unsigned char *)pAzimuth +
              sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4));

      // point to next block azimuth addr
      pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4) * pHeader->GetLaserNum());
      auto elevation = 0;
      for (int j = 0; j < pHeader->GetLaserNum(); ++j) {
        if (this->get_firetime_file_) {
          float fireTimeCollection = this->rotation_flag * GetFiretimesCorrection(j, this->spin_speed_, optMode, angleState, pChnUnit->GetDistance() * pHeader->GetDistUnit());
          frame.pointData[index].azimuth = u16Azimuth + fireTimeCollection * kResolutionFloat;
        }else {
          frame.pointData[index].azimuth = u16Azimuth;
        }
        frame.pointData[index].reflectivities = pChnUnit->GetReflectivity();  
        frame.pointData[index].distances = pChnUnit->GetDistance();
        frame.pointData[index].confidence = pChnUnit->GetConfidenceLevel();
        frame.pointData[index].elevation = elevation;
        pChnUnit = pChnUnit + 1;
        index++;
      }
    } else {
      const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 *pChnUnitNoConf =
          reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 *>(
              (const unsigned char *)pAzimuth +
              sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4));

      // point to next block azimuth addr
      pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4) *
              pHeader->GetLaserNum());
      auto elevation = 0;
      for (int j = 0; j < pHeader->GetLaserNum(); ++j) {
        if (this->get_firetime_file_) {
          float fireTimeCollection = this->rotation_flag * GetFiretimesCorrection(j, this->spin_speed_, optMode, angleState, pChnUnitNoConf->GetDistance() * pHeader->GetDistUnit());
          frame.pointData[index].azimuth = u16Azimuth + fireTimeCollection * kResolutionFloat;
        } else {
          frame.pointData[index].azimuth = u16Azimuth;
        }
        frame.pointData[index].reflectivities = pChnUnitNoConf->GetReflectivity();  
        frame.pointData[index].distances = pChnUnitNoConf->GetDistance();
        frame.pointData[index].elevation = elevation;
        pChnUnitNoConf += 1;
        index++;
      }
    }
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