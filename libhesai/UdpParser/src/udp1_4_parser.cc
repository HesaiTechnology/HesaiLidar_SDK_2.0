/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
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
}

template<typename T_Point>
Udp1_4Parser<T_Point>::~Udp1_4Parser() { printf("release general parser\n"); }

template<typename T_Point>
void Udp1_4Parser<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  std::ifstream inFile(firetimes_path, std::ios::in);
  if (inFile.is_open()) {
    int uselessLine = 0;
    int count = 0;
    std::string lineStr;
    while (std::getline(inFile, lineStr)) {
      std::stringstream ss(lineStr);
      std::string str;
      std::vector<std::string> strList;
      strList.reserve(20);
      while(std::getline(ss, str, ',')){
        strList.push_back(str);
      }
      if(uselessLine == 0) {
        std::string dist;
        for (auto e : strList[0]) {
          if (e == '.' || (e >= '0' && e <= '9')) {
            dist.push_back(e);
          }
        }
        section_distance = std::stod(dist);
      }
      if(uselessLine < 3) {
        uselessLine++;
        continue;
      }
      if (lineStr[lineStr.size() - 1] == '\n'){
        lineStr = lineStr.substr(lineStr.size() - 1);
      }
      if (strList.size() < 17) {
        std::cout << "nvalid input file!" << std::endl;
        this->get_firetime_file_ = false;
        return;
      }
      int idx = std::stoi(strList[0]) - 1;
      for (int i = 1; i <= 15; i += 2) {
        int a = std::stoi(strList[i]);
        int b = std::stoi(strList[i + 1]);
        firetime_section_values[idx].section_values[i / 2].firetime[0] = a;
        firetime_section_values[idx].section_values[i / 2].firetime[1] = b;
      }
      count++;
      if (count > 128) {
        std::cout << "Invalid input file!" << std::endl;
        this->get_firetime_file_ = false;
        return;
      }
    }
  } else {
    std::cout << "Open firetime file failed" << std::endl;
    this->get_firetime_file_ = false;
    return;
  }
  this->get_firetime_file_ = true;
  std::cout << "Open firetime file success!" << std::endl;
  return;
}

template<typename T_Point>
double Udp1_4Parser<T_Point>::GetFiretimesCorrection(int laserId, double speed, uint8_t optMode, uint8_t angleState,uint16_t dist){
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
int Udp1_4Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  for (int blockid = 0; blockid < packet.block_num; blockid++) {
    T_Point point;
    int Azimuth = packet.azimuth[blockid * packet.laser_num];
    int elevation = 0;
    auto azimuth = Azimuth;

    for (int i = 0; i < packet.laser_num; i++) {
      int point_index = packet.packet_index * packet.points_num + blockid * packet.laser_num + i;
      float distance = packet.distances[blockid * packet.laser_num + i] * packet.distance_unit;  
      if (this->get_correction_file_) {
        elevation = this->elevation_correction_[i] * kResolutionInt;
        elevation = (CIRCLE + elevation) % CIRCLE;
        azimuth = Azimuth + this->azimuth_collection_[i] * kResolutionInt;
        azimuth = (CIRCLE + azimuth) % CIRCLE;
      } 
      if (this->enable_distance_correction_) {
        GetDistanceCorrection(i, distance, azimuth, elevation);
      }
      float xyDistance = distance * this->cos_all_angle_[(elevation)];
      float x = xyDistance * this->sin_all_angle_[(azimuth)];
      float y = xyDistance * this->cos_all_angle_[(azimuth)];
      float z = distance * this->sin_all_angle_[(elevation)];

      this->TransformPoint(x, y, z);
      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], packet.reflectivities[blockid * packet.laser_num + i]);
      setTimestamp(frame.points[point_index], double(packet.sensor_timestamp) / kMicrosecondToSecond);
      setRing(frame.points[point_index], i);
    }
  }
  frame.points_num += packet.points_num;
  frame.packet_num = packet.packet_index;
  return 0;
}

template<typename T_Point>
int Udp1_4Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
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
    output.lidar_state = function_savety_ptr->GetLidarState();
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
  }    
  this->spin_speed_ = pTail->m_u16MotorSpeed;
  this->is_dual_return_= pTail->IsDualReturn();
  output.spin_speed = pTail->m_u16MotorSpeed;
  output.work_mode = pTail->getOperationMode();
  output.host_timestamp = GetMicroTickCountU64();
  // 如下三条：max min这样的参数一点用都没有
  output.maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // 不填直接崩调，=0界面一个点也没有
  output.points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // 不填则仅显示很小一部分点云
  output.scan_complete = false;
  // 不填可以播放，只是显示的时间戳不对
  output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  output.distance_unit = pHeader->GetDistUnit();
  int index = 0;
  float minAzimuth = 0;
  float maxAzimuth = 0;
  output.block_num = pHeader->GetBlockNum();
  output.laser_num = pHeader->GetLaserNum();
  
  if (this->enable_update_monitor_info_) {
    this->monitor_info1_[pTail->m_reservedInfo1.m_u8ID] =
        pTail->m_reservedInfo1.m_u16Sts;
    this->monitor_info2_[pTail->m_reservedInfo2.m_u8ID] =
        pTail->m_reservedInfo2.m_u16Sts;
    this->monitor_info3_[pTail->m_reservedInfo3.m_u8ID] = pTail->m_reservedInfo3.m_u16Sts;
  }
  uint8_t optMode = pTail->getOperationMode();
  for (int i = 0; i < pHeader->GetBlockNum(); i++) {
    uint8_t angleState = pTail->getAngleState(i);
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    output.azimuths = u16Azimuth;
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
          float fireTimeCollection = GetFiretimesCorrection(i, this->spin_speed_, optMode, angleState, pChnUnit->GetDistance());
          output.azimuth[index] = u16Azimuth + fireTimeCollection * kResolutionInt;
        }else {
          output.azimuth[index] = u16Azimuth;
        }
        output.reflectivities[index] = pChnUnit->GetReflectivity();  
        output.distances[index] = pChnUnit->GetDistance();
        output.elevation[index] = elevation;
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
          float fireTimeCollection = GetFiretimesCorrection(i, this->spin_speed_, optMode, angleState, pChnUnitNoConf->GetDistance());
          output.azimuth[index] = u16Azimuth + fireTimeCollection * kResolutionInt;
        }else {
          output.azimuth[index] = u16Azimuth;
        }
        output.reflectivities[index] = pChnUnitNoConf->GetReflectivity();  
        output.distances[index] = pChnUnitNoConf->GetDistance();
        output.elevation[index] = elevation;
        index++;
        pChnUnitNoConf += 1;
      }
    }
    if (IsNeedFrameSplit(u16Azimuth)) {
      output.scan_complete = true;
    }
    this->last_azimuth_ = u16Azimuth;
  }
  
  return 0;
} 

template<typename T_Point>
bool Udp1_4Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  if (this->last_azimuth_ > azimuth && (this->last_azimuth_- azimuth > kSplitFrameMinAngle)) {
      return true;
    }
  return false;
}

template<typename T_Point>
void Udp1_4Parser<T_Point>::GetDistanceCorrection(int laser_id, float distance, int& azimuth, int& elevation) {    
  double sin_delt_elevation = distance_correction_para_c_ / distance * this->sin_all_angle_[elevation];
  if (sin_delt_elevation >= -1 && sin_delt_elevation <= 1) {
      elevation -= distance_correction_para_a_ * std::asin(sin_delt_elevation) * kHalfCircleFloat / M_PI;
      elevation = elevation % CIRCLE;
  }
  double sin_delt_azimuth = distance_correction_para_c_ / distance / this->cos_all_angle_[elevation] * \
                        this->sin_all_angle_[int((distance_correction_para_d_ + this->azimuth_collection_[laser_id]) * kResolutionInt) % CIRCLE];
  if (sin_delt_azimuth >= -1 && sin_delt_azimuth <= 1) {
      azimuth -= distance_correction_para_a_ * std::asin(sin_delt_azimuth) * kHalfCircleFloat / M_PI;
      azimuth = azimuth % CIRCLE;
  }
}