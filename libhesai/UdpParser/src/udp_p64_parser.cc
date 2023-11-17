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
 * File:       udp_p64_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente UdpP64Parser class
*/

#include "udp_p64_parser.h"
#include "general_parser.h"
#include "udp_protocol_p64.h"
using namespace hesai::lidar;
template<typename T_Point>
UdpP64Parser<T_Point>::UdpP64Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
}

template<typename T_Point>
UdpP64Parser<T_Point>::~UdpP64Parser() { printf("release general parser\n"); }

template<typename T_Point>
int UdpP64Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  for (int blockid = 0; blockid < packet.block_num; blockid++) {
    T_Point point;
    int elevation = 0;
    int azimuth = 0;   
    
    for (int i = 0; i < packet.laser_num; i++) {
      int point_index = packet.packet_index * packet.points_num + blockid * packet.laser_num + i;
      float distance = packet.distances[blockid * packet.laser_num + i] * packet.distance_unit;
      int Azimuth = packet.azimuth[blockid * packet.laser_num + i];
      if (this->get_correction_file_) {
        elevation = this->elevation_correction_[i] * kResolutionInt;
        elevation = (CIRCLE + elevation) % CIRCLE;
        azimuth = Azimuth + this->azimuth_collection_[i] * kResolutionInt;
        azimuth = (CIRCLE + azimuth) % CIRCLE;
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
int UdpP64Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }
  const HS_LIDAR_L64_Header *pHeader = 
      reinterpret_cast<const HS_LIDAR_L64_Header *>(&(udpPacket.buffer[0]));
  
  const HS_LIDAR_BODY_AZIMUTH_L64 *pAzimuth = 
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L64 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_L64_Header));
 
  const HS_LIDAR_BODY_CHN_UNIT_L64 *pChnUnit = 
      reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L64 *>(
          (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L64));

  const HS_LIDAR_TAIL_L64 *pTail = 
      reinterpret_cast<const HS_LIDAR_TAIL_L64 * >(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_L64_Header) + 
          (sizeof(HS_LIDAR_BODY_AZIMUTH_L64) + 
            sizeof(HS_LIDAR_BODY_CHN_UNIT_L64) * pHeader->GetLaserNum()) * 
              pHeader->GetBlockNum());
  this->spin_speed_ = pTail->GetMotorSpeed();
  output.spin_speed = pTail->m_u16MotorSpeed;
  output.host_timestamp = GetMicroTickCountU64();
  output.points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  output.scan_complete = false;
  output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  output.distance_unit = pHeader->GetDistUnit();
  output.block_num = pHeader->GetBlockNum();
  output.laser_num = pHeader->GetLaserNum();

  int index = 0;
  float minAzimuth = 0;
  float maxAzimuth = 0;

  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L64 *>(
          (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L64));

    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L64 *>(
      (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L64) + 
      sizeof(HS_LIDAR_BODY_CHN_UNIT_L64) * pHeader->GetLaserNum());
    
    auto elevation = 0;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      if(this->get_firetime_file_) {
        output.azimuth[index] = u16Azimuth  + this->GetFiretimesCorrection(i, this->spin_speed_) * kResolutionInt;
      }else {
        output.azimuth[index] = u16Azimuth;
      }
      output.distances[index] = pChnUnit->GetDistance();
      output.reflectivities[index] = pChnUnit->GetReflectivity();
      output.elevation[index] = elevation;
      pChnUnit = pChnUnit + 1;
      index++; 
    }
    if (IsNeedFrameSplit(u16Azimuth)) {
      output.scan_complete = true;
    }
    this->last_azimuth_ = u16Azimuth;
  }
  return 0;
}  

template<typename T_Point>
bool UdpP64Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  // Determine the correctness of the frame angle
  if (this->frame_start_azimuth_ < 0.0f || this->frame_start_azimuth_ >= 360.0f) {
    this->frame_start_azimuth_ = 0.0f;
    printf("the set of frame_start_azimuth is wrong, please make it [0, 360) (no 360), we set it 0\n");
  }
  static int32_t i = 0;
  int8_t fb_flag = 1;
  if (i)
  {
    if ((this->last_azimuth_ > azimuth && azimuth - this->last_azimuth_ <= kSplitFrameMinAngle) || (azimuth - this->last_azimuth_ > kSplitFrameMinAngle*100)) {
      fb_flag = 0;
    }
  } else {
    i++;
    return false;
  }
  if (fb_flag) {
    // Special 0 is handled separately
    if (this->frame_start_azimuth_ == 0.0f)
    {
      if (this->last_azimuth_ > azimuth && (this->last_azimuth_- azimuth > kSplitFrameMinAngle)) {
        return true;
      } 
      return false;
    } else {
      if (this->last_azimuth_ < azimuth && this->last_azimuth_ < uint16_t(this->frame_start_azimuth_ * kResolutionInt) 
          && azimuth >= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) {
        return true;
      }
      return false;
    }
  } else {
    // Special the last degree is handled separately
    static int j = 0;
    static float divison = 0.8f;
    if (j == 0 && this->last_azimuth_ != 0 && abs(this->last_azimuth_ - azimuth) < kSplitFrameMinAngle)
    {
      j++;
      divison = (this->last_azimuth_ - azimuth) / 100.0;
    }
    if (360.0f - this->frame_start_azimuth_ <= divison)
    {
      if (this->last_azimuth_ < azimuth && (azimuth - this->last_azimuth_ > kSplitFrameMinAngle)) {
        return true;
      } 
      return false;
    } else {
      if (this->last_azimuth_ > azimuth && this->last_azimuth_ > uint16_t(this->frame_start_azimuth_ * kResolutionInt) 
          && azimuth <= uint16_t(this->frame_start_azimuth_ * kResolutionInt)) {
        return true;
      }
      return false;
    }
  }
}

