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
 * File:       udp6_1_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp6_1Parser class
*/

#include "udp6_1_parser.h"
#include "udp_protocol_v6_1.h"
#include "udp_protocol_header.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp6_1Parser<T_Point>::Udp6_1Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  block_num_ = 6; 
}
template<typename T_Point>
Udp6_1Parser<T_Point>::~Udp6_1Parser() { printf("release general parser\n"); }

template<typename T_Point>
int Udp6_1Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
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
      float x = 0;
      float y = 0;
      float z = 0;

      if (this->enable_distance_correction_) {
        GetDistanceCorrection(Azimuth, this->azimuth_collection_[i] * kResolutionInt, elevation , distance, x, y, z);
      } 
      else {
        float xyDistance = distance * this->cos_all_angle_[(elevation)];
        x = xyDistance * this->sin_all_angle_[(azimuth)];
        y = xyDistance * this->cos_all_angle_[(azimuth)];
        z = distance * this->sin_all_angle_[(elevation)];
      }    
      
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
int Udp6_1Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }
  const HsLidarXTV1Header *pHeader =
      reinterpret_cast<const HsLidarXTV1Header *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  const HsLidarXTV1Tail *pTail =
      reinterpret_cast<const HsLidarXTV1Tail *>(
          (const unsigned char *)pHeader + sizeof(HsLidarXTV1Header) +
          (sizeof(HsLidarXTV1BodyAzimuth) +
           sizeof(HsLidarXTV1BodyChannelData) * pHeader->GetLaserNum()) *
              pHeader->GetBlockNum());          
  this->spin_speed_ = pTail->m_u16MotorSpeed;
  this->is_dual_return_= pTail->IsDualReturn();
  output.spin_speed = pTail->m_u16MotorSpeed;

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
  
  const HsLidarXTV1BodyAzimuth *pAzimuth =
      reinterpret_cast<const HsLidarXTV1BodyAzimuth *>(
          (const unsigned char *)pHeader + sizeof(HsLidarXTV1Header));

  const HsLidarXTV1BodyChannelData *pChnUnit =
      reinterpret_cast<const HsLidarXTV1BodyChannelData *>(
          (const unsigned char *)pAzimuth +
          sizeof(HsLidarXTV1BodyAzimuth));
  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    pChnUnit = reinterpret_cast<const HsLidarXTV1BodyChannelData *>(
        (const unsigned char *)pAzimuth + sizeof(HsLidarXTV1BodyAzimuth));
    // point to next block azimuth addr
    pAzimuth = reinterpret_cast<const HsLidarXTV1BodyAzimuth *>(
        (const unsigned char *)pAzimuth + sizeof(HsLidarXTV1BodyAzimuth) +
        sizeof(HsLidarXTV1BodyChannelData) * pHeader->GetLaserNum());
    // point to next block fine azimuth addr
    auto elevation = 0;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) { 
      if (this->get_firetime_file_) {
        output.azimuth[index] = u16Azimuth + this->GetFiretimesCorrection(i, this->spin_speed_) * kResolutionInt;
      }else {
        output.azimuth[index] = u16Azimuth;
      }
      output.distances[index] = pChnUnit->GetDistance() ;
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
bool Udp6_1Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  if (this->last_azimuth_ > azimuth && (this->last_azimuth_- azimuth > kSplitFrameMinAngle)) {
      return true;
    }
  return false;
}

template<typename T_Point>
void Udp6_1Parser<T_Point>::GetDistanceCorrection(int const& aziOrigin,
                                                      int const& aziDelt,
                                                      int const& elevation,
                                                      float const& distance,
                                                      float& x,
                                                      float& y,
                                                      float& z
                                                      ) {
    int aziCal = (aziOrigin + aziDelt) % CIRCLE;                                                    
    if(distance <= 0.1) {
      float xyDistance = distance * this->cos_all_angle_[(elevation)];
      float x = xyDistance * this->sin_all_angle_[(aziCal)];
      float y = xyDistance * this->cos_all_angle_[(aziCal)];
      float z = distance * this->sin_all_angle_[(elevation)];
      return;
    }
    switch (block_num_) {
        case 6: // XTM
            distance_correction_b_ = 0.0130;
            distance_correction_h_ = 0.0305;
            break;
        case 8: // XT
            distance_correction_b_ = 0.0130;
            distance_correction_h_ = 0.0315;
            break;
        default:
            std::cout << __func__ << "default: never occur" << block_num_ << std::endl;
            break;
    }
    int aziCorrection = aziDelt % CIRCLE;
    float calDistance = distance
                  - this->cos_all_angle_[elevation] * (distance_correction_h_ * this->cos_all_angle_[aziCorrection] - distance_correction_b_ * this->sin_all_angle_[aziCorrection]);
    x = calDistance * this->cos_all_angle_[elevation] * this->sin_all_angle_[aziCal]
                  - distance_correction_b_ * this->cos_all_angle_[aziOrigin] + distance_correction_h_ * this->sin_all_angle_[aziOrigin];
    y = calDistance * this->cos_all_angle_[elevation] * this->cos_all_angle_[aziCal]
                  + distance_correction_b_ * this->sin_all_angle_[aziOrigin] + distance_correction_h_ * this->cos_all_angle_[aziOrigin];
    z = calDistance * this->sin_all_angle_[elevation];
}

