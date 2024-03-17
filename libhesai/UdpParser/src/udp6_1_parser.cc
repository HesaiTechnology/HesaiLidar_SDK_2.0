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
Udp6_1Parser<T_Point>::~Udp6_1Parser() { printf("release Udp6_1parser\n"); }

template<typename T_Point>
int Udp6_1Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  frame.work_mode = packet.work_mode;
  frame.spin_speed = packet.spin_speed;
  for (int blockid = 0; blockid < packet.block_num; blockid++) {
    // T_Point point;
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
      if (packet.config.fov_start != -1 && packet.config.fov_end != -1)
      {
        int fov_transfer = azimuth / 256 / 100;
        if (fov_transfer < packet.config.fov_start || fov_transfer > packet.config.fov_end){//不在fov范围continue
          continue;
        }
      }       
      float x = 0;
      float y = 0;
      float z = 0;
      if (this->enable_distance_correction_) {
        GetDistanceCorrection(Azimuth, this->azimuth_collection_[i] * kResolutionInt, elevation , distance, x, y, z);
      } else {
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

  if (output.use_timestamp_type == 0) {
    output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  } else {
    output.sensor_timestamp = udpPacket.recv_timestamp;
  }
  output.host_timestamp = GetMicroTickCountU64();

  if (this->enable_packet_loss_tool_ == true) {
    this->current_seqnum_ = pTail->m_u32SeqNum;
    if (this->current_seqnum_ > this->last_seqnum_ && this->last_seqnum_ != 0) {
      this->total_packet_count_ += this->current_seqnum_ - this->last_seqnum_;
    }
    pTail->CalPktLoss(this->start_seqnum_, this->last_seqnum_, this->loss_count_, 
        this->start_time_, this->total_loss_count_, this->total_start_seqnum_);
    // return 0;
    // why return 0 ? 
  }

  this->spin_speed_ = pTail->m_u16MotorSpeed;
  this->is_dual_return_= pTail->IsDualReturn();
  output.spin_speed = pTail->GetMotorSpeed();
  output.work_mode = pTail->HasShutdown();
  output.maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  output.points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  output.scan_complete = false;
  output.distance_unit = pHeader->GetDistUnit();
  int index = 0;
  // float minAzimuth = 0;
  // float maxAzimuth = 0;
  output.block_num = pHeader->GetBlockNum();
  output.laser_num = pHeader->GetLaserNum();
  
  const HsLidarXTV1BodyAzimuth *pAzimuth =
      reinterpret_cast<const HsLidarXTV1BodyAzimuth *>(
          (const unsigned char *)pHeader + sizeof(HsLidarXTV1Header));

  const HsLidarXTV1BodyChannelData *pChnUnit =
      reinterpret_cast<const HsLidarXTV1BodyChannelData *>(
          (const unsigned char *)pAzimuth +
          sizeof(HsLidarXTV1BodyAzimuth));
  uint16_t u16Azimuth = 0;
  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    u16Azimuth = pAzimuth->GetAzimuth();
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
  }
  if (IsNeedFrameSplit(u16Azimuth)) {
    output.scan_complete = true;
  }
  this->last_last_azimuth_ = this->last_azimuth_;
  this->last_azimuth_ = u16Azimuth;
  return 0;
}  

template<typename T_Point>
bool Udp6_1Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  // Determine frame_start_azimuth_ [0,360)
  if (this->frame_start_azimuth_ < 0.0f || this->frame_start_azimuth_ >= 360.0f) {
    this->frame_start_azimuth_ = 0.0f;
  }
  // The first two packet dont have the information of last_azimuth_  and last_last_azimuth, so do not need split frame
  // The initial value of last_azimuth_ is -1
  // Determine the rotation direction and division
  int8_t rotation_flag = 1;
  uint16_t division = 0;
  // If last_last_azimuth_ != -1，the packet is the third, so we can determine whether the current packet requires framing
  if (this->last_last_azimuth_ != -1) 
  {
    // Get the division
    uint16_t division1 = abs(this->last_azimuth_ - this->last_last_azimuth_);
    uint16_t division2 = abs(this->last_azimuth_ - azimuth);
    division = std::min(division1, division2);
    // Prevent two consecutive packets from having the same angle when causing an error in framing
    if ( division == 0) return false;
    // In the three consecutive angle values, if the angle values appear by the division of the decreasing situation,it must be reversed
    // The same is true for FOV
    if( this->last_last_azimuth_ - this->last_azimuth_ == division || this->last_azimuth_ -azimuth == division)
    {
      rotation_flag = 0;
    }
  } else {
    // The first  and second packet do not need split frame
    return false;
  }
  if (rotation_flag) {
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
      x = xyDistance * this->sin_all_angle_[(aziCal)];
      y = xyDistance * this->cos_all_angle_[(aziCal)];
      z = distance * this->sin_all_angle_[(elevation)];
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

template<typename T_Point>
int Udp6_1Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
{
  // TO DO
  return 0;
}