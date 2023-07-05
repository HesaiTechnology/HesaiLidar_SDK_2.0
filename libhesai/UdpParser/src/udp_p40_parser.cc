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
 * File:       udp_p40_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente UdpP40Parser class
*/

#include "udp_p40_parser.h"
#include "general_parser.h"
#include "udp_protocol_p40.h"
using namespace hesai::lidar;
#define DISTANCEUNIT 0.004
template<typename T_Point>
UdpP40Parser<T_Point>::UdpP40Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
}

template<typename T_Point>
UdpP40Parser<T_Point>::~UdpP40Parser() { printf("release general parser\n"); }

template<typename T_Point>
int UdpP40Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  for (int blockid = 0; blockid < packet.block_num; blockid++) {
    T_Point point;
    int elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < packet.laser_num; i++) {
      int point_index = packet.packet_index * packet.points_num + blockid * packet.laser_num + i;
      float distance = packet.distances[blockid * packet.laser_num + i] * DISTANCEUNIT;
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
int UdpP40Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  if (udpPacket.buffer[0] != 0xFF || udpPacket.buffer[1] != 0xEE ) {
    return -1;
  }
  const HS_LIDAR_BODY_AZIMUTH_L40 *pAzimuth = 
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L40 *> (&(udpPacket.buffer[0]));
  
  const HS_LIDAR_BODY_CHN_UNIT_L40 *pChnUnit = 
      reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L40 *> ((const unsigned char *)pAzimuth + 
        sizeof(HS_LIDAR_BODY_AZIMUTH_L40));

  const HS_LIDAR_TAIL_L40 *pTail = 
      reinterpret_cast<const HS_LIDAR_TAIL_L40 *> (
        &(udpPacket.buffer[0]) +(sizeof(HS_LIDAR_BODY_AZIMUTH_L40) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_L40) * LASERNUM) * BLOCKNUM);

  this->spin_speed_ = pTail->GetMotorSpeed();
  output.spin_speed = pTail->m_u16MotorSpeed;

  output.points_num = BLOCKNUM * LASERNUM;
  output.scan_complete = false;
  output.block_num = BLOCKNUM;
  output.laser_num = LASERNUM;
  output.distance_unit = DISTANCEUNIT;

  output.host_timestamp = GetMicroTickCountU64();
  output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  int index = 0;
  float minAzimuth = 0;
  float maxAzimuth = 0;

  for (int j = 0; j < BLOCKNUM; j++) {
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L40 *>((const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L40));

    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L40 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L40) + 
        sizeof(HS_LIDAR_BODY_CHN_UNIT_L40) * LASERNUM
    );
    auto elevation = 0;
    for (int i = 0; i < LASERNUM; i++) {
      if (this->get_firetime_file_) {
        output.azimuth[index] = u16Azimuth + this->GetFiretimesCorrection(i, this->spin_speed_) * kResolutionInt;
      }else {
        output.azimuth[index] = u16Azimuth;
      }
      output.distances[index] = pChnUnit->GetDistance();
      output.reflectivities[index] = pChnUnit->GetReflectivity();
      output.elevation[index] = elevation;
      pChnUnit = pChnUnit + 1;
      index = index + 1;   
    }

    if (IsNeedFrameSplit(u16Azimuth)) {
      output.scan_complete = true;
    }
    this->last_azimuth_ = u16Azimuth;  
  }
  return 0;
}  

template<typename T_Point>
bool UdpP40Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  if (this->last_azimuth_ > azimuth && (this->last_azimuth_- azimuth > kSplitFrameMinAngle)) {
      return true;
    }
  return false;
}
