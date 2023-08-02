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
 * File:       udp3_1_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp3_1Parser class
*/

#include "udp3_1_parser.h"
#include "udp_protocol_v3_1.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp3_1Parser<T_Point>::Udp3_1Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
}

template<typename T_Point>
Udp3_1Parser<T_Point>::~Udp3_1Parser() { printf("release general parser\n"); }

template<typename T_Point>
int Udp3_1Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  for (int blockid = 0; blockid < packet.block_num; blockid++) {
    T_Point point;
    int elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < packet.laser_num; i++) {
      int point_index = packet.packet_index * packet.points_num + blockid * packet.laser_num + i; 
      float distance = packet.distances[blockid * packet.laser_num + i] * packet.distance_unit; 
      int Azimuth = packet.azimuth[blockid * packet.laser_num + i];
      if (this->get_correction_file_) {
        elevation = packet.elevation[blockid * packet.laser_num + i];
        elevation = (CIRCLE + elevation) % CIRCLE;
        azimuth = Azimuth;
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
int Udp3_1Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }
  const HS_LIDAR_HEADER_QT_V1 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_QT_V1 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  const HS_LIDAR_TAIL_QT_V1 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_QT_V1 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V1) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V1) +
           sizeof(HS_LIDAR_BODY_CHN_NNIT_QT_V1) * pHeader->GetLaserNum()) *
              pHeader->GetBlockNum());
  // pTail->CalPktLoss(this->start_seqnum_, this->last_seqnum_, this->loss_count_, this->start_time_);            
  this->spin_speed_ = pTail->m_u16MotorSpeed;
  this->is_dual_return_= pTail->IsDualReturn();
  output.spin_speed = pTail->m_u16MotorSpeed;
  output.distance_unit = pHeader->GetDistUnit();

  output.host_timestamp = GetMicroTickCountU64();
  // 如下三条：max min这样的参数一点用都没有
  output.maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // 不填直接崩调，=0界面一个点也没有
  output.points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // 不填则仅显示很小一部分点云
  output.scan_complete = false;
  // 不填可以播放，只是显示的时间戳不对
  output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  int index = 0;
  float minAzimuth = 0;
  float maxAzimuth = 0;
  output.block_num = pHeader->GetBlockNum();
  output.laser_num = pHeader->GetLaserNum();
  
  const HS_LIDAR_BODY_AZIMUTH_QT_V1 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V1 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V1));

  const HS_LIDAR_BODY_CHN_NNIT_QT_V1 *pChnUnit =
      reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_QT_V1 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V1));
  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    output.azimuths = u16Azimuth;
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_QT_V1 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V1));
    // point to next block azimuth addr
    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V1 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V1) +
        sizeof(HS_LIDAR_BODY_CHN_NNIT_QT_V1) * pHeader->GetLaserNum());
    // point to next block fine azimuth addr
    auto elevation = 0;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      double elevationCorr = this->elevation_correction_[i];
      double azimuthCorr = u16Azimuth / kResolutionFloat + this->azimuth_collection_[i];
      double distance = static_cast<double>(pChnUnit->GetDistance()) * pHeader->GetDistUnit();
      if (this->enable_distance_correction_) {
        GetDistanceCorrection(azimuthCorr, elevationCorr, distance);
      }
      if (this->enable_firetime_correction_ && this->get_firetime_file_) {
        azimuthCorr += this->GetFiretimesCorrection(i, this->spin_speed_) * kResolutionInt;
      }
      output.reflectivities[index] = pChnUnit->GetReflectivity();  
      output.distances[index] = pChnUnit->GetDistance();
      output.azimuth[index] = azimuthCorr * kResolutionFloat;
      output.elevation[index] = elevationCorr * kResolutionFloat;
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
bool Udp3_1Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth) {
  if (this->last_azimuth_ > azimuth && (this->last_azimuth_- azimuth > kSplitFrameMinAngle)) {
      return true;
    }
  return false;
}

template<typename T_Point>
void Udp3_1Parser<T_Point>::GetDistanceCorrection(double &azi, double &ele,
                                          double &distance) {
  int aziIndex = int(azi * kResolutionFloat);
  int eleIndex = int(ele * kResolutionFloat);
  if (aziIndex >= CIRCLE) aziIndex -= CIRCLE;
  if (aziIndex < 0) aziIndex += CIRCLE;
  if (eleIndex >= CIRCLE) eleIndex -= CIRCLE;
  if (eleIndex < 0) eleIndex += CIRCLE;
  float point_x, point_y, point_z;
  if (distance > 0.1) {
    if (this->sin_all_angle_[eleIndex] != 0) {
      // printf("%d  %f\n", eleIndex, this->sin_all_angle_[eleIndex]);
      float c = (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG *
                     HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG +
                 HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT *
                     HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT -
                 distance * distance) *
                this->sin_all_angle_[eleIndex] * this->sin_all_angle_[eleIndex];
      float b = 2 * this->sin_all_angle_[eleIndex] * this->cos_all_angle_[eleIndex] *
                (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG *
                     this->cos_all_angle_[aziIndex] -
                 HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT *
                     this->sin_all_angle_[aziIndex]);
      point_z = (-b + sqrt(b * b - 4 * c)) / 2;
      point_x = point_z * this->sin_all_angle_[aziIndex] * this->cos_all_angle_[eleIndex] /
                    this->sin_all_angle_[eleIndex] -
                HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT;
      point_y = point_z * this->cos_all_angle_[aziIndex] * this->cos_all_angle_[eleIndex] /
                    this->sin_all_angle_[eleIndex] +
                HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
      if (((point_x + HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT) *
               this->cos_all_angle_[eleIndex] * this->sin_all_angle_[aziIndex] +
           (point_y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) *
               this->cos_all_angle_[eleIndex] * this->cos_all_angle_[aziIndex] +
           point_z * this->sin_all_angle_[eleIndex]) <= 0) {
        point_z = (-b - sqrt(b * b - 4 * c)) / 2;
        point_x = point_z * this->sin_all_angle_[aziIndex] *
                      this->cos_all_angle_[eleIndex] / this->sin_all_angle_[eleIndex] -
                  HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT;
        point_y = point_z * this->cos_all_angle_[aziIndex] *
                      this->cos_all_angle_[eleIndex] / this->sin_all_angle_[eleIndex] +
                  HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
      }
    } else if (this->cos_all_angle_[aziIndex] != 0) {
      float tan_azimuth = this->sin_all_angle_[aziIndex] / this->cos_all_angle_[aziIndex];
      float c = (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * tan_azimuth +
                 HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT) *
                    (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * tan_azimuth +
                     HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT) -
                distance * distance;
      float a = 1 + tan_azimuth * tan_azimuth;
      float b = -2 * tan_azimuth *
                (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * tan_azimuth +
                 HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT);
      point_z = 0;
      point_y = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      point_x =
          (point_y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) * tan_azimuth -
          HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT;
      if (((point_x + HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT) *
               this->cos_all_angle_[eleIndex] * this->sin_all_angle_[aziIndex] +
           (point_y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) *
               this->cos_all_angle_[eleIndex] * this->cos_all_angle_[aziIndex] +
           point_z * this->sin_all_angle_[eleIndex]) <= 0) {
        point_z = 0;
        point_y = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
        point_x = (point_y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) *
                      tan_azimuth -
                  HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT;
      }
    } else {
      point_x = sqrt(distance * distance -
                     HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG *
                         HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG);
      point_y = HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
      point_z = 0;
      if (((point_x + HS_LIDAR_QT_COORDINATE_CORRECTION_OGOT) *
               this->cos_all_angle_[eleIndex] * this->sin_all_angle_[aziIndex] +
           (point_y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) *
               this->cos_all_angle_[eleIndex] * this->cos_all_angle_[aziIndex] +
           point_z * this->sin_all_angle_[eleIndex]) <= 0) {
        point_x = -sqrt(distance * distance -
                        HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG *
                            HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG);
        point_y = HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
        point_z = 0;
      }
    }
    azi = atan2(point_x, point_y) / M_PI * (kHalfCircleInt / kResolutionInt);
    azi = azi < 0 ? azi + (kCircle / kResolutionFloat) : azi;
    ele = atan2(point_z, sqrt(point_x * point_x + point_y * point_y)) / M_PI *
          180;
    ele = ele < 0 ? ele + (kCircle / kResolutionFloat) : ele;
    distance = sqrt(point_x * point_x + point_y * point_y + point_z * point_z);
  }
}





