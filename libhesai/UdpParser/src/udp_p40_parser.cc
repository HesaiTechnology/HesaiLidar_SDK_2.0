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
 * File:       udp_p40_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente UdpP40Parser class
*/

#include "udp_p40_parser.h"

using namespace hesai::lidar;
template<typename T_Point>
UdpP40Parser<T_Point>::UdpP40Parser() {
  this->optical_center.setNoFlag(LidarOpticalCenter{-0.012, 0.03873, 0});
  this->default_remake_config.min_azi = 0.f;
  this->default_remake_config.max_azi = 360.f;
  this->default_remake_config.ring_azi_resolution = 0.2f;
  this->default_remake_config.max_azi_scan = 1800;   // (max_azi - min_azi) / ring_azi_resolution
  this->default_remake_config.min_elev = -25.f;
  this->default_remake_config.max_elev = 15.f;
  this->default_remake_config.ring_elev_resolution = 0.32f;
  this->default_remake_config.max_elev_scan = 125;   // (max_elev - min_elev) / ring_elev_resolution
  LogInfo("init P40 parser");
}

template<typename T_Point>
UdpP40Parser<T_Point>::~UdpP40Parser() { LogInfo("release P40 parser"); }

template <typename T_Point>
void UdpP40Parser<T_Point>::setFrameRightMemorySpace(LidarDecodedFrame<T_Point> &frame) {
  frame.resetMalloc(1800, 400);
}

template<typename T_Point>
int UdpP40Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, uint32_t packet_index) {
  if (packet_index >= frame.maxPacketPerFrame || frame.point_cloud_raw_data == nullptr) {
    LogFatal("packet_index(%d) out of %d. or data ptr is nullptr", packet_index, frame.maxPacketPerFrame);
    GeneralParser<T_Point>::FrameNumAdd();
    return -1;
  }
  uint8_t* data = frame.point_cloud_raw_data + packet_index * frame.point_cloud_size;
  const HS_LIDAR_TAIL_L40 *pTail = 
      reinterpret_cast<const HS_LIDAR_TAIL_L40 *> (
        data + (sizeof(HS_LIDAR_BODY_AZIMUTH_L40) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_L40) * P40_LASERNUM) * P40_BLOCKNUM);

  int point_index = packet_index * frame.per_points_num;
  int point_num = 0;
  auto& packetData = frame.packetData[packet_index];
  int32_t block_ns_offset = 0;
  for (int blockid = 0; blockid < P40_BLOCKNUM; blockid++) {
    int current_block_echo_count = frame.return_mode < RETURN_MODE_MULTI ? 0 : blockid % 2 + 1;
    if (frame.fParam.echo_mode_filter != 0 && current_block_echo_count != 0 && frame.fParam.echo_mode_filter != current_block_echo_count) {
      continue;
    }
    block_ns_offset = P40::P40_BLOCK_NS_OFFSET1 + P40::P40_BLOCK_NS_OFFSET2 * int((P40_BLOCKNUM - blockid - 1) / (frame.return_mode < RETURN_MODE_MULTI ? 1 : 2));
    const HS_LIDAR_BODY_AZIMUTH_L40 *pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L40 *>(
                data + (sizeof(HS_LIDAR_BODY_AZIMUTH_L40) + 
                sizeof(HS_LIDAR_BODY_CHN_UNIT_L40) * P40_LASERNUM) * blockid);
    const HS_LIDAR_BODY_CHN_UNIT_L40 *pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_L40 *>(
                                                  (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_L40));
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    for (int channel_index = 0; channel_index < P40_LASERNUM; channel_index++) {
      if (this->correction.display[channel_index] == false) {
        pChnUnit++;
        continue;
      }
      int elevation = 0;
      int azimuth = u16Azimuth * kFineResolutionInt;
      float distance = static_cast<float>(pChnUnit->GetDistance() * frame.distance_unit);
      if (this->get_firetime_file_ && frame.fParam.firetimes_flag) {
        azimuth += (frame.fParam.rotation_flag > 0 ? 1 : -1) * 
          doubleToInt(this->GetFiretimesCorrection(channel_index, pTail->GetMotorSpeed()) * kAllFineResolutionInt);
      }
      if (this->get_correction_file_) {
        int azimuth_coll = doubleToInt(this->correction.azimuth[channel_index] * kAllFineResolutionFloat);
        int elevation_corr = doubleToInt(this->correction.elevation[channel_index] * kAllFineResolutionFloat);
        if (frame.fParam.distance_correction_flag) {
          GeneralParser<T_Point>::GetDistanceCorrection(this->optical_center, azimuth_coll, elevation_corr, distance, GeometricCenter);
        }
        elevation = elevation_corr;
        azimuth += azimuth_coll;
      } 
      this->CircleRevise(azimuth);
      this->CircleRevise(elevation);
      if (this->IsChannelFovFilter(azimuth / kAllFineResolutionInt, channel_index, frame.fParam) == 1) {
        pChnUnit++;
        continue;
      }

      uint64_t timestamp = packetData.t.sensor_timestamp * kMicrosecondToNanosecondInt;
      if (this->get_firetime_file_) {
        timestamp += block_ns_offset + floatToInt(this->firetime_correction_[channel_index] * kMicrosecondToNanosecondInt);
      }
      float xyDistance = distance * this->cos_all_angle_[(elevation)];
      float x = xyDistance * this->sin_all_angle_[(azimuth)];
      float y = xyDistance * this->cos_all_angle_[(azimuth)];
      float z = distance * this->sin_all_angle_[(elevation)];
      this->TransformPoint(x, y, z, frame.fParam.transform);

      int point_index_rerank = point_index + point_num; 
      GeneralParser<T_Point>::DoRemake(azimuth, elevation, frame.fParam.remake_config, point_index_rerank); 
      if(point_index_rerank >= 0) { 
        auto& ptinfo = frame.points[point_index_rerank]; 
        set_x(ptinfo, x); 
        set_y(ptinfo, y); 
        set_z(ptinfo, z); 
        set_ring(ptinfo, channel_index); 
        set_intensity(ptinfo, pChnUnit->GetReflectivity());  
        set_timestamp(ptinfo, double(packetData.t.sensor_timestamp) / kMicrosecondToSecond);
        set_timeSecond(ptinfo, timestamp / kNanosecondToSecondInt);
        set_timeNanosecond(ptinfo, timestamp % kNanosecondToSecondInt);

        point_num++;
      }
      pChnUnit++;
    }
  }
  frame.valid_points[packet_index] = point_num;
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
int UdpP40Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket, const int packet_index)
{
  uint32_t packet_index_use = packet_index >= 0 ? packet_index : frame.packet_num;
  if (udpPacket.buffer[0] != 0xFF || udpPacket.buffer[1] != 0xEE || 
      (udpPacket.packet_len != PKT_SIZE_40P && udpPacket.packet_len != PKT_SIZE_40P + 4 &&
      udpPacket.packet_len != PKT_SIZE_AC)) {
    LogDebug("Invalid point cloud");
    return -1;
  }
  frame.scan_complete = false;

  const HS_LIDAR_TAIL_L40 *pTail = 
      reinterpret_cast<const HS_LIDAR_TAIL_L40 *> (
        &(udpPacket.buffer[0]) +(sizeof(HS_LIDAR_BODY_AZIMUTH_L40) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_L40) * P40_LASERNUM) * P40_BLOCKNUM);

  if (frame.frame_init_ == false) {
    frame.block_num = P40_BLOCKNUM;
    frame.laser_num = P40_LASERNUM;
    frame.per_points_num = P40_BLOCKNUM * P40_LASERNUM;
    frame.distance_unit = P40_DISTANCEUNIT;
    if (frame.per_points_num > frame.maxPointPerPacket) {
      LogFatal("per_points_num(%u) out of %d", frame.per_points_num, frame.maxPointPerPacket);
      return -1;
    }
    frame.frame_init_ = true;
  }
  // frame split
  uint16_t u16Azimuth = 0;
  const HS_LIDAR_BODY_AZIMUTH_L40 *pAzimuth = 
        reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_L40 *>(&(udpPacket.buffer[0]));
  u16Azimuth = pAzimuth->GetAzimuth();
  if (this->IsNeedFrameSplit(u16Azimuth, frame.fParam)) {
    frame.scan_complete = true;
  }
  if (u16Azimuth != this->last_azimuth_) {
    this->last_last_azimuth_ = this->last_azimuth_; 
    this->last_azimuth_ = u16Azimuth;  
  }
  if (frame.scan_complete)
    return 0;

  const HS_LIDAR_TAIL_SEQ_NUM_L40 *pSeqNum = 
      reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_L40 *> (
        (const unsigned char *)pTail + sizeof(HS_LIDAR_TAIL_L40));

  this->CalPktLoss(pSeqNum->GetSeqNum(), frame.fParam);
  this->CalPktTimeLoss(pTail->GetMicroLidarTimeU64(this->last_utc_time), frame.fParam);
  frame.lidar_state = pTail->m_u8Shutdown;
  frame.return_mode = pTail->m_u8ReturnMode;

  if (frame.fParam.use_timestamp_type == 0) {
    frame.packetData[packet_index_use].t.sensor_timestamp = pTail->GetMicroLidarTimeU64(this->last_utc_time);
  } else {
    frame.packetData[packet_index_use].t.sensor_timestamp = udpPacket.recv_timestamp;
  }
  if (frame.frame_start_timestamp == 0) frame.frame_start_timestamp = double(frame.packetData[packet_index_use].t.sensor_timestamp) / kMicrosecondToSecond;
  frame.frame_end_timestamp = double(frame.packetData[packet_index_use].t.sensor_timestamp) / kMicrosecondToSecond;
  

  auto packet_size = udpPacket.packet_len;
  if (this->last_max_packet_num_ != frame.maxPacketPerFrame) {
    this->last_max_packet_num_ = frame.maxPacketPerFrame;
    if (frame.point_cloud_raw_data != nullptr) delete[] frame.point_cloud_raw_data;
    frame.point_cloud_size = packet_size;
    frame.point_cloud_raw_data = new uint8_t[frame.point_cloud_size * frame.maxPacketPerFrame];
    memset(frame.point_cloud_raw_data, 0, frame.point_cloud_size * frame.maxPacketPerFrame);
  }
  if (frame.point_cloud_size != packet_size) {
    LogFatal("point cloud size is should be %d, but is %d", frame.point_cloud_size, packet_size);
    return -1;
  }
  memcpy(frame.point_cloud_raw_data + packet_index_use * frame.point_cloud_size, udpPacket.buffer, packet_size);
  frame.packet_num++;
  return 0;
}