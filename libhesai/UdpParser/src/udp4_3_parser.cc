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
 * File:       udp4_3_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp4_3Parser class
*/

#include "udp4_3_parser.h"
#include "udp_protocol_v4_3.h"
#include "udp_protocol_header.h"
#include "udp_parser.h"

using namespace hesai::lidar;
template<typename T_Point>
Udp4_3Parser<T_Point>::Udp4_3Parser() {
  view_mode_ = 1;
  this->get_correction_file_ = false;
}

template<typename T_Point>
void Udp4_3Parser<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
  LogInfo("load correction file from local correction.csv now!");
  std::ifstream fin(lidar_correction_file);
  if (fin.is_open()) {
    LogDebug("Open correction file success");
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = static_cast<int>(fin.tellg());
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    int ret = LoadCorrectionString(buffer);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local Correction file Error");
    } else {
      LogInfo("Parse local Correction file Success!!!");
    }
  } else {
    LogError("Open correction file failed");
    return;
  }
}

template<typename T_Point>
int Udp4_3Parser<T_Point>::LoadCorrectionString(char *data) {
  try {
    char *p = data;
    PandarATCorrectionsHeader header = *(PandarATCorrectionsHeader *)p;
    if (0xee == header.delimiter[0] && 0xff == header.delimiter[1]) {
      switch (header.version[1]) {
        case 5: {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
          if (frame_num > 8 || channel_num > AT128_LASER_NUM) {
            LogError("correction error, frame_num: %u, channel_num: %u", frame_num, channel_num);
            return -1;
          }
          memcpy((void *)&m_PandarAT_corrections.l.start_frame, p,
                 sizeof(uint32_t) * frame_num);
          p += sizeof(uint32_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.l.end_frame, p,
                 sizeof(uint32_t) * frame_num);
          p += sizeof(uint32_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.l.azimuth, p,
                 sizeof(int32_t) * channel_num);
          p += sizeof(int32_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.l.elevation, p,
                 sizeof(int32_t) * channel_num);
          p += sizeof(int32_t) * channel_num;
          auto adjust_length = channel_num * CORRECTION_AZIMUTH_NUM;
          memcpy((void *)&m_PandarAT_corrections.azimuth_offset, p,
                 sizeof(int8_t) * adjust_length);
          p += sizeof(int8_t) * adjust_length;
          memcpy((void *)&m_PandarAT_corrections.elevation_offset, p,
                 sizeof(int8_t) * adjust_length);
          p += sizeof(int8_t) * adjust_length;
          memcpy((void *)&m_PandarAT_corrections.SHA256, p,
                 sizeof(uint8_t) * 32);
          p += sizeof(uint8_t) * 32;
          for (int i = 0; i < frame_num; ++i) {
            m_PandarAT_corrections.l.start_frame[i] =
                m_PandarAT_corrections.l.start_frame[i] *
                m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.l.end_frame[i] =
                m_PandarAT_corrections.l.end_frame[i] *
                m_PandarAT_corrections.header.resolution;
          }
          for (int i = 0; i < channel_num; i++) {
            m_PandarAT_corrections.l.azimuth[i] =
                m_PandarAT_corrections.l.azimuth[i] *
                m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.l.elevation[i] =
                m_PandarAT_corrections.l.elevation[i] *
                m_PandarAT_corrections.header.resolution;
          }
          for (int i = 0; i < adjust_length; i++) {
            m_PandarAT_corrections.azimuth_offset[i] =
                m_PandarAT_corrections.azimuth_offset[i] *
                m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.elevation_offset[i] =
                m_PandarAT_corrections.elevation_offset[i] *
                m_PandarAT_corrections.header.resolution;
          }
          this->get_correction_file_ = true;
          return 0;
        } break;
        default:
          break;
      }
    }
    return -1;
  } catch (const std::exception &e) {
    LogFatal("load correction error: %s", e.what());
    return -1;
  }
  return -1;
}

template<typename T_Point>
Udp4_3Parser<T_Point>::~Udp4_3Parser() { LogInfo("release Udp4_3Parser"); }

template<typename T_Point>
int Udp4_3Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket){
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if(udpPacket.is_timeout == true) {
    for(int i = 0; i < 3; i++) {
      int angle = m_PandarAT_corrections.l.start_frame[i] / ANGULAR_RESOLUTION + MARGINAL_ANGLE - this->last_azimuth_;
      if (abs(angle) < ACCEPTANCE_ANGLE) {
        this->use_angle_ = false;
        frame.scan_complete = true;
        return 0;
      }
    }
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }
  const HS_LIDAR_HEADER_ST_V3 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_ST_V3 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  const HS_LIDAR_TAIL_ST_V3 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_ST_V3 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V3) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
           sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
           sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V3) * pHeader->GetLaserNum()) *
              pHeader->GetBlockNum() +
          sizeof(HS_LIDAR_BODY_CRC_ST_V3));
  if (pHeader->HasSeqNum()) {
    const HS_LIDAR_TAIL_SEQ_NUM_ST_V3 *pTailSeqNum =
        reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ST_V3 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V3) +
            (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
             sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
             sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V3) * pHeader->GetLaserNum()) *
                pHeader->GetBlockNum() +
            sizeof(HS_LIDAR_BODY_CRC_ST_V3) + sizeof(HS_LIDAR_TAIL_ST_V3));
    uint32_t packet_seqnum = pTailSeqNum->m_u32SeqNum;
    this->CalPktLoss(packet_seqnum);
  } 
  uint64_t packet_timestamp = pTail->GetMicroLidarTimeU64();
  this->CalPktTimeLoss(packet_timestamp);    
  frame.host_timestamp = GetMicroTickCountU64();    
  this->spin_speed_ = pTail->GetMotorSpeed();
  this->is_dual_return_= pTail->IsDualReturn();
  frame.spin_speed = pTail->GetMotorSpeed();
  frame.return_mode = pTail->GetReturnMode();
  frame.work_mode = pTail->m_u8Shutdown;
  frame.distance_unit = pHeader->GetDistUnit();

  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }
  frame.laser_num = pHeader->GetLaserNum();
  frame.block_num = pHeader->GetBlockNum();
  frame.per_points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.scan_complete = false;
  int index = frame.packet_num * pHeader->GetBlockNum() * pHeader->GetLaserNum();
  const HS_LIDAR_BODY_AZIMUTH_ST_V3 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ST_V3 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V3));
  const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 *pFineAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3));

  const HS_LIDAR_BODY_CHN_NNIT_ST_V3 *pChnUnit =
      reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_ST_V3 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3));
  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    uint8_t u8FineAzimuth = pFineAzimuth->GetFineAzimuth();
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_ST_V3 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
        sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3));
    // point to next block azimuth addr
    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ST_V3 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
        sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
        sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V3) * pHeader->GetLaserNum());
    // point to next block fine azimuth addr
    pFineAzimuth = reinterpret_cast<const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3));

    int azimuth = u16Azimuth * ANGULAR_RESOLUTION + u8FineAzimuth;
    int field = 0;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      if (this->get_firetime_file_) {
        frame.pointData[index].azimuth = azimuth + this->GetFiretimesCorrection(i, this->spin_speed_) * kAllFineResolutionFloat * 2;
      }else {
        frame.pointData[index].azimuth = azimuth;
      }
      frame.pointData[index].distances = pChnUnit->GetDistance();
      frame.pointData[index].reflectivities = pChnUnit->GetReflectivity(); 
      frame.pointData[index].confidence = pChnUnit->GetConfidenceLevel(); 
      pChnUnit = pChnUnit + 1;
      index++;
    }
    if (IsNeedFrameSplit(u16Azimuth, field)) {
      //超时分帧的情况下，this->last_azimuth_肯定和u16Azimuth差值很大
      //此时通过use_angle_来避免进行错误分帧的情况。
      if(this->use_angle_ == false){
        this->use_angle_ = true;
      }
      else{
        //未设置true会一直显示loading,几秒后超时退出崩溃
        frame.scan_complete = true;
      }
      // printf("IsNeedFrameSplit %d %d\n", u16Azimuth, this->last_azimuth_);
    }
    this->last_azimuth_ = u16Azimuth;    
  }
  frame.packet_num++;
  return 0;
}

template<typename T_Point>
int Udp4_3Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index){
  for (int blockid = 0; blockid < frame.block_num; blockid++) {
    // T_Point point;
    int Azimuth = int(frame.pointData[packet_index * frame.per_points_num + blockid * frame.laser_num].azimuth);
    int field = 0;
    if ( this->get_correction_file_) {
      int count = 0;
      while (count < m_PandarAT_corrections.header.frame_number &&
             (((Azimuth + CIRCLE - m_PandarAT_corrections.l.start_frame[field]) % CIRCLE +
             (m_PandarAT_corrections.l.end_frame[field] + CIRCLE - Azimuth) % CIRCLE) !=
             (m_PandarAT_corrections.l.end_frame[field] + CIRCLE -
             m_PandarAT_corrections.l.start_frame[field]) % CIRCLE)) {
        field = (field + 1) % m_PandarAT_corrections.header.frame_number;
        count++;
      }
      if (count >= m_PandarAT_corrections.header.frame_number) continue;
    }
    
    auto elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < frame.laser_num; i++) {
      int point_index = packet_index * frame.per_points_num + blockid * frame.laser_num + i;  
      float distance = static_cast<float>(frame.pointData[point_index].distances * frame.distance_unit);
      Azimuth = frame.pointData[point_index].azimuth;
      if (this->get_correction_file_) {
        elevation = (m_PandarAT_corrections.l.elevation[i] +
                   m_PandarAT_corrections.GetElevationAdjustV3(i, Azimuth) *
                       kFineResolutionInt );
        elevation = (CIRCLE + elevation) % CIRCLE;
        azimuth = ((Azimuth + CIRCLE - m_PandarAT_corrections.l.start_frame[field]) * 2 -
                         m_PandarAT_corrections.l.azimuth[i] +
                         m_PandarAT_corrections.GetAzimuthAdjustV3(i, Azimuth) * kFineResolutionInt);
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
      setRing(frame.points[point_index], static_cast<uint16_t>(i));
    }
  }
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
int16_t Udp4_3Parser<T_Point>::GetVecticalAngle(int channel) {
  if (this->get_correction_file_ == false) {
    LogError("GetVecticalAngle: no correction file get, Error");
    return -1;
  }
  return m_PandarAT_corrections.elevation[channel];
}

template<typename T_Point>
bool Udp4_3Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth, int field) {
  (void)field;
  if (abs(azimuth - this->last_azimuth_) > GeneralParser<T_Point>::kAzimuthTolerance &&
        this->last_azimuth_ != 0 ) {
      return true;
    }
  return false;
}

template<typename T_Point>
void Udp4_3Parser<T_Point>::ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info) {
  FaultMessageVersion4_3 *fault_message_ptr =  
      reinterpret_cast< FaultMessageVersion4_3*> (&(udp_packet.buffer[0]));
  fault_message_ptr->ParserFaultMessage(fault_message_info);
  return;
}
