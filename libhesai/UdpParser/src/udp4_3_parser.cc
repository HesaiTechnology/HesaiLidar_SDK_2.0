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
void Udp4_3Parser<T_Point>::HandlePacketData(uint8_t *u8Buf, uint16_t u16Len) {
}

template<typename T_Point>
void Udp4_3Parser<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
  int ret = 0;
  printf("load correction file from local correction.csv now!\n");
  std::ifstream fin(lidar_correction_file);
  if (fin.is_open()) {
    printf("Open correction file success\n");
    int length = 0;
    std::string str_lidar_calibration;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    str_lidar_calibration = buffer;
    ret = LoadCorrectionString(buffer);
    if (ret != 0) {
      printf("Parse local Correction file Error\n");
    } else {
      printf("Parse local Correction file Success!!!\n");
    }
  } else {
    printf("Open correction file failed\n");
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
        case 3: {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
          memcpy((void *)&m_PandarAT_corrections.start_frame, p,
                 sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.end_frame, p,
                 sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          for (int i = 0; i < frame_num; ++i)
          memcpy((void *)&m_PandarAT_corrections.azimuth, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.elevation, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.azimuth_offset, p,
                 sizeof(int8_t) * CIRCLE_ANGLE);
          p += sizeof(int8_t) * CIRCLE_ANGLE;
          memcpy((void *)&m_PandarAT_corrections.elevation_offset, p,
                 sizeof(int8_t) * CIRCLE_ANGLE);
          p += sizeof(int8_t) * CIRCLE_ANGLE;
          memcpy((void *)&m_PandarAT_corrections.SHA256, p,
                 sizeof(uint8_t) * 32);
          p += sizeof(uint8_t) * 32;
          this->get_correction_file_ = true;
          return 0;
        } break;
        case 5: {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
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
          for (int i = 0; i < AT128_LASER_NUM; i++) {
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
    std::cerr << e.what() << '\n';
    return -1;
  }
  return -1;
}

template<typename T_Point>
Udp4_3Parser<T_Point>::~Udp4_3Parser() { printf("release Udp4_3Parser\n"); }

template<typename T_Point>
int Udp4_3Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket){
  if(udpPacket.is_timeout == true) {
    for(int i = 0; i < 3; i++) {
      int angle = m_PandarAT_corrections.l.start_frame[i] / ANGULAR_RESOLUTION + MARGINAL_ANGLE - this->last_azimuth_;
      if (abs(angle) < ACCEPTANCE_ANGLE) {
        this->use_angle_ = false;
        output.scan_complete = true;
        output.points_num = 0;
        output.block_num = 0;
        output.laser_num = 0;
        return 0;
      }
    }
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    output.scan_complete = false;
    output.points_num = 0;
    output.block_num = 0;
    output.laser_num = 0;
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
    if(this->enable_packet_loss_tool_ == true) {
      this->current_seqnum_ = pTailSeqNum->m_u32SeqNum;
      if (this->current_seqnum_ > this->last_seqnum_ && this->last_seqnum_ != 0) {
        this->total_packet_count_ += this->current_seqnum_ - this->last_seqnum_;
      }
      pTailSeqNum->CalPktLoss(this->start_seqnum_, this->last_seqnum_, this->loss_count_, 
        this->start_time_, this->total_loss_count_, this->total_start_seqnum_);
    }
  }      
  output.host_timestamp = GetMicroTickCountU64();  
  if (output.use_timestamp_type == 0) {
    output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  } else {
    output.sensor_timestamp = udpPacket.recv_timestamp;
    // printf("udpPacket.recv_timestamp %lu, %lu\n", pTail->GetMicroLidarTimeU64(), udpPacket.recv_timestamp);
  }

  // if(this->enable_packet_loss_tool_ == true) return 0;

  this->spin_speed_ = pTail->GetMotorSpeed();
  this->is_dual_return_= pTail->IsDualReturn();
  output.spin_speed = pTail->m_i16MotorSpeed;
  output.points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  output.scan_complete = false;

  output.distance_unit = pHeader->GetDistUnit();
  int index = 0;
  output.block_num = pHeader->GetBlockNum();
  output.laser_num = pHeader->GetLaserNum();
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
    output.azimuths = u16Azimuth;
    // point to next block fine azimuth addr
    pFineAzimuth = reinterpret_cast<const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3));

    int azimuth = u16Azimuth * ANGULAR_RESOLUTION + u8FineAzimuth;
    auto elevation = 0;
    int field = 0;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      if (this->get_firetime_file_) {
        output.azimuth[index] = azimuth + this->GetFiretimesCorrection(i, this->spin_speed_) * RESOLUTION * 2;
      }else {
        output.azimuth[index] = azimuth;
      }
      output.reflectivities[index] = pChnUnit->GetReflectivity();  
      output.distances[index] = pChnUnit->GetDistance();
      output.elevation[index] = elevation;
      pChnUnit = pChnUnit + 1;
      index++;
    }

    if (this->use_angle_ && IsNeedFrameSplit(u16Azimuth, field)) {
      output.scan_complete = true;
      // printf("IsNeedFrameSplit %d %d\n", u16Azimuth, this->last_azimuth_);
    }
    this->last_azimuth_ = u16Azimuth;
    // printf("%d, %u, %d\n", this->current_seqnum_, u16Azimuth, this->last_azimuth_);
  }
  return 0;
}

template<typename T_Point>
int Udp4_3Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket){
  if(udpPacket.is_timeout == true) {
    for(int i = 0; i < 3; i++) {
      int angle = m_PandarAT_corrections.l.start_frame[i] / ANGULAR_RESOLUTION + MARGINAL_ANGLE - this->last_azimuth_;
      if (abs(angle) < ACCEPTANCE_ANGLE) {
        this->use_angle_ = false;
        frame.scan_complete = true;
        frame.laser_num = 0;
        frame.block_num = 0;
        return 0;
      }
    }
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    printf("Udp4_3Parser: DecodePacket, invalid packet %x %x\n", udpPacket.buffer[0], udpPacket.buffer[1]);
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
  // if (pHeader->HasSeqNum()) {
  //   const HS_LIDAR_TAIL_SEQ_NUM_ST_V3 *pTailSeqNum =
  //       reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ST_V3 *>(
  //           (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V3) +
  //           (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
  //            sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
  //            sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V3) * pHeader->GetLaserNum()) *
  //               pHeader->GetBlockNum() +
  //           sizeof(HS_LIDAR_BODY_CRC_ST_V3) + sizeof(HS_LIDAR_TAIL_ST_V3));      
  // }        
  this->spin_speed_ = pTail->m_i16MotorSpeed;
  this->is_dual_return_= pTail->IsDualReturn();
  frame.laser_num = pHeader->GetBlockNum();
  frame.block_num = pHeader->GetLaserNum();
  frame.points_num += pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.scan_complete = false;
  int index = frame.packet_index * pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // float minAzimuth = 0;
  // float maxAzimuth = 0;
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
    auto elevation = 0;
    int field = 0;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      frame.distances[index] = pChnUnit->GetDistance();
      frame.reflectivities[index] = pChnUnit->GetReflectivity(); 
      frame.azimuth[index] = azimuth; 
      frame.elevation[index] = elevation; 
      frame.azimuths[frame.packet_index] = u16Azimuth;
      pChnUnit = pChnUnit + 1;
      index++;
    }
    if (this->use_angle_ && IsNeedFrameSplit(u16Azimuth, field)) {
      frame.scan_complete = true;
    }
    this->last_azimuth_ = u16Azimuth;
    // if(blockid  == 0 ) minAzimuth =  azimuth;
    // else maxAzimuth = azimuth;
    
  }
  frame.packet_index++;
  return 0;
}

template<typename T_Point>
int Udp4_3Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet){
  for (int blockid = 0; blockid < packet.block_num; blockid++) {
    // T_Point point;
    int Azimuth = packet.azimuth[blockid * packet.laser_num];
    int count = 0, field = 0;
    if ( this->get_correction_file_) {
      while (count < m_PandarAT_corrections.header.frame_number &&
             (((Azimuth + MAX_AZI_LEN - m_PandarAT_corrections.l.start_frame[field]) % MAX_AZI_LEN +
             (m_PandarAT_corrections.l.end_frame[field] + MAX_AZI_LEN - Azimuth) % MAX_AZI_LEN) !=
             (m_PandarAT_corrections.l.end_frame[field] + MAX_AZI_LEN -
             m_PandarAT_corrections.l.start_frame[field]) % MAX_AZI_LEN)) {
        field = (field + 1) % m_PandarAT_corrections.header.frame_number;
        count++;
      }
      if (count >= m_PandarAT_corrections.header.frame_number) continue;
    }
    
    auto elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < packet.laser_num; i++) {
      int point_index = packet.packet_index * packet.points_num + blockid * packet.laser_num + i;  
      float distance = packet.distances[blockid * packet.laser_num + i] * packet.distance_unit;
      Azimuth = packet.azimuth[blockid * packet.laser_num + i];
      if (this->get_correction_file_) {
        elevation = (m_PandarAT_corrections.l.elevation[i] +
                   m_PandarAT_corrections.GetElevationAdjustV3(i, Azimuth) *
                       FINE_AZIMUTH_UNIT );
        elevation = (MAX_AZI_LEN + elevation) % MAX_AZI_LEN;
        azimuth = ((Azimuth + MAX_AZI_LEN - m_PandarAT_corrections.l.start_frame[field]) * 2 -
                         m_PandarAT_corrections.l.azimuth[i] +
                         m_PandarAT_corrections.GetAzimuthAdjustV3(i, Azimuth) * FINE_AZIMUTH_UNIT);
        azimuth = (MAX_AZI_LEN + azimuth) % MAX_AZI_LEN;
      }
      if (packet.config.fov_start != -1 && packet.config.fov_end != -1)
      {
        int fov_transfer = azimuth / 256 / 100;
        if (fov_transfer < packet.config.fov_start || fov_transfer > packet.config.fov_end){//不在fov范围continue
          continue;
        }
      }
      float xyDistance = distance * m_PandarAT_corrections.cos_map[(elevation)];
      float x = xyDistance * m_PandarAT_corrections.sin_map[(azimuth)];
      float y = xyDistance * m_PandarAT_corrections.cos_map[(azimuth)];
      float z = distance * m_PandarAT_corrections.sin_map[(elevation)];
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
int16_t Udp4_3Parser<T_Point>::GetVecticalAngle(int channel) {
  if (this->get_correction_file_ == false) {
    printf ("GetVecticalAngle: no correction file get, Error");
    return -1;
  }
  return m_PandarAT_corrections.elevation[channel];
}

template<typename T_Point>
bool Udp4_3Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth, int field) {
  if (abs(azimuth - this->last_azimuth_) > GeneralParser<T_Point>::kAzimuthTolerance &&
        this->last_azimuth_ != 0 ) {
      return true;
    }
  return false;
}
