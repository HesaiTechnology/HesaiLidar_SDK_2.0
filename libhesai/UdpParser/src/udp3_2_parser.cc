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
 * File:       udp3_2_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp3_2Parser class
*/

#include "udp3_2_parser.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp3_2Parser<T_Point>::Udp3_2Parser() {
  this->optical_center.setNoFlag(LidarOpticalCenter{0.0072, 0.0354, 0});
  this->default_remake_config.min_azi = 0.f;
  this->default_remake_config.max_azi = 360.f;
  this->default_remake_config.ring_azi_resolution = 0.4f;
  this->default_remake_config.max_azi_scan = 900;   // (max_azi - min_azi) / ring_azi_resolution
  this->default_remake_config.min_elev = -52.6f;
  this->default_remake_config.max_elev = 52.6f;
  this->default_remake_config.ring_elev_resolution = 0.4f;
  this->default_remake_config.max_elev_scan = 263;   // (max_elev - min_elev) / ring_elev_resolution
  LogInfo("init 3_2 parser");
}

template<typename T_Point>
Udp3_2Parser<T_Point>::~Udp3_2Parser() { LogInfo("release 3_2 parser"); }

template <typename T_Point>
void Udp3_2Parser<T_Point>::setFrameRightMemorySpace(LidarDecodedFrame<T_Point> &frame) {
  frame.resetMalloc(1800, 512);
}

template<typename T_Point>
int Udp3_2Parser<T_Point>::LoadFiretimesString(const char *firetimes_string, int len) {
  try {
    std::string firetimes_content_str(firetimes_string, len);
    std::istringstream fin(firetimes_content_str);
    std::string line;
    // first line sequence,chn id,firetime/us
    std::getline(fin, line);
    std::vector<std::string> firstLine;
    split_string(firstLine, line, ',');
    if (firstLine[0] == "EEFF" || firstLine[0] == "eeff") {
      memset(&qt128_firetimes.firetimes, 0, sizeof(float) * QT128::Qt128LoopNum * QT128::kMaxChannelQT128);
      std::getline(fin, line);
      std::vector<std::string> loopNumLine;
      split_string(loopNumLine, line, ',');
      horizontal_resolution_mode = atoi(loopNumLine[1].c_str());
      loop_num = atoi(loopNumLine[3].c_str());
      if (loop_num > QT128::Qt128LoopNum) {
        throw std::invalid_argument("firetimes loop num is error : " + std::to_string(loop_num));
      }
      std::getline(fin, line);
      while (std::getline(fin, line)) {
        std::vector<std::string> ChannelLine;
        split_string(ChannelLine, line, ',');
        if (ChannelLine.size() <= 2) continue;
        if (static_cast<int>(ChannelLine.size()) == loop_num * 2) {
          for (int j = 0; j < loop_num; j++) {
            int laserId = atoi(ChannelLine[j * 2].c_str()) - 1;
            if (laserId >= 0 && laserId < QT128::kMaxChannelQT128)
              qt128_firetimes.firetimes[j][laserId] = std::stof(ChannelLine[j * 2 + 1].c_str());
            else if (std::stof(ChannelLine[j * 2 + 1].c_str()) != 0)
              throw std::invalid_argument("firetimes laserId is invalid : " + std::to_string(laserId + 1));
          }
        } else {
          throw std::invalid_argument("loop num is not equal to the first channel line, size:" + std::to_string(ChannelLine.size()));
        }
      }
      this->loadFiretimeSuccess();
    } else {
      throw std::invalid_argument("firetime file delimiter is wrong");
    }
  } catch (const std::exception &e) {
    this->get_firetime_file_ = false;
    LogError("load firetimes error: %s", e.what());
    return -1;
  }
  return 0;
}
template<typename T_Point>
void Udp3_2Parser<T_Point>::LoadFiretimesFile(const std::string& firetimes_path) {
  try {
    std::ifstream fin(firetimes_path, std::ios::in);
    if (fin.is_open()) {
      int length = 0;
      fin.seekg(0, std::ios::end);
      length = static_cast<int>(fin.tellg());
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      fin.read(buffer, length);
      fin.close();
      int ret = LoadFiretimesString(buffer, length);
      delete[] buffer;
      if (ret != 0) {
        LogError("Parse local firetimes file Error");
      }
    } else {
      LogError("Open firetimes file failed");
      return;
    }
  } catch (const std::exception& e) {
    LogFatal("error loading firetimes file: %s", e.what());
  }
}
template<typename T_Point>
int Udp3_2Parser<T_Point>::LoadChannelConfigString(char *channel_config_content) {
  auto &channel_config_ = qt128_firetimes.m_channelConfig;
  try {
    std::string channel_config_content_str = channel_config_content;
    std::istringstream ifs(channel_config_content_str);
    std::string line;

    std::getline(ifs, line);
    std::vector<std::string> versionLine;
    split_string(versionLine, line, ',');
    if (versionLine.size() < 1) throw std::invalid_argument("channel config file first line is error");
    if (versionLine[0] == "EEFF" || versionLine[0] == "eeff") {
      channel_config_.major_version =
          std::stoi(versionLine[1].c_str());
      channel_config_.min_version = std::stoi(versionLine[2].c_str());
    } else {
      throw std::invalid_argument("channel config file delimiter is wrong");
    }
    std::getline(ifs, line);
    std::vector<std::string> channelNumLine;
    split_string(channelNumLine, line, ',');
    if (channelNumLine.size() < 4) throw std::invalid_argument("channel config file second line is error");
    channel_config_.laser_num = std::stoi(channelNumLine[1].c_str());
    channel_config_.m_u8BlockNum = std::stoi(channelNumLine[3].c_str());
    if (channel_config_.laser_num <= 0 || channel_config_.m_u8BlockNum <= 0 ||
        channel_config_.laser_num > QT128::kMaxChannelQT128) {
      throw std::invalid_argument("LaserNum or BlockNum error");
    }
    std::getline(ifs, line);
    std::vector<std::string> firstChannelLine;
    split_string(firstChannelLine, line, ',');
    channel_config_.loopNum = firstChannelLine.size();
    if (channel_config_.loopNum > QT128::Qt128LoopNum) {
      throw std::invalid_argument("loop num is error");
    }

    for (int i = 0; i < channel_config_.laser_num; i++) {
      std::getline(ifs, line);
      std::vector<std::string> ChannelLine;
      split_string(ChannelLine, line, ',');
      if (ChannelLine.size() == channel_config_.loopNum) {
        for (int j = 0; j < channel_config_.loopNum; j++) {
          channel_config_.channelConfigTable[j][i] =
              std::stoi(ChannelLine[j].c_str());
          if (channel_config_.channelConfigTable[j][i] < 1 || channel_config_.channelConfigTable[j][i] > QT128::kMaxChannelQT128) {
            throw std::invalid_argument("channel config laserId is invalid : " + std::to_string(i + 1));
          }
        }
      } else {
        throw std::invalid_argument("loop num is not equal to the first channel line");
      }
    }
    std::getline(ifs, line);
    channel_config_.m_sHashValue = line;
    channel_config_.is_obtained = true;
    (*this->getFiretimeLoadSequenceNum())++;
  } catch (const std::exception &e) {
    channel_config_.is_obtained = false;
    LogFatal("load channel config error: %s", e.what());
    return -1;
  }
  return 0;
}
template<typename T_Point>
int Udp3_2Parser<T_Point>::LoadChannelConfigFile(const std::string channel_config_path) {
  try {
    std::ifstream fin(channel_config_path, std::ios::in);
    if (fin.is_open()) {
      int length = 0;
      fin.seekg(0, std::ios::end);
      length = static_cast<int>(fin.tellg());
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      fin.read(buffer, length);
      fin.close();
      int ret = LoadChannelConfigString(buffer);
      delete[] buffer;
      if (ret != 0) {
        LogError("Parse local channel congfig file Error");
        return -1;
      }
    } else {
      LogError("Open channel congfig file failed");
      return -1;
    }
    return 0;
  } catch (const std::exception& e) {
    LogFatal("error loading channel config file: %s", e.what());
    return -1;
  }
}

template <typename T_Point>
void* Udp3_2Parser<T_Point>::getStruct(const int type) { 
  if (type == CORRECTION_STRUCT)
    return (void*)&(this->correction);
  else if (type == FIRETIME_STRUCT)
    return (void*)&qt128_firetimes;
  return nullptr;
}

template<typename T_Point>
int Udp3_2Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, uint32_t packet_index) {
  if (packet_index >= frame.maxPacketPerFrame || frame.point_cloud_raw_data == nullptr) {
    LogFatal("packet_index(%d) out of %d. or data ptr is nullptr", packet_index, frame.maxPacketPerFrame);
    GeneralParser<T_Point>::FrameNumAdd();
    return -1;
  }
  uint8_t* data = frame.point_cloud_raw_data + packet_index * frame.point_cloud_size;
  const HS_LIDAR_HEADER_QT_V2 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_QT_V2 *>(
          data + sizeof(HS_LIDAR_PRE_HEADER));
  int unitSize = pHeader->unitSize();
  const HS_LIDAR_TAIL_QT_V2 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_QT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + unitSize * 
          pHeader->GetLaserNum()) * pHeader->GetBlockNum() +
          sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
          (hasFunctionSafety(pHeader->m_u8Status) ? sizeof(HS_LIDAR_FUNCTION_SAFETY) : 0));
  
  int point_index = packet_index * frame.per_points_num;
  int point_num = 0;
  auto& packetData = frame.packetData[packet_index];
  bool isSelfDefine = hasSelfDefine(pHeader->m_u8Status);
  bool isHavConfidence = hasConfidence(pHeader->m_u8Status);
  int32_t block_ns_offset = 0;
  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    int current_block_echo_count = ((pHeader->GetEchoCount() > 0 && pHeader->GetEchoNum() > 0) ?
            ((pHeader->GetEchoCount() - 1 + blockid) % pHeader->GetEchoNum() + 1) : 0);
    if (frame.fParam.echo_mode_filter != 0 && current_block_echo_count != 0 && frame.fParam.echo_mode_filter != current_block_echo_count) {
      continue;
    }
    block_ns_offset = QT128C2X::QT128C2X_BLOCK_NS_OFFSET1 + QT128C2X::PandarQT_BLOCK_NS_OFFSET2 * int(blockid / (frame.return_mode < RETURN_MODE_MULTI ? 1 : 2));
    int loopIndex = (pTail->GetModeFlag() + (blockid / ((pTail->GetReturnMode() < 0x39) ? 1 : 2)) + 1) % 2;
    if((isSelfDefine && qt128_firetimes.m_channelConfig.is_obtained)){
      loopIndex = (pTail->GetModeFlag() + (blockid / ((pTail->GetReturnMode() < 0x39) ? 1 : 2)) + 1) % qt128_firetimes.m_channelConfig.loopNum;
    }
    const HS_LIDAR_BODY_AZIMUTH_QT_V2 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) + 
          (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + unitSize * 
          pHeader->GetLaserNum()) * blockid);
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      int channel_index = (isSelfDefine && qt128_firetimes.m_channelConfig.is_obtained && i < qt128_firetimes.m_channelConfig.laser_num) 
                      ? qt128_firetimes.m_channelConfig.channelConfigTable[loopIndex][i] - 1 : i;
      if (this->correction.display[channel_index] == false) {
        continue;
      }
      const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *pChnUnit =
        reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *>(
            (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + 
            unitSize * i);
      uint8_t confidence = 0;
      if (isHavConfidence) confidence = pChnUnit->reserved;
      int elevation = 0;
      int azimuth = u16Azimuth * kFineResolutionInt;
      float distance = static_cast<float>(pChnUnit->GetDistance() * frame.distance_unit);
      if (this->get_firetime_file_ && frame.fParam.firetimes_flag) {
        azimuth += (frame.fParam.rotation_flag > 0 ? 1 : -1) * doubleToInt(
          qt128_firetimes.firetimes[loopIndex][channel_index] * pTail->GetMotorSpeed() * 6E-6 * kAllFineResolutionInt);
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
      if (this->IsChannelFovFilter(azimuth / kAllFineResolutionInt, channel_index, frame.fParam) == 1) continue;
      
      uint64_t timestamp = packetData.t.sensor_timestamp * kMicrosecondToNanosecondInt;
      if (this->get_firetime_file_)
        timestamp += block_ns_offset + floatToInt(qt128_firetimes.firetimes[loopIndex][channel_index] * kMicrosecondToNanosecondInt);
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
        set_confidence(ptinfo, confidence);

        point_num++;
      }
    }
  }
  frame.valid_points[packet_index] = point_num;
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
int Udp3_2Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket, const int packet_index)
{
  uint32_t packet_index_use = (packet_index >= 0 ? packet_index : frame.packet_num);
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ||
      udpPacket.buffer[2] != 3 || udpPacket.buffer[3] != 2) {
    LogDebug("Invalid point cloud");
    return -1;
  }
  frame.scan_complete = false;

  const HS_LIDAR_HEADER_QT_V2 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_QT_V2 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  if (frame.frame_init_ == false) {
    frame.block_num = pHeader->GetBlockNum();
    frame.laser_num = pHeader->GetLaserNum();
    frame.per_points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
    frame.distance_unit = pHeader->GetDistUnit();
    if (frame.per_points_num > frame.maxPointPerPacket) {
      LogFatal("per_points_num(%u) out of %d", frame.per_points_num, frame.maxPointPerPacket);
      return -1;
    }
    if (frame.laser_num > QT128::kMaxChannelQT128) {
      LogFatal("laser_num(%u) out of %d", frame.laser_num, QT128::kMaxChannelQT128);
      return -1;
    }
    frame.frame_init_ = true;
  }
  if (pHeader->GetBlockNum() != frame.block_num
      || pHeader->GetLaserNum() != frame.laser_num
      || pHeader->GetDistUnit() != frame.distance_unit) {
    LogFatal("block_num or laser_num or distance_unit is not match");
    return -1;
  }

  uint16_t u16Azimuth = 0;
  const HS_LIDAR_BODY_AZIMUTH_QT_V2 *pAzimuth =
    reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
        (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2));
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
    
  int unitSize = pHeader->unitSize();
  if(hasFunctionSafety(pHeader->m_u8Status)) {
    const auto *function_savety_ptr = reinterpret_cast<const HS_LIDAR_FUNCTION_SAFETY *>(
      (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) +
      (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + unitSize * pHeader->GetLaserNum()) *
      pHeader->GetBlockNum() + sizeof(HS_LIDAR_BODY_CRC_QT_V2));
    frame.lidar_state = function_savety_ptr->GetLidarState();
    if (frame.fParam.update_function_safety_flag) {
      frame.funcSafety[packet_index_use].is_valid = true;
      frame.funcSafety[packet_index_use].fs_version = function_savety_ptr->m_u8Version;
      frame.funcSafety[packet_index_use].status = function_savety_ptr->m_u8Status;
      frame.funcSafety[packet_index_use].fault_info = function_savety_ptr->m_u8FaultInfo;
      frame.funcSafety[packet_index_use].fault_code = function_savety_ptr->GetFaultCode();
    }
  } else {
    frame.lidar_state = -1;
  }
  const HS_LIDAR_TAIL_QT_V2 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_QT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + unitSize * 
          pHeader->GetLaserNum()) * pHeader->GetBlockNum() +
          sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
          (hasFunctionSafety(pHeader->m_u8Status) ? sizeof(HS_LIDAR_FUNCTION_SAFETY) : 0));

  if (hasSeqNum(pHeader->m_u8Status)) {
    const HS_LIDAR_TAIL_SEQ_NUM_QT_V2 *pTailSeqNum =
      reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_QT_V2 *>(
          (const unsigned char *)pTail + sizeof(HS_LIDAR_TAIL_QT_V2));
    this->CalPktLoss(pTailSeqNum->GetSeqNum(), frame.fParam);
  }
  this->CalPktTimeLoss(pTail->GetMicroLidarTimeU64(this->last_utc_time), frame.fParam);
  frame.return_mode = pTail->GetReturnMode();

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


