/*
 * @Description: 
 * @Date: 2025-08-12 15:10:12
 * @LastEditTime: 2025-09-12 13:55:39
 * @FilePath: /PandarView_Pro_DEMO/3rd-party/HesaiLidar_SDK_2.0/libhesai/UdpParser/include/udp7_3_parser.h
 */
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
 * File:       udp7_3_parser.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Declare Udp7_3Parser class
*/

#ifndef UDP7_3_PARSER_H_
#define UDP7_3_PARSER_H_

#include "general_parser.h"
#include "udp_protocol_v7_3.h"
namespace hesai
{
namespace lidar
{
// class Udp7_3Parser
// parsers packets and computes points for PandarFT120
template<typename T_Point>
class Udp7_3Parser : public GeneralParser<T_Point> {
 public:
  Udp7_3Parser() {
    memset(display, true, sizeof(display));
    LogInfo("init 7_3 parser");
  }

  virtual ~Udp7_3Parser() { LogInfo("release 7_3 Parser "); }

  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket, const int packet_index = -1) {
    uint32_t packet_index_use = (packet_index >= 0 ? packet_index : frame.packet_num);
    if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ||
        udpPacket.buffer[2] != 7 || udpPacket.buffer[3] != 3) {
      LogDebug("Invalid point cloud");
      return -1;
    }
    frame.scan_complete = false;
    frame.imu_config.flag = false;
    const HS_LIDAR_PRE_HEADER *pPreHeader =
        reinterpret_cast<const HS_LIDAR_PRE_HEADER *>(
            &(udpPacket.buffer[0]));
    if (pPreHeader->m_u8Reserved1 == 1) {
      if (udpPacket.packet_len != 34) {
        LogWarning("Invalid imu, V7_3 packet size: %d, expected: 34", udpPacket.packet_len);
        return -1;
      }
      const HS_LIDAR_HEADER_IMU_FT_V3 *pHeader =
        reinterpret_cast<const HS_LIDAR_HEADER_IMU_FT_V3 *>(
            (const unsigned char *)pPreHeader + sizeof(HS_LIDAR_PRE_HEADER));
      const HS_LIDAR_BODY_IMU_FT_V3 *pBody =
        reinterpret_cast<const HS_LIDAR_BODY_IMU_FT_V3 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_IMU_FT_V3));
      if (frame.fParam.use_timestamp_type == 0) {
        const uint64_t sample_us =
            static_cast<uint64_t>(pHeader->GetMicroLidarTimeU64(this->last_utc_time));
        frame.imu_config.sample_timestamp_us = sample_us;
        frame.imu_config.timestamp = double(sample_us) / kMicrosecondToSecond;
      } else {
        frame.imu_config.sample_timestamp_us = udpPacket.recv_timestamp;
        frame.imu_config.timestamp = double(udpPacket.recv_timestamp) / kMicrosecondToSecond;
      }  
      frame.imu_config.imu_accel_x = pBody->GetIMUXAccel();
      frame.imu_config.imu_accel_y = pBody->GetIMUYAccel();
      frame.imu_config.imu_accel_z = pBody->GetIMUZAccel();
      frame.imu_config.imu_ang_vel_x = pBody->GetIMUXAngVel();
      frame.imu_config.imu_ang_vel_y = pBody->GetIMUYAngVel();
      frame.imu_config.imu_ang_vel_z = pBody->GetIMUZAngVel();
      frame.imu_config.flag = true;
      return 1;
    }
    const HS_LIDAR_HEADER_FT_V3 *pHeader =
        reinterpret_cast<const HS_LIDAR_HEADER_FT_V3 *>(
            &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

    if (frame.frame_init_ == false) {
      frame.block_num = 1;
      frame.laser_num = pHeader->total_row;
      frame.channel_num = HEIGHT_MAX;
      frame.per_points_num = pHeader->GetChannelNum();
      frame.distance_unit = pHeader->GetDistUnit();
      if (frame.per_points_num > frame.maxPointPerPacket) {
        LogFatal("per_points_num(%u) out of %d", frame.per_points_num, frame.maxPointPerPacket);
        return -1;
      }
      frame.frame_init_ = true;
    } else if (pHeader->total_row != frame.laser_num
        || pHeader->GetDistUnit() != frame.distance_unit
        || pHeader->GetChannelNum() != frame.per_points_num) {
      // FT V3 变焦/非变焦切换时，包头 total_row / total_column 与每包点数会变；
      // 若仍用首包缓存会误报 Fatal。此处丢弃当前未拼完的帧并重新对齐新配置。
      LogWarning(
          "FT V3 header changed (zoom/normal switch or dist unit): "
          "row %u->%u, dist_unit %g->%g, chn %u->%u; reset raw frame buffer",
          static_cast<unsigned>(frame.laser_num),
          static_cast<unsigned>(pHeader->total_row),
          frame.distance_unit,
          pHeader->GetDistUnit(),
          static_cast<unsigned>(frame.per_points_num),
          static_cast<unsigned>(pHeader->GetChannelNum()));
      frame.laser_num = pHeader->total_row;
      frame.distance_unit = pHeader->GetDistUnit();
      frame.per_points_num = pHeader->GetChannelNum();
      if (frame.per_points_num > frame.maxPointPerPacket) {
        LogFatal("per_points_num(%u) out of %d", frame.per_points_num, frame.maxPointPerPacket);
        return -1;
      }
      frame.packet_num = 0;
      frame.frame_start_timestamp = 0;
      if (frame.point_cloud_raw_data != nullptr) {
        delete[] frame.point_cloud_raw_data;
        frame.point_cloud_raw_data = nullptr;
      }
      frame.point_cloud_size = 0;
      this->last_max_packet_num_ = 0;
    }

    const HS_LIDAR_TAIL_FT_V3 *pTail =
        reinterpret_cast<const HS_LIDAR_TAIL_FT_V3 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V3) +
            (sizeof(HS_LIDAR_BODY_CHN_UNIT_FT_V3) * pHeader->GetChannelNum()));  
    
    if (IsNeedFrameSplit(pTail->frame_id)) {
      frame.scan_complete = true;
    }
    this->last_frame_id_ = pTail->frame_id;
    if (frame.scan_complete)
      return 0;
    
    this->CalPktLoss(pTail->GetSeqNum(), frame.fParam);
    this->CalPktTimeLoss(pTail->GetMicroLidarTimeU64(this->last_utc_time), frame.fParam);

    frame.return_mode = pTail->GetReturnMode();

    if (frame.fParam.use_timestamp_type == 0) {
      frame.packetData[packet_index_use].t.sensor_timestamp = pTail->GetMicroLidarTimeU64(this->last_utc_time);
    } else {
      frame.packetData[packet_index_use].t.sensor_timestamp = udpPacket.recv_timestamp;
    }   
    if (frame.frame_start_timestamp == 0) frame.frame_start_timestamp = double(frame.packetData[packet_index_use].t.sensor_timestamp) / kMicrosecondToSecond;
    frame.frame_end_timestamp = double(frame.packetData[packet_index_use].t.sensor_timestamp) / kMicrosecondToSecond;
    m_maxColMin = pHeader->total_column / PIXEL_COL_V3;
 
    auto current_block_echo = pHeader->echo;
    if (frame.fParam.echo_mode_filter != 0 && current_block_echo != 0 && frame.fParam.echo_mode_filter != current_block_echo) {
      return -1;
    }

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

  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, uint32_t packet_index) {
    if (packet_index >= frame.maxPacketPerFrame || frame.point_cloud_raw_data == nullptr) {
      LogFatal("packet_index(%d) out of %d. or data ptr is nullptr", packet_index, frame.maxPacketPerFrame);
      GeneralParser<T_Point>::FrameNumAdd();
      return -1;
    }
    uint8_t* data = frame.point_cloud_raw_data + packet_index * frame.point_cloud_size;
    const HS_LIDAR_HEADER_FT_V3 *pHeader =
        reinterpret_cast<const HS_LIDAR_HEADER_FT_V3 *>(
            data + sizeof(HS_LIDAR_PRE_HEADER));
    const HS_LIDAR_TAIL_FT_V3 *pTail =
        reinterpret_cast<const HS_LIDAR_TAIL_FT_V3 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V3) +
            (sizeof(HS_LIDAR_BODY_CHN_UNIT_FT_V3) * pHeader->GetChannelNum()));  
    
    // 通过 total_row 和 total_column 判断模式
    bool is_zoom_mode = (pHeader->total_row == ZOOM_HEIGHT && pHeader->total_column == ZOOM_WIDTH);
    int miniflash_row_max = is_zoom_mode ? ZOOM_MINIFLASH_ROW_V3 : MINIFLASH_ROW_V3;
    int miniflash_col_max = is_zoom_mode ? ZOOM_MINIFLASH_COL_V3 : MINIFLASH_COL_V3;
    int row_pixel_offset = is_zoom_mode ? ZOOM_ROW_OFFSET * PIXEL_ROW_V3 : 0;
    int col_pixel_offset = is_zoom_mode ? ZOOM_COL_OFFSET * PIXEL_COL_V3 : 0;

    if (pTail->row_id >= miniflash_row_max || pTail->column_id >= miniflash_col_max) {
      LogError("invalid row_id(%d) or column_id(%d), zoom_mode=%d", pTail->row_id, pTail->column_id, is_zoom_mode);
      frame.valid_points[packet_index] = 0;
      GeneralParser<T_Point>::FrameNumAdd();
      return -1;
    }

    int point_index = packet_index * frame.per_points_num;
    int point_num = 0;
    auto& packetData = frame.packetData[packet_index];
    if (frame.fParam.remake_config.flag) {
      frame.fParam.remake_config.max_azi_scan = is_zoom_mode ? WIDTH_MAX : pHeader->total_column;
      frame.fParam.remake_config.max_elev_scan = is_zoom_mode ? HEIGHT_MAX : pHeader->total_row;
    }

    const HS_LIDAR_BODY_CHN_UNIT_FT_V3 *pChnUnit =
        reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_FT_V3 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V3));
    for (int i = 0; i < pHeader->GetChannelNum(); i++) {
      int row_id = pTail->row_id * PIXEL_ROW_V3 + (i % PIXEL_ROW_V3) + row_pixel_offset;
      int col_id = pTail->column_id * PIXEL_COL_V3 + int(i / PIXEL_ROW_V3) + col_pixel_offset;
      if (display[row_id * FT::FT2_CORRECTION_LEN + col_id] == false) {
        pChnUnit++;
        continue;
      }
      float distance = static_cast<float>(pChnUnit->GetDistance() * frame.distance_unit); 
      int row_id_use = row_id;
      if (pHeader->configuration_number == 1 || pHeader->configuration_number == 3) {
        row_id_use = HEIGHT_MAX - row_id - 1;
      }
      float x = 0, y = 0, z = 0;
      float corr_x = 0, corr_y = 0, corr_z = 0;
      if (this->get_correction_file_) {   
        int offset = row_id_use * FT::FT2_CORRECTION_LEN + col_id;
        corr_x = corrections_[offset].x;
        corr_y = corrections_[offset].y;
        corr_z = corrections_[offset].z;
        x = distance * corr_x;
        y = distance * corr_y;
        z = distance * corr_z;
        if (corr_x == 0 && corr_y == 0 && corr_z == 0) {
          pChnUnit++;
          continue;
        }
      }
      int azimuth = atan2(corr_x, corr_y) / M_PI * 180 * kAllFineResolutionInt;
      if (this->IsChannelFovFilter(azimuth / kAllFineResolutionInt, row_id, frame.fParam) == 1) {
        pChnUnit++;
        continue;
      }
      this->TransformPoint(x, y, z, frame.fParam.transform);

      int point_index_rerank = point_index + point_num; 
      if (frame.fParam.remake_config.flag) {
        point_index_rerank = col_id * frame.fParam.remake_config.max_elev_scan + row_id_use;
      }
      if(point_index_rerank >= 0) { 
        auto& ptinfo = frame.points[point_index_rerank]; 
        set_x(ptinfo, x); 
        set_y(ptinfo, y); 
        set_z(ptinfo, z); 
        set_ring(ptinfo, row_id); 
        set_intensity(ptinfo, pChnUnit->GetReflectivity());  
        set_timestamp(ptinfo, double(packetData.t.sensor_timestamp) / kMicrosecondToSecond);
        set_confidence(ptinfo, pChnUnit->GetConfidenceLevel());
        set_azimuth(ptinfo, static_cast<float>(azimuth) / kAllFineResolutionFloat); 
        set_azimuthCalib_lazy(ptinfo, [azimuth]() {return static_cast<float>(azimuth) / kAllFineResolutionFloat; }); 
        set_elevation_lazy(ptinfo, [corr_x, corr_y, corr_z]() { return static_cast<float>(atan2(corr_z, sqrt(corr_x * corr_x + corr_y * corr_y)) / M_PI * 180.0); }); 
        set_elevationCalib_lazy(ptinfo, [corr_x, corr_y, corr_z]() { return static_cast<float>(atan2(corr_z, sqrt(corr_x * corr_x + corr_y * corr_y)) / M_PI * 180.0); }); 
        set_distance(ptinfo, distance); 

        point_num++;
      }
      pChnUnit++;
    }
    frame.valid_points[packet_index] = point_num;
    GeneralParser<T_Point>::FrameNumAdd();
    return 0;
  }

  // get lidar correction file from local file,and pass to udp parser 
  virtual void LoadCorrectionFile(const std::string& correction_path) {
    try {
      int type = 0;
      size_t length = correction_path.length();
      if (length >= 4) {
        std::string extension = correction_path.substr(length - 4);
        if (extension == ".bin" || extension == ".dat") {
            type = 1; //  .bin
        } else if (extension == ".csv") {
            type = 2; //  .csv
        } else {
            // type = 0; //  wrong
            return;
        }   
      }

      LogInfo("load correction file from local correction now!");
      if (type == 1) {
        std::ifstream fin(correction_path, std::ios::binary);
        if (fin.is_open()) {
          fin.seekg(0, std::ios::end);
          int len = static_cast<int>(fin.tellg());
          // return the begin of file
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[len];
          // file --> buffer
          fin.read(buffer, len);
          fin.close();
          int ret = 0;
          ret = LoadCorrectionString(buffer, len);
          delete[] buffer;
          if (ret != 0) {
            LogError("Parse local Correction file Error!");
          } else {
            LogInfo("Parse local Correction file Success!!!");
          }
        } else { // open failed
          LogError("Open correction file failed");
          return;
        }
      }
      else if (type == 2) {
        std::ifstream fin(correction_path, std::ios::in);
        if (fin.is_open()) {
          fin.seekg(0, std::ios::end);
          int len = static_cast<int>(fin.tellg());
          // return the begin of file
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[len];
          // file --> buffer
          fin.read(buffer, len);
          fin.close();
          int ret = LoadCorrectionCsvData(buffer, len);
          delete[] buffer;
          if (ret != 0) {
            LogError("Parse local Correction file Error!");
          } else {
            LogInfo("Parse local Correction file Success!!!");
          }
        } else { // open failed
          LogError("Open correction file failed");
          return;
        }
      }
      else {
        LogError("Invalid suffix name");
        return;
      }
    } catch (const std::exception& e) {
      LogFatal("error loading correction file: %s", e.what());
    }
  }

  virtual int LoadCorrectionString(const char *correction_string, int len) {
    try {
      uint8_t *correction_string_ptr = (uint8_t *)correction_string;
      if (correction_string_ptr[0] == 0xEE && correction_string_ptr[1] == 0xFF) {
        if (static_cast<size_t>(len) < sizeof(FT::CorrectionV3)) {
          throw std::invalid_argument("V2 correction string length is too short:" + std::to_string(len) + ", expected: " + std::to_string(sizeof(FT::CorrectionV3)));
        }
        memcpy(&correction_v3_, correction_string_ptr, sizeof(FT::CorrectionV3));
        if (correction_v3_.m_u8VersionMajor != 0x07 || 
            correction_v3_.m_u8VersionMinor != 0x03) {
          throw std::invalid_argument("invalid correction string");
        }
        memset(corrections_, 0, sizeof(FT::CorrectionDis) * FT::FT2_CORRECTION_LEN * FT::FT2_CORRECTION_LEN);
        auto pixel_vectors = correction_v3_.generate_pixel_vectors();
        for (int i = 0; i < correction_v3_.m_u16TotalRow; i++) {
          for (int j = 0; j < correction_v3_.m_u16TotalCol; j++) {
            corrections_[i * FT::FT2_CORRECTION_LEN + j].x = pixel_vectors[i * correction_v3_.m_u16TotalCol + j][0];
            corrections_[i * FT::FT2_CORRECTION_LEN + j].y = pixel_vectors[i * correction_v3_.m_u16TotalCol + j][1];
            corrections_[i * FT::FT2_CORRECTION_LEN + j].z = pixel_vectors[i * correction_v3_.m_u16TotalCol + j][2];
          }
        }
        this->loadCorrectionSuccess();
      }
      else {
        throw std::invalid_argument("correction dat is only support EE FF format");
      }
    } catch (const std::exception &e) {
      LogFatal("load correction error: %s", e.what());
      this->get_correction_file_ = false;
      return -1;
    }
    return 0;
  }

  int LoadCorrectionCsvData(const char *correction_string, int len) {
    try {
      memset(corrections_, 0, sizeof(FT::CorrectionDis) * FT::FT2_CORRECTION_LEN * FT::FT2_CORRECTION_LEN);
      std::string correction_string_str(correction_string, len);
      std::istringstream ifs(correction_string_str);
      std::string line;
      // first line "Laser id,Elevation,Azimuth"
      std::getline(ifs, line);
      int lineCounter = 0;
      std::vector<std::string>  firstLine;
      split_string(firstLine, line, ',');
      int rowIdMax = 0, cloumnIdMax = 0;
      while (std::getline(ifs, line)) {
        if(line.length() < strlen("1,1,1,1,1")) {
          continue;
        } 
        else {
          lineCounter++;
        }
        float x = 0.0f, y = 0.0f, z = 0.0f;  // 初始化
        int rowId = 0;
        int cloumnId = 0;
        std::stringstream ss(line);
        std::string subline;
        // 添加解析成功检查
        if (!(std::getline(ss, subline, ',') && (std::stringstream(subline) >> rowId) &&
              std::getline(ss, subline, ',') && (std::stringstream(subline) >> cloumnId) &&
              std::getline(ss, subline, ',') && (std::stringstream(subline) >> x) &&
              std::getline(ss, subline, ',') && (std::stringstream(subline) >> y) &&
              std::getline(ss, subline, ',') && (std::stringstream(subline) >> z))) {
          // LogError("Invalid correction line format: %s", line.c_str());
          continue;
        }
        // 添加负数检查
        if (rowId < 0 || cloumnId < 0 || 
            rowId > FT::FT2_CORRECTION_LEN || cloumnId > FT::FT2_CORRECTION_LEN) {
          throw std::invalid_argument("Invalid correction file, out of range");
        }
        if (rowId % 2 == 1 && cloumnId % 2 == 1) {
          rowId /= 2;
          cloumnId /= 2;
          corrections_[rowId * FT::FT2_CORRECTION_LEN + cloumnId].x = x;
          corrections_[rowId * FT::FT2_CORRECTION_LEN + cloumnId].y = y;
          corrections_[rowId * FT::FT2_CORRECTION_LEN + cloumnId].z = z;
          rowIdMax = rowIdMax > rowId ? rowIdMax : rowId;
          cloumnIdMax = cloumnIdMax > cloumnId ? cloumnIdMax : cloumnId;
        }
      }
      this->loadCorrectionSuccess();
      correction_v3_.m_u16TotalRow = rowIdMax + 1;
      correction_v3_.m_u16TotalCol = cloumnIdMax + 1;
    } catch (const std::exception &e) {
      LogFatal("load correction error: %s", e.what());
      this->get_correction_file_ = false;
      return -1;
    }
    return 0;
  }

  virtual void LoadFiretimesFile(const std::string& firetimes_path) {
    LogWarning("don't support firetimes file");
  }

  // get the pointer to the struct of the parsed correction file or firetimes file
  virtual void* getStruct(const int type) {
    if (type == CORRECTION_STRUCT)
      return (void*)&(corrections_);
    else if (type == FIRETIME_STRUCT)
      return nullptr;
    return nullptr;
  }

  // get display 
  virtual int getDisplay(bool **display_) {
    *display_ = display;
    return FT::FT2_CORRECTION_LEN * FT::FT2_CORRECTION_LEN;
  }

  // determine whether frame splitting is needed
  bool IsNeedFrameSplit(uint16_t frame_id) {
    if (this->last_frame_id_ != -1 && frame_id != this->last_frame_id_) {
      return true;
    }
    return false;
  }

  virtual void setFrameRightMemorySpace(LidarDecodedFrame<T_Point> &frame) {
    frame.resetMalloc(3000, 192);
  }

  virtual int ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info) {
    FaultMessageVersion7_3 *fault_message_ptr =  
        reinterpret_cast< FaultMessageVersion7_3*> (&(udp_packet.buffer[0]));
    fault_message_ptr->ParserFaultMessage(fault_message_info, this->last_utc_time);
    return 0;
  }

  virtual int FrameProcess(LidarDecodedFrame<T_Point> &frame) {
    return 0;
  }




 private:
  int m_maxColMin = 16;
  int last_frame_id_ = -1;
  FT::CorrectionDis corrections_[FT::FT2_CORRECTION_LEN * FT::FT2_CORRECTION_LEN];
  bool display[FT::FT2_CORRECTION_LEN * FT::FT2_CORRECTION_LEN];
  FT::CorrectionV3 correction_v3_;
};
}  // namespace lidar
}  // namespace hesai

#endif  // UDP7_3_PARSER_H_
