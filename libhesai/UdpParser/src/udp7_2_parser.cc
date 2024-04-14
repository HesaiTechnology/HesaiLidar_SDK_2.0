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
 * File:       udp7_2_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp7_2Parser class
*/

#include "udp7_2_parser.h"
#include "udp_protocol_v7_2.h"
#include "udp_protocol_header.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp7_2Parser<T_Point>::Udp7_2Parser() {
  this->motor_speed_ = 0;
  this->return_mode_ = 0;
  this->last_cloumn_id_ = -1;
}
template<typename T_Point>
Udp7_2Parser<T_Point>::~Udp7_2Parser() { printf("release Udp7_2Parser \n"); }


template<typename T_Point>
void Udp7_2Parser<T_Point>::LoadCorrectionFile(std::string correction_path) {
  int ret = 0;
  printf("load correction file from local correction.csv now!\n");
  std::ifstream fin(correction_path);
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
int Udp7_2Parser<T_Point>::LoadCorrectionString(char *data) {
  if (LoadCorrectionDatData(data) == 0) {
    return 0;
  }
  return LoadCorrectionCsvData(data);
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::LoadCorrectionCsvData(char *correction_string) {
  std::istringstream ifs(correction_string);
	std::string line;
  // first line "Laser id,Elevation,Azimuth"
	if(std::getline(ifs, line)) {  
		printf("Parse Lidar Correction...\n");
	}
	int lineCounter = 0;
	std::vector<std::string>  firstLine;
  split_string(firstLine, line, ',');
  while (std::getline(ifs, line)) {
    if(line.length() < strlen("1,1,1,1")) {
      return -1;
    } 
    else {
      lineCounter++;
    }
    float elev, azimuth;
    int lineId = 0;
    int cloumnId = 0;
    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> cloumnId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;
    corrections_.elevations[lineId - 1][cloumnId - 1] = elev * kResolutionInt;
    corrections_.azimuths[lineId - 1][cloumnId - 1] = azimuth * kResolutionInt;
  }
  this->get_correction_file_ = true;
	return 0;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::LoadCorrectionDatData(char *correction_string) {

  try {
    char *p = correction_string;
    PandarFTCorrectionsHeader header = *(PandarFTCorrectionsHeader *)p;
    if (0xee == header.pilot[0] && 0xff == header.pilot[1]) {
      switch (header.version[1]) {
        case 0: {
          int column_num = header.column_number;
          int channel_num = header.channel_number;
          int resolution = header.resolution;
          float fResolution = float(resolution);
          int angleNum = column_num * channel_num;
          int doubleAngleNum = angleNum * 2;
          int16_t* angles = new int16_t[doubleAngleNum]{0};
          int readLen = sizeof(int16_t) * doubleAngleNum;
          memcpy((void*)angles, correction_string, readLen);
          int hashLen = 32;
          uint8_t* hashValue = new uint8_t[hashLen];
          memcpy((void*)hashValue, correction_string + readLen, hashLen);
          for (int row = 0; row < column_num; row++) {
              for (int col = 0; col < channel_num; col++) {
                  int idx = row * channel_num + col;
                  corrections_.azimuths[col][row] = angles[idx] * fResolution;
              }
          }

          for (int row = 0; row < column_num; row++) {
              for (int col = 0; col < channel_num; col++) {
                  int idx = angleNum + row * channel_num + col;
                  corrections_.elevations[col][row] = angles[idx] * fResolution;
              }
          }
          this->get_correction_file_ = true;
          return 0;
        } break;
        case 1: {
          int column_num = header.column_number;
          int channel_num = header.channel_number;
          int resolution = header.resolution;
          float fResolution = float(resolution);
          int angleNum = column_num * channel_num;
          int doubleAngleNum = angleNum * 2;
          int32_t* angles = new int32_t[doubleAngleNum]{0};
          int readLen = sizeof(int32_t) * doubleAngleNum;
          memcpy((void*)angles, correction_string + sizeof(PandarFTCorrectionsHeader), readLen);
          int hashLen = 32;
          uint8_t* hashValue = new uint8_t[hashLen];
          memcpy((void*)hashValue, correction_string + readLen + sizeof(PandarFTCorrectionsHeader), hashLen);
          for (int row = 0; row < column_num; row++) {
              for (int col = 0; col < channel_num; col++) {
                  int idx = row * channel_num + col;
                  corrections_.azimuths[col][row] = angles[idx] * fResolution;
              }
          }

          for (int row = 0; row < column_num; row++) {
              for (int col = 0; col < channel_num; col++) {
                  int idx = angleNum + row * channel_num + col;
                  corrections_.elevations[col][row] = angles[idx] * fResolution;
              }
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
int Udp7_2Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  // T_Point point;
  for (int i = 0; i < packet.laser_num; i++) {
    int point_index = packet.packet_index * packet.points_num + i;
    float distance = packet.distances[i] * packet.distance_unit;
    int azimuth = 0;
    int elevation = 0;   
    if (this->get_correction_file_) {
      azimuth = packet.azimuth[i];
      elevation = packet.elevation[i];
      elevation = (CIRCLE + elevation) % CIRCLE;
      azimuth = (CIRCLE + azimuth) % CIRCLE;
    }
    if (packet.config.fov_start != -1 && packet.config.fov_end != -1)
    {
      int fov_transfer = azimuth / 256 / 100;
      if (fov_transfer < packet.config.fov_start || fov_transfer > packet.config.fov_end){//不在fov范围continue
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
    setIntensity(frame.points[point_index], packet.reflectivities[i]);
    setTimestamp(frame.points[point_index], double(packet.sensor_timestamp) / kMicrosecondToSecond);
    setRing(frame.points[point_index], i);
  }
  frame.points_num += packet.points_num;
  frame.packet_num = packet.packet_index;
  return 0;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }
  const HS_LIDAR_HEADER_FT_V2 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_FT_V2 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  const HS_LIDAR_TAIL_FT_V2 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_FT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V2) +
          (sizeof(HS_LIDAR_BODY_CHN_UNIT_FT_V2) * pHeader->GetChannelNum()));  
  if (output.use_timestamp_type == 0) {
    output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  } else {
    output.sensor_timestamp = udpPacket.recv_timestamp;
  }
  output.host_timestamp = GetMicroTickCountU64();

  if (this->enable_packet_loss_tool_ == true) {
    this->current_seqnum_ = pTail->sequence_num;
    if (this->current_seqnum_ > this->last_seqnum_ && this->last_seqnum_ != 0) {
      this->total_packet_count_ += this->current_seqnum_ - this->last_seqnum_;
    }
    pTail->CalPktLoss(this->start_seqnum_, this->last_seqnum_, this->loss_count_, 
        this->start_time_, this->total_loss_count_, this->total_start_seqnum_);
    return 0;
  }    

  output.maxPoints = pHeader->GetChannelNum();
  output.points_num = pHeader->GetChannelNum();
  output.scan_complete = false;
  output.distance_unit = pHeader->GetDistUnit();
  int index = 0;
  // float minAzimuth = 0;
  // float maxAzimuth = 0;
  output.block_num = 1;
  output.laser_num = pHeader->GetChannelNum();

  const HS_LIDAR_BODY_CHN_UNIT_FT_V2 *pChnUnit =
      reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_FT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V2));
  for (int blockid = 0; blockid < 1; blockid++) {
    for (int i = 0; i < pHeader->GetChannelNum(); i++) {
      output.azimuth[index] = corrections_.azimuths[i][pTail->column_id];
      output.elevation[index] = corrections_.elevations[i][pTail->column_id];
      output.reflectivities[index] = pChnUnit->GetReflectivity();  
      output.distances[index] = pChnUnit->GetDistance();
      pChnUnit = pChnUnit + 1;
      index++;
    }
  }
  if (IsNeedFrameSplit(pTail->column_id, pHeader->total_column)) {
    output.scan_complete = true;
  }
  this->last_cloumn_id_ = pTail->column_id;
  return 0;
}  

template<typename T_Point>
bool Udp7_2Parser<T_Point>::IsNeedFrameSplit(uint16_t column_id, uint16_t total_column) {
  if (column_id < this->last_cloumn_id_) {
      return true;
    }
  return false;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
{
  // TO DO
  return 0;
}