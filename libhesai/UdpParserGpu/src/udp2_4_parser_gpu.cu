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

#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_device_runtime_api.h>

#include "udp2_4_parser_gpu.h"
#include "safe_call.cuh"
#include "return_code.h"

using namespace hesai::lidar;
template <typename T_Point>
Udp2_4ParserGpu<T_Point>::Udp2_4ParserGpu() {
  corrections_loaded_ = false;
  cudaSafeMalloc(point_data_cu_, POINT_DATA_LEN);
  cudaSafeMalloc(sensor_timestamp_cu_, SENSOR_TIMESTAMP_LEN);
}
template <typename T_Point>
Udp2_4ParserGpu<T_Point>::~Udp2_4ParserGpu() {
  cudaSafeFree(point_data_cu_);
  cudaSafeFree(sensor_timestamp_cu_);
  if (corrections_loaded_) {
    cudaSafeFree(channel_elevations_cu_);
    cudaSafeFree(channel_azimuths_cu_);
    corrections_loaded_ = false;
  }
}
template <typename T_Point>
__global__ void compute_xyzs_2_4_impl(T_Point *xyzs, const float* channel_azimuths, const float* channel_elevations, const PointDecodeData* point_data, const uint64_t* sensor_timestamp,
    const double raw_distance_unit, Transform transform, const uint16_t blocknum, const uint16_t lasernum, const uint16_t packet_index) {
  auto iscan = blockIdx.x;
  auto ichannel = threadIdx.x;
  if (iscan >= packet_index || ichannel >= blocknum * lasernum) return;
  int point_index = iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum));
  float apha =  channel_elevations[0];
  float beta =  channel_elevations[1];
  float gamma =  channel_elevations[2];
  float raw_azimuth = point_data[point_index].azimuth;
  float raw_elevation = point_data[point_index].elevation;
  float phi = channel_azimuths[(ichannel % lasernum) + 3];
  float theta = channel_elevations[(ichannel % lasernum) + 3];
  float an = apha + phi;
  float theta_n = (raw_elevation + theta / std::cos(an * M_PI / 180));
  float elv_v = raw_elevation * M_PI / 180 + theta * M_PI / 180 - std::tan(raw_elevation * M_PI / 180) * (1 - std::cos(an * M_PI / 180)) ;
  float delt_azi_v = std::sin(an * M_PI / 180) * std::cos(an * M_PI / 180) * theta_n * theta_n / 2  * 1.016 * M_PI / 180 * M_PI / 180;
  float eta = phi + delt_azi_v * 180 / M_PI + beta + raw_azimuth / 2;
  float delt_azi_h = std::sin(eta * M_PI / 180) * std::tan(2 * gamma * M_PI / 180) * std::tan(elv_v ) + std::sin(2 * eta * M_PI / 180) * gamma * gamma * M_PI / 180 * M_PI / 180;
  float elv_h = elv_v * 180 / M_PI + std::cos(eta * M_PI / 180) * 2 * gamma ;
  float azi_h = 90 +  raw_azimuth + delt_azi_h * 180 / M_PI + delt_azi_v * 180 / M_PI + phi;

  auto rho = point_data[point_index].distances * raw_distance_unit;
  float z = rho * std::sin(elv_h * M_PI / 180);
  auto r = rho * std::cos(elv_h * M_PI / 180) ;
  float x = r * std::sin(azi_h * M_PI / 180);
  float y = r * std::cos(azi_h * M_PI / 180);

  float cosa = std::cos(transform.roll);
  float sina = std::sin(transform.roll);
  float cosb = std::cos(transform.pitch);
  float sinb = std::sin(transform.pitch);
  float cosc = std::cos(transform.yaw);
  float sinc = std::sin(transform.yaw);

  float x_ = cosb * cosc * x + (sina * sinb * cosc - cosa * sinc) * y +
              (sina * sinc + cosa * sinb * cosc) * z + transform.x;
  float y_ = cosb * sinc * x + (cosa * cosc + sina * sinb * sinc) * y +
              (cosa * sinb * sinc - sina * cosc) * z + transform.y;
  float z_ = -sinb * x + sina * cosb * y + cosa * cosb * z + transform.z;
  gpu::setX(xyzs[point_index], x_);
  gpu::setY(xyzs[point_index],  y_);
  gpu::setZ(xyzs[point_index], z_);
  gpu::setIntensity(xyzs[point_index], point_data[point_index].reflectivities);
  gpu::setTimestamp(xyzs[point_index], double(sensor_timestamp[iscan]) / kMicrosecondToSecond);
  gpu::setRing(xyzs[point_index], ichannel % lasernum);
  gpu::setConfidence(xyzs[point_index], point_data[point_index].confidence);
}
template <typename T_Point>
int Udp2_4ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!corrections_loaded_) return int(ReturnCode::CorrectionsUnloaded);          
  cudaSafeCall(cudaMemcpy(point_data_cu_, frame.pointData,
                          frame.block_num * frame.laser_num * frame.packet_num * sizeof(PointDecodeData), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);
  cudaSafeCall(cudaMemcpy(sensor_timestamp_cu_, frame.sensor_timestamp,
                          frame.packet_num * sizeof(uint64_t), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);                                   
compute_xyzs_2_4_impl<<<frame.packet_num, frame.block_num * frame.laser_num>>>(this->frame_.gpu()->points, channel_azimuths_cu_, channel_elevations_cu_, 
  point_data_cu_, sensor_timestamp_cu_, frame.distance_unit, this->transform_, frame.block_num, frame.laser_num, frame.packet_num);
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  this->frame_.DeviceToHost(0, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  std::memcpy(frame.points, this->frame_.cpu()->points, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  return 0;
}
template<typename T_Point>
int Udp2_4ParserGpu<T_Point>::LoadCorrectionString(char *data) {
  if (LoadCorrectionDatData(data) == 0) {
    return 0;
  }
  return LoadCorrectionCsvData(data);
}


// csv ----> correction
template<typename T_Point>
int  Udp2_4ParserGpu<T_Point>::LoadCorrectionCsvData(char *correction_string)
{
  std::ifstream file(correction_string);
  if (!file.is_open()) {
      LogError("open the .csv faild");
      return 0;
  }
  std::vector<float> column2;
  std::vector<float> column3;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
      std::stringstream lineStream(line);
      std::string cell;
      std::vector<std::string> row;
      while (std::getline(lineStream, cell, ',')) {
          row.push_back(cell);
      }
      if (row.size() >= 3) {
          column2.push_back(std::stof(row[1]));
          column3.push_back(std::stof(row[2]));
      }
  }
  file.close();
  corrections_.delimiter[0] = 0xee;
  corrections_.delimiter[1] = 0xff;
  corrections_.major_version = 0x03;
  corrections_.min_version = 0x01;
  corrections_.channel_number = 0x40;
  corrections_.angle_division = 0x01;

  for (size_t i = 0; i < column3.size() && i < ET_MAX_CHANNEL_NUM_24; i++) {
    corrections_.elevations[i] = column2[i];
    corrections_.azimuths[i] = column3[i];
  }
  if(column3.size() > ET_MAX_CHANNEL_NUM_24) {
    LogError("correction.csv have invalid data, max line:%u", column3.size());
  }
  CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(corrections_.azimuths)));
  CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(corrections_.elevations)));
  CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections_.azimuths, sizeof(corrections_.azimuths), cudaMemcpyHostToDevice));
  CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections_.elevations, sizeof(corrections_.elevations), cudaMemcpyHostToDevice));
  corrections_loaded_ = true;
  return 0;
}


// buffer(.bin) ---> correction
template<typename T_Point>
int Udp2_4ParserGpu<T_Point>::LoadCorrectionDatData(char *data) {
  try {
    char *p = data;
    struct ETCorrections_v4_Header ETheader = *((struct ETCorrections_v4_Header* )p);
    if (0xee == ETheader.delimiter[0] && 0xff == ETheader.delimiter[1]) {
      switch (ETheader.min_version) {
        case 1: {
          memcpy((void *)&corrections_, p, sizeof(struct ETCorrections_v4_Header));
          p += sizeof(ETCorrections_v4_Header);
          auto channel_num = corrections_.channel_number;
          uint16_t division = corrections_.angle_division;
          if ((channel_num > ET_MAX_CHANNEL_NUM_24 - 3) || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.gamma)) / division;
          LogDebug("apha:%f, beta:%f, gamma:%f", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            LogDebug("%d %f %f",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          memcpy((void*)&corrections_.SHA_value, p, 32);
          // successed
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(corrections_.azimuths)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(corrections_.elevations)));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections_.azimuths, sizeof(corrections_.azimuths), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections_.elevations, sizeof(corrections_.elevations), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          return 0;
        } break;
        default:
          LogWarning("min_version is wrong!");
          break;
      }
    } else {
        return -1;
    }
  } catch (const std::exception &e) {
    LogFatal("load correction error: %s", e.what());
    return -1;
  }
  return -1;
}
template <typename T_Point>
int Udp2_4ParserGpu<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
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
    if (ret != 0) {
      LogError("Parse local Correction file Error");
    } else {
      LogInfo("Parse local Correction file Success!!!");
      return 0;
    }
  } else {
    LogError("Open correction file failed");
    return -1;
  }
  return -1;
}

