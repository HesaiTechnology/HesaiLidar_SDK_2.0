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

#include "udp4_7_parser_gpu.h"
#include "safe_call.cuh"
#include "return_code.h"

using namespace hesai::lidar;
template <typename T_Point>
Udp4_7ParserGpu<T_Point>::Udp4_7ParserGpu() {
  corrections_loaded_ = false;
  cudaSafeMalloc(point_data_cu_, POINT_DATA_LEN);
  cudaSafeMalloc(sensor_timestamp_cu_, SENSOR_TIMESTAMP_LEN);
}
template <typename T_Point>
Udp4_7ParserGpu<T_Point>::~Udp4_7ParserGpu() {
  cudaSafeFree(point_data_cu_);
  cudaSafeFree(sensor_timestamp_cu_);
  if (corrections_loaded_) {
    cudaSafeFree(deles_cu);
    cudaSafeFree(channel_elevations_cu_);
    corrections_loaded_ = false;
  }
}
template <typename T_Point>
__global__ void compute_xyzs_v4_7_impl(
    T_Point *xyzs, const float* channel_elevations, const float* elevation_adjust,
    const PointDecodeData* point_data, const uint64_t* sensor_timestamp, const double raw_distance_unit, Transform transform, const uint16_t blocknum, const uint16_t lasernum, 
    const uint8_t version, const uint16_t packet_index) {
  auto iscan = blockIdx.x;
  auto ichannel = threadIdx.x;
  if (iscan >= packet_index || ichannel >= blocknum * lasernum) return;
  int point_index = iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum));
  float azimuth = point_data[point_index].azimuth / kAllFineResolutionFloat;
  float elevation = channel_elevations[ichannel % lasernum];
  
  if (version == 3) {
    const float BegElevationAdjust = 20.0;
    const float StepElevationAdjust = 2.0;
    const float EndElevationAdjust = 158.0;  // 20 + 2 * (70 - 1)
    if (azimuth >= StepElevationAdjust && azimuth <= EndElevationAdjust) {
      int index = (azimuth - BegElevationAdjust) / StepElevationAdjust;
      float left_percent = (azimuth - BegElevationAdjust - index * StepElevationAdjust) / StepElevationAdjust;
      elevation += elevation_adjust[index] * (1 - left_percent) + elevation_adjust[index + 1] * left_percent;
    }
  }

  auto phi = elevation * kResolutionFloat / kHalfCircleFloat * M_PI;
  auto theta = azimuth * kResolutionFloat / kHalfCircleFloat * M_PI;

  auto rho = point_data[point_index].distances * raw_distance_unit;
  float z = rho * sin(phi);
  auto r = rho * cosf(phi);
  float x = r * sin(theta);
  float y = r * cos(theta);

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
int Udp4_7ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!corrections_loaded_) return int(ReturnCode::CorrectionsUnloaded);  
  cudaSafeCall(cudaMemcpy(point_data_cu_, frame.pointData,
                          frame.block_num * frame.laser_num * frame.packet_num * sizeof(PointDecodeData), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);
  cudaSafeCall(cudaMemcpy(sensor_timestamp_cu_, frame.sensor_timestamp,
                          frame.packet_num * sizeof(uint64_t), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);                                          
compute_xyzs_v4_7_impl<<<frame.packet_num, frame.block_num * frame.laser_num>>>(
    this->frame_.gpu()->points, channel_elevations_cu_, deles_cu, point_data_cu_, sensor_timestamp_cu_,
    frame.distance_unit, this->transform_, frame.block_num, frame.laser_num, m_ATX_corrections.header.version[1], frame.packet_num);
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  this->frame_.DeviceToHost(0, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  std::memcpy(frame.points, this->frame_.cpu()->points, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  return 0;
}
template <typename T_Point>
int Udp4_7ParserGpu<T_Point>::LoadCorrectionString(char *data) {
  try {
    char *p = data;
    ATXCorrectionsHeader header = *(ATXCorrectionsHeader *)p;
    if (0xee == header.delimiter[0] && 0xff == header.delimiter[1]) {
      switch (header.version[1]) {
        case 1: {
          m_ATX_corrections.header = header;
          auto channel_num = m_ATX_corrections.header.channel_number;
          uint16_t division = m_ATX_corrections.header.angle_division;
          p += sizeof(ATXCorrectionsHeader);
          if (channel_num > ATX_LASER_NUM || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&m_ATX_corrections.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;

          for (int i = 0; i < channel_num; i++) {
            m_ATX_corrections.azimuth[i] = ((float)(m_ATX_corrections.raw_azimuths[i])) / (float)division;
            m_ATX_corrections.elevation[i] = ((float)(m_ATX_corrections.raw_elevations[i])) / (float)division;
            // printf("%d %f %f %d\n", i, m_ATX_corrections.azimuth[i], m_ATX_corrections.elevation[i], division);
          } 
          memcpy((void*)&m_ATX_corrections.SHA_value, p, 32);
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(m_ATX_corrections.elevation)));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, m_ATX_corrections.elevation, sizeof(m_ATX_corrections.elevation), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          return 0;
        } break;
        case 2: {
          m_ATX_corrections.header = header;
          auto channel_num = m_ATX_corrections.header.channel_number;
          uint16_t division = m_ATX_corrections.header.angle_division;
          p += sizeof(ATXCorrectionsHeader);
          if (channel_num > ATX_LASER_NUM || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&m_ATX_corrections.raw_azimuths_even, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;       
          memcpy((void *)&m_ATX_corrections.raw_azimuths_odd, p,
                 sizeof(int16_t) * channel_num);       
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;

          for (int i = 0; i < channel_num; i++) {
            m_ATX_corrections.azimuth_even[i] = ((float)(m_ATX_corrections.raw_azimuths_even[i])) / (float)division;
            m_ATX_corrections.azimuth_odd[i] = ((float)(m_ATX_corrections.raw_azimuths_odd[i])) / (float)division;
            m_ATX_corrections.elevation[i] = ((float)(m_ATX_corrections.raw_elevations[i])) / (float)division;
            // printf("%d %f %f %f %d\n", i, m_ATX_corrections.azimuth_even[i], m_ATX_corrections.azimuth_odd[i],  m_ATX_corrections.elevation[i], division);
          } 
          memcpy((void*)&m_ATX_corrections.SHA_value, p, 32);
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(m_ATX_corrections.elevation)));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, m_ATX_corrections.elevation, sizeof(m_ATX_corrections.elevation), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          return 0;
        } break;
        case 3: {
          m_ATX_corrections.header = header;
          auto channel_num = m_ATX_corrections.header.channel_number;
          uint16_t division = m_ATX_corrections.header.angle_division;
          p += sizeof(ATXCorrectionsHeader);
          if (channel_num > ATX_LASER_NUM || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&m_ATX_corrections.raw_azimuths_even, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;       
          memcpy((void *)&m_ATX_corrections.raw_azimuths_odd, p,
                 sizeof(int16_t) * channel_num);       
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations_adjust, p,
                 sizeof(int16_t) * m_ATX_corrections.kLenElevationAdjust);
          p += sizeof(int16_t) * m_ATX_corrections.kLenElevationAdjust;

          for (int i = 0; i < channel_num; i++) {
            m_ATX_corrections.azimuth_even[i] = ((float)(m_ATX_corrections.raw_azimuths_even[i])) / (float)division;
            m_ATX_corrections.azimuth_odd[i] = ((float)(m_ATX_corrections.raw_azimuths_odd[i])) / (float)division;
            m_ATX_corrections.elevation[i] = ((float)(m_ATX_corrections.raw_elevations[i])) / (float)division;
            // printf("%d %f %f %f %d\n", i, m_ATX_corrections.azimuth_even[i], m_ATX_corrections.azimuth_odd[i],  m_ATX_corrections.elevation[i], division);
          } 
          for (uint32_t i = 0; i < m_ATX_corrections.kLenElevationAdjust; i++) {
            m_ATX_corrections.elevation_adjust[i] = ((float)(m_ATX_corrections.raw_elevations_adjust[i])) / (float)division;
          }
          memcpy((void*)&m_ATX_corrections.SHA_value, p, 32);
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(m_ATX_corrections.elevation)));
          CUDACheck(cudaMalloc(&deles_cu, sizeof(m_ATX_corrections.elevation_adjust)));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, m_ATX_corrections.elevation, sizeof(m_ATX_corrections.elevation), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(deles_cu, m_ATX_corrections.elevation_adjust, sizeof(m_ATX_corrections.elevation_adjust), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
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
template <typename T_Point>
int Udp4_7ParserGpu<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
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
      return 0;
    }
  } else {
    LogError("Open correction file failed");
    return -1;
  }
  return -1;
}
