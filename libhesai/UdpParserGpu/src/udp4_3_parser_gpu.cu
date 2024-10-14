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

#include "udp4_3_parser_gpu.h"
#include "safe_call.cuh"
#include "return_code.h"

using namespace hesai::lidar;
template <typename T_Point>
Udp4_3ParserGpu<T_Point>::Udp4_3ParserGpu() {
  corrections_loaded_ = false;
  cudaSafeMalloc(point_data_cu_, POINT_DATA_LEN);
  cudaSafeMalloc(sensor_timestamp_cu_, SENSOR_TIMESTAMP_LEN);
}
template <typename T_Point>
Udp4_3ParserGpu<T_Point>::~Udp4_3ParserGpu() {
  cudaSafeFree(point_data_cu_);
  cudaSafeFree(sensor_timestamp_cu_);
  if (corrections_loaded_) {
    cudaSafeFree(deles_cu);
    cudaSafeFree(dazis_cu);
    cudaSafeFree(channel_elevations_cu_);
    cudaSafeFree(channel_azimuths_cu_);
    cudaSafeFree(mirror_azi_begins_cu);
    cudaSafeFree(mirror_azi_ends_cu);
    corrections_loaded_ = false;
  }
}
template <typename T_Point>
__global__ void compute_xyzs_v4_3_impl(
    T_Point *xyzs, const int32_t* channel_azimuths,
    const int32_t* channel_elevations, const int8_t* dazis, const int8_t* deles,
    const uint32_t* raw_azimuth_begin, const uint32_t* raw_azimuth_end, const uint8_t raw_correction_resolution,
    const PointDecodeData* point_data, const uint64_t* sensor_timestamp, const double raw_distance_unit, Transform transform, const uint16_t blocknum, const uint16_t lasernum, uint16_t packet_index) {
  auto iscan = blockIdx.x;
  auto ichannel = threadIdx.x;
  if (iscan >= packet_index || ichannel >= blocknum * lasernum) return;
  int point_index = iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum));
  float azimuth = point_data[point_index].azimuth / kFineResolutionInt;
  int Azimuth = point_data[point_index].azimuth;
  int count = 0, field = 0;
  while (count < 3 &&
          (((Azimuth + (kFineResolutionInt * kCircle) - raw_azimuth_begin[field]) % (kFineResolutionInt * kCircle) +
          (raw_azimuth_end[field] + kFineResolutionInt * kCircle - Azimuth) % (kFineResolutionInt * kCircle)) !=
          (raw_azimuth_end[field] + kFineResolutionInt * kCircle -
          raw_azimuth_begin[field]) % (kFineResolutionInt * kCircle))) {
    field = (field + 1) % 3;
    count++;
  }
  if (count >= 3) return;
  float m = azimuth / 200.f;
  int i = m;
  int j = i + 1;
  float alpha = m - i;    // k
  float beta = 1 - alpha; // 1-k
  // getAziAdjustV3
  auto dazi =
      beta * dazis[(ichannel % lasernum) * (kHalfCircleInt / kResolutionInt) + i] + alpha * dazis[(ichannel % lasernum) * (kHalfCircleInt / kResolutionInt) + j];
  auto theta = ((azimuth + kCircle - raw_azimuth_begin[field] / kFineResolutionFloat) * 2 -
                channel_azimuths[(ichannel % lasernum)] * raw_correction_resolution / kFineResolutionFloat + dazi * raw_correction_resolution) /
               kHalfCircleFloat * M_PI;
   // getEleAdjustV3
  auto dele =
      beta * deles[(ichannel % lasernum) * (kHalfCircleInt / kResolutionInt) + i] + alpha * deles[(ichannel % lasernum) * (kHalfCircleInt / kResolutionInt) + j];
  auto phi = (channel_elevations[(ichannel % lasernum)] / kFineResolutionFloat + dele) * raw_correction_resolution / kHalfCircleFloat * M_PI;

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
int Udp4_3ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!corrections_loaded_) return int(ReturnCode::CorrectionsUnloaded);  
  cudaSafeCall(cudaMemcpy(point_data_cu_, frame.pointData,
                          frame.block_num * frame.laser_num * frame.packet_num * sizeof(PointDecodeData), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);
  cudaSafeCall(cudaMemcpy(sensor_timestamp_cu_, frame.sensor_timestamp,
                          frame.packet_num * sizeof(uint64_t), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);                          
compute_xyzs_v4_3_impl<<<frame.packet_num, frame.block_num * frame.laser_num>>>(
    this->frame_.gpu()->points, (const int32_t*)channel_azimuths_cu_,
    (const int32_t*)channel_elevations_cu_, (const int8_t*)dazis_cu, deles_cu,
    mirror_azi_begins_cu, mirror_azi_ends_cu, m_PandarAT_corrections.header.resolution,
    point_data_cu_, sensor_timestamp_cu_, frame.distance_unit, this->transform_, frame.block_num, frame.laser_num, frame.packet_num);
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  this->frame_.DeviceToHost(0, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  std::memcpy(frame.points, this->frame_.cpu()->points, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  return 0;
}
template <typename T_Point>
int Udp4_3ParserGpu<T_Point>::LoadCorrectionString(char *data) {
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
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(m_PandarAT_corrections.l.azimuth)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(m_PandarAT_corrections.l.elevation)));
          CUDACheck(cudaMalloc(&dazis_cu, sizeof(m_PandarAT_corrections.azimuth_offset)));
          CUDACheck(cudaMalloc(&deles_cu, sizeof(m_PandarAT_corrections.elevation_offset)));
          CUDACheck(cudaMalloc(&mirror_azi_begins_cu, sizeof(m_PandarAT_corrections.l.start_frame)));
          CUDACheck(cudaMalloc(&mirror_azi_ends_cu, sizeof(m_PandarAT_corrections.l.end_frame)));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, m_PandarAT_corrections.l.azimuth, sizeof(m_PandarAT_corrections.l.azimuth), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, m_PandarAT_corrections.l.elevation, sizeof(m_PandarAT_corrections.l.elevation), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(dazis_cu, m_PandarAT_corrections.azimuth_offset, sizeof(m_PandarAT_corrections.azimuth_offset), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(deles_cu, m_PandarAT_corrections.elevation_offset, sizeof(m_PandarAT_corrections.elevation_offset), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(mirror_azi_begins_cu, m_PandarAT_corrections.l.start_frame, sizeof(m_PandarAT_corrections.l.start_frame), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(mirror_azi_ends_cu, m_PandarAT_corrections.l.end_frame, sizeof(m_PandarAT_corrections.l.end_frame), cudaMemcpyHostToDevice));
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
int Udp4_3ParserGpu<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
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
