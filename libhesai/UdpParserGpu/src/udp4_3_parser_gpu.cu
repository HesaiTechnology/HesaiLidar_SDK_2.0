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
  cudaSafeMalloc(raw_azimuths_cu_, sizeof(PointCloudStruct<T_Point>::azimuths));
  cudaSafeMalloc(raw_distances_cu_, sizeof(PointCloudStruct<T_Point>::distances));
  cudaSafeMalloc(raw_reflectivities_cu_, sizeof(PointCloudStruct<T_Point>::reflectivities));
  cudaSafeMalloc(raw_sensor_timestamp_cu_, sizeof(PointCloudStruct<T_Point>::sensor_timestamp));
}
template <typename T_Point>
Udp4_3ParserGpu<T_Point>::~Udp4_3ParserGpu() {
  cudaSafeFree(raw_azimuths_cu_);
  cudaSafeFree(raw_distances_cu_);
  cudaSafeFree(raw_reflectivities_cu_);
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
    const float* raw_azimuths, const uint16_t *raw_distances, const uint8_t *raw_reflectivities, 
    const uint64_t *raw_sensor_timestamp, const double raw_distance_unit, Transform transform, const uint16_t blocknum, const uint16_t lasernum) {
  auto iscan = blockIdx.x;
  auto ichannel = threadIdx.x;
  if (ichannel >= blocknum * lasernum) return;
  float azimuth = raw_azimuths[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))] / kFineResolutionInt;
  int Azimuth = raw_azimuths[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))];
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

  auto rho = raw_distances[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))] * raw_distance_unit;
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
  gpu::setX(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], x_);
  gpu::setY(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))],  y_);
  gpu::setZ(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], z_);
  gpu::setIntensity(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], raw_reflectivities[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))]);
  gpu::setTimestamp(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], double(raw_sensor_timestamp[iscan]) / kMicrosecondToSecond);
}
template <typename T_Point>
int Udp4_3ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!corrections_loaded_) return int(ReturnCode::CorrectionsUnloaded);  
  cudaSafeCall(cudaMemcpy(raw_azimuths_cu_, frame.azimuth,
                          kMaxPacketNumPerFrame * kMaxPointsNumPerPacket * sizeof(float), cudaMemcpyHostToDevice),
               ReturnCode::CudaMemcpyHostToDeviceError);
  cudaSafeCall(cudaMemcpy(raw_distances_cu_, frame.distances,
                          kMaxPacketNumPerFrame * kMaxPointsNumPerPacket * sizeof(uint16_t),
                          cudaMemcpyHostToDevice),
               ReturnCode::CudaMemcpyHostToDeviceError); 
  cudaSafeCall(cudaMemcpy(raw_reflectivities_cu_, frame.reflectivities,
                          kMaxPacketNumPerFrame * kMaxPointsNumPerPacket * sizeof(uint8_t),
                          cudaMemcpyHostToDevice),
               ReturnCode::CudaMemcpyHostToDeviceError);  
  cudaSafeCall(cudaMemcpy(raw_sensor_timestamp_cu_, frame.sensor_timestamp,
                          kMaxPacketNumPerFrame * sizeof(uint64_t),
                          cudaMemcpyHostToDevice),
               ReturnCode::CudaMemcpyHostToDeviceError);                                       
compute_xyzs_v4_3_impl<<<kMaxPacketNumPerFrame, kMaxPointsNumPerPacket>>>(
    this->frame_.gpu()->points, (const int32_t*)channel_azimuths_cu_,
    (const int32_t*)channel_elevations_cu_, (const int8_t*)dazis_cu, deles_cu,
    mirror_azi_begins_cu, mirror_azi_ends_cu, corrections_header.resolution,
    raw_azimuths_cu_, raw_distances_cu_, raw_reflectivities_cu_, raw_sensor_timestamp_cu_, frame.distance_unit, 
    this->transform_, frame.block_num, frame.laser_num);
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  this->frame_.DeviceToHost();
  std::memcpy(frame.points, this->frame_.cpu()->points, sizeof(T_Point) * kMaxPacketNumPerFrame * kMaxPointsNumPerPacket);
  return 0;
}
template <typename T_Point>
int Udp4_3ParserGpu<T_Point>::LoadCorrectionString(char *p) {
  corrections_header = *(Corrections::Header *)p;
  if ( 0xee != corrections_header.delimiter[0] || 0xff != corrections_header.delimiter[1]) {
    std::cerr << "Correction Header delimiter not right" << std::endl;
    return -1;
  }
  // printf("mirror_num: %d,\t channel_num=%d,\tversion=%d\n",
  //        corrections_header.nframes, corrections_header.max_channel_num,
  //        corrections_header.version[1]);
  // printf("resolution: %d\n",
  //        corrections_header.resolution);
  if (corrections_loaded_) {
    return 0;
    if (deles_cu) cudaFree(deles_cu);
    if (dazis_cu) cudaFree(dazis_cu);
    if (channel_elevations_cu_) cudaFree(channel_elevations_cu_);
    if (channel_azimuths_cu_) cudaFree(channel_azimuths_cu_);
    if (mirror_azi_begins_cu) cudaFree(mirror_azi_begins_cu);
    if (mirror_azi_ends_cu) cudaFree(mirror_azi_ends_cu);
    corrections_loaded_ = false;
  }
  // float channel_azimuths[kMaxPointsNumPerPacket];
  switch (corrections_header.version[1]) {
    case 3: {
      // HCHECK_GE(size, sizeof(CorrectionsV1_3));
      auto& corrections = *(CorrectionsV1_3*)p;
      mirror_azi_begins[0] = corrections.mirror_azi_begins[0];
      mirror_azi_begins[1] = corrections.mirror_azi_begins[1];
      mirror_azi_begins[2] = corrections.mirror_azi_begins[2];
      mirror_azi_ends[0] = corrections.mirror_azi_ends[0];
      mirror_azi_ends[1] = corrections.mirror_azi_ends[1];
      mirror_azi_ends[2] = corrections.mirror_azi_ends[2];
      CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(CorrectionsV1_3::channel_azimuths)));
      CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(CorrectionsV1_3::channel_elevations)));
      CUDACheck(cudaMalloc(&dazis_cu, sizeof(CorrectionsV1_3::dazis)));
      CUDACheck(cudaMalloc(&deles_cu, sizeof(CorrectionsV1_3::deles)));
     
      CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections.channel_azimuths, sizeof(corrections.channel_azimuths), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections.channel_elevations, sizeof(corrections.channel_elevations), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(dazis_cu, corrections.dazis, sizeof(corrections.dazis), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(deles_cu, corrections.deles, sizeof(corrections.deles), cudaMemcpyHostToDevice));
      // for (auto i = 0; i < kMaxPointsNumPerPacket; ++i) {
      //     channel_azimuths[i] = corrections.channel_azimuths[i];
      // }
      break;
    }
    case 5: {
      // HCHECK_GE(size, sizeof(CorrectionsV1_5));
      auto& corrections = *(CorrectionsV1_5*)(p);
      mirror_azi_begins[0] = corrections.mirror_azi_begins[0] / kFineResolutionFloat * corrections.header.resolution;
      mirror_azi_begins[1] = corrections.mirror_azi_begins[1] / kFineResolutionFloat * corrections.header.resolution;
      mirror_azi_begins[2] = corrections.mirror_azi_begins[2] / kFineResolutionFloat * corrections.header.resolution;
      mirror_azi_ends[0] = corrections.mirror_azi_ends[0] / kFineResolutionFloat * corrections.header.resolution;
      mirror_azi_ends[1] = corrections.mirror_azi_ends[1] / kFineResolutionFloat * corrections.header.resolution;
      mirror_azi_ends[2] = corrections.mirror_azi_ends[2] / kFineResolutionFloat * corrections.header.resolution;
      for (int i = 0; i < corrections_header.nframes; ++i) {
            corrections.mirror_azi_begins[i] =
                corrections.mirror_azi_begins[i] *
                corrections.header.resolution;
            corrections.mirror_azi_ends[i] =
                corrections.mirror_azi_ends[i] *
                corrections.header.resolution;
            // printf("%lf,   %lf\n",
            //        corrections.mirror_azi_begins[i] / (kFineResolutionFloat * kResolutionFloat),
            //        corrections.mirror_azi_ends[i] / (kFineResolutionFloat * kResolutionFloat));
          }
      CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(CorrectionsV1_5::channel_azimuths)));
      CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(CorrectionsV1_5::channel_elevations)));
      CUDACheck(cudaMalloc(&dazis_cu, sizeof(CorrectionsV1_5::dazis)));
      CUDACheck(cudaMalloc(&deles_cu, sizeof(CorrectionsV1_5::deles)));
      CUDACheck(cudaMalloc(&mirror_azi_begins_cu, sizeof(CorrectionsV1_5::mirror_azi_begins)));
      CUDACheck(cudaMalloc(&mirror_azi_ends_cu, sizeof(CorrectionsV1_5::mirror_azi_ends)));
      CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections.channel_azimuths, sizeof(corrections.channel_azimuths), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections.channel_elevations, sizeof(corrections.channel_elevations), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(dazis_cu, corrections.dazis, sizeof(corrections.dazis), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(deles_cu, corrections.deles, sizeof(corrections.deles), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(mirror_azi_begins_cu, corrections.mirror_azi_begins, sizeof(corrections.mirror_azi_begins), cudaMemcpyHostToDevice));
      CUDACheck(cudaMemcpy(mirror_azi_ends_cu, corrections.mirror_azi_ends, sizeof(corrections.mirror_azi_ends), cudaMemcpyHostToDevice));
      // for (auto i = 0; i < 128; ++i) {
      //     channel_azimuths[i] = corrections.channel_azimuths[i] * corrections.header.resolution / kFineResolutionFloat;
      //     // std::cout << corrections.channel_azimuths[i] * corrections.header.resolution / kFineResolutionFloat << std::endl;
      // }
      break;
    }
    default:
      std::cout << "Unknown Corrections Version !!!" << std::endl;
      return -1;
  }
  corrections_loaded_ = true;
  return 0;
}
template <typename T_Point>
int Udp4_3ParserGpu<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
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
      return 0;
    }
  } else {
    printf("Open correction file failed\n");
    return -1;
  }
  return -1;
}
