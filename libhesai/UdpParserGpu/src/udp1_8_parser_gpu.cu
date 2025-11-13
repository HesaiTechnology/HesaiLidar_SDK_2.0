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

#include "udp1_8_parser_gpu.h"
namespace hesai
{
namespace lidar
{

__global__ void compute_xyzs_1_8_impl(uint8_t* point_cloud_cu_, uint32_t point_cloud_size, const float* correction_azi, const float* correction_ele, 
    bool distance_correction_flag, TransformParam transform, const LidarOpticalCenter optical_center, CudaPointXYZAER* points_cu_) {
  auto packet_index = blockIdx.x;
  auto channel_index = threadIdx.x;
  auto block_index = threadIdx.y;
  auto channel_num = blockDim.x;
  auto block_num = blockDim.y;

  extern __shared__ uint8_t shared_data[];
  int tid = block_index * channel_num + channel_index;
  int thread_count = block_num * channel_num;
  const uint8_t* input_data = point_cloud_cu_ + packet_index * point_cloud_size;
  if (input_data[0] != 0xEE || input_data[1] != 0xFF) return;
  for (int i = tid; i < point_cloud_size; i += thread_count) {
    shared_data[i] = input_data[i];
  }
  __syncthreads();
  int point_index = packet_index * block_num * channel_num + block_index * channel_num + channel_index;
  int azimuth_offset = 16;
  float azimuth = (shared_data[azimuth_offset + 0] + shared_data[azimuth_offset + 1] * 0x100) / 100.0;
  int uint_offset = azimuth_offset + 2 + 3 * channel_index;
  float distance = (shared_data[uint_offset + 0] + shared_data[uint_offset + 1] * 0x100) * 0.004f;
  uint8_t intensity = shared_data[uint_offset + 2];

  int tail_offset = azimuth_offset + 2 + (channel_num * 3);
  uint32_t dirty_degree_all = shared_data[tail_offset] + shared_data[tail_offset + 1] * 0x100 + shared_data[tail_offset + 2] * 0x10000 + shared_data[tail_offset + 3] * 0x1000000;
  uint8_t dirty_degree= (dirty_degree_all >> (channel_index * 2)) & 0x03;

  float azimuthCalib = azimuth;
  azimuthCalib = azimuthCalib / HALF_CIRCLE * M_PI;
  float theta = correction_azi[channel_index] / HALF_CIRCLE * M_PI;
  float phi = correction_ele[channel_index] / HALF_CIRCLE * M_PI;
  if(distance > 0.09 && distance_correction_flag) {
    theta -= 190.0 / HALF_CIRCLE * M_PI;
    float tx = cos(phi) * sin(theta);
    float ty = cos(phi) * cos(theta);
    float tz = sin(phi);
    float d = distance;
    float x = d * tx + optical_center.x;
    float y = d * ty + optical_center.y;
    float z = d * tz + optical_center.z;
    float d_geometric_center = sqrt(x * x + y * y + z * z);
    theta = azimuthCalib + atan2(x, y);
    phi = asin(z / d_geometric_center);
    distance = d_geometric_center;
    theta += 190.0 / HALF_CIRCLE * M_PI;
  } else {
    theta += azimuthCalib;
  }
  
  float z = distance * sin(phi);
  auto r = distance * cosf(phi);
  float x = r * sin(theta);
  float y = r * cos(theta);

  if (transform.use_flag) {
    float cosa = cos(transform.roll);
    float sina = sin(transform.roll);
    float cosb = cos(transform.pitch);
    float sinb = sin(transform.pitch);
    float cosc = cos(transform.yaw);
    float sinc = sin(transform.yaw);

    float x_ = cosb * cosc * x + (sina * sinb * cosc - cosa * sinc) * y +
                (sina * sinc + cosa * sinb * cosc) * z + transform.x;
    float y_ = cosb * sinc * x + (cosa * cosc + sina * sinb * sinc) * y +
                (cosa * sinb * sinc - sina * cosc) * z + transform.y;
    float z_ = -sinb * x + sina * cosb * y + cosa * cosb * z + transform.z;

    x = x_;
    y = y_;
    z = z_;
  }

  points_cu_[point_index].x = x;
  points_cu_[point_index].y = y;
  points_cu_[point_index].z = z;
  points_cu_[point_index].azimuthCalib = theta * HALF_CIRCLE / M_PI;
  while (points_cu_[point_index].azimuthCalib < 0) points_cu_[point_index].azimuthCalib += 360.0f;
  while (points_cu_[point_index].azimuthCalib >= 360.0f) points_cu_[point_index].azimuthCalib -= 360.0f;
  points_cu_[point_index].elevationCalib = phi * HALF_CIRCLE / M_PI;
  while (points_cu_[point_index].elevationCalib < -180.0f) points_cu_[point_index].elevationCalib += 360.0f;
  while (points_cu_[point_index].elevationCalib >= 180.0f) points_cu_[point_index].elevationCalib -= 360.0f;
  points_cu_[point_index].reserved[0] = intensity;
  points_cu_[point_index].reserved[1] = dirty_degree;
}

int compute_1_8_cuda(uint8_t* point_cloud_cu_, CudaPointXYZAER* points_cu_, uint32_t point_cloud_size, const float* correction_azi, 
    const float* correction_ele, const FrameDecodeParam* fParam, LidarOpticalCenter optical_center, 
    uint32_t packet_num, uint16_t block_num, uint16_t channel_num) {
  dim3 grid(packet_num);
  dim3 block(channel_num, block_num);
  compute_xyzs_1_8_impl<<<grid, block, point_cloud_size * sizeof(uint8_t)>>>(point_cloud_cu_, point_cloud_size, correction_azi, correction_ele, 
    fParam->distance_correction_flag, fParam->transform, optical_center, points_cu_);
  cudaDeviceSynchronize();
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  return 0;
}

}
}

