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

#include "udp2_5_parser_gpu.h"
#include "safe_call.cuh"
#include "return_code.h"

using namespace hesai::lidar;
template <typename T_Point>
Udp2_5ParserGpu<T_Point>::Udp2_5ParserGpu() {
  corrections_loaded_ = false;
  cudaSafeMalloc(point_data_cu_, POINT_DATA_LEN);
  cudaSafeMalloc(sensor_timestamp_cu_, SENSOR_TIMESTAMP_LEN);
}
template <typename T_Point>
Udp2_5ParserGpu<T_Point>::~Udp2_5ParserGpu() {
  cudaSafeFree(point_data_cu_);
  cudaSafeFree(sensor_timestamp_cu_);
  if (corrections_loaded_) {
    cudaSafeFree(channel_elevations_cu_);
    cudaSafeFree(channel_azimuths_cu_);
    if (corrections_.min_version >= 2) {
      cudaSafeFree(channel_azimuths_adjust_cu_);
      cudaSafeFree(channel_elevations_adjust_cu_);
    }
    if (corrections_.min_version >= 4) {
      cudaSafeFree(gamma_cu);
      cudaSafeFree(elevation_offset_delta_cu);
      cudaSafeFree(azimuth_offset_delta_cu);
    }
    corrections_loaded_ = false;
  }
}
template <typename T_Point>
__global__ void compute_xyzs_2_5_impl_v1(T_Point *xyzs, const float* channel_azimuths, const float* channel_elevations, const PointDecodeData* point_data, const uint64_t* sensor_timestamp, 
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
}
template <typename T_Point>
__global__ void compute_xyzs_2_5_impl_v2_v3(T_Point *xyzs, const float* channel_azimuths, const float* channel_elevations,
    const float* azimuth_adjust, const float* elevation_adjust, const uint8_t azimuth_adjust_interval, const uint8_t elevation_adjust_interval,
    const PointDecodeData* point_data, const uint64_t* sensor_timestamp, const double raw_distance_unit, Transform transform, const uint16_t blocknum, const uint16_t lasernum, const uint16_t packet_index) {
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

  float azi_a = azi_h - 90;
  float azimuth_fov = 120.0f;
  float elevation_fov = 25.0f;
  float adjust_interval_resolution = 0.5f;
  int azimuth_offset_num = int(azimuth_fov / (azimuth_adjust_interval * adjust_interval_resolution) + 1);
  int elevation_offset_num = int(elevation_fov / (elevation_adjust_interval * adjust_interval_resolution) + 1);
  int offset_index1 = int((azi_a + azimuth_fov / 2) /  (azimuth_adjust_interval * adjust_interval_resolution));      //azi dimension
  int offset_index2 = int((elv_h + elevation_fov / 2) /  (elevation_adjust_interval * adjust_interval_resolution));      //ele dimension
  if (offset_index1 >= 0 && offset_index1 < (azimuth_offset_num - 1) && offset_index2 >= 0 && offset_index2 < (elevation_offset_num - 1)) {
    float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi_a - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
    float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - elv_h - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
    float offset1 = coefficient1 * azimuth_adjust[offset_index1  + offset_index2 * azimuth_offset_num] + (1 - coefficient1) * azimuth_adjust[offset_index1 + 1 + offset_index2 * azimuth_offset_num];
    float offset2 = coefficient1 * azimuth_adjust[offset_index1 + (offset_index2 + 1) * azimuth_offset_num] + (1 - coefficient1) * azimuth_adjust[offset_index1 + 1 + (offset_index2 + 1) * azimuth_offset_num];
    azi_h += (coefficient2 * offset1 + (1 - coefficient2) * offset2);
  }
  azi_a = azi_h - 90;
  offset_index1 = int((azi_a + azimuth_fov / 2) /  (azimuth_adjust_interval * adjust_interval_resolution));
  if (offset_index1 >= 0 && offset_index1 < (azimuth_offset_num - 1) && offset_index2 >= 0 && offset_index2 < (elevation_offset_num - 1)) {
    float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi_a - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
    float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - elv_h - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
    float offset1 = coefficient1 * elevation_adjust[offset_index1  + offset_index2 * azimuth_offset_num] + (1 - coefficient1) * elevation_adjust[offset_index1 + 1 + offset_index2 * azimuth_offset_num];
    float offset2 = coefficient1 * elevation_adjust[offset_index1 + (offset_index2 + 1) * azimuth_offset_num] + (1 - coefficient1) * elevation_adjust[offset_index1 + 1 + (offset_index2 + 1) * azimuth_offset_num];
    elv_h += (coefficient2 * offset1 + (1 - coefficient2) * offset2);
  }

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
}
template <typename T_Point>
__global__ void compute_xyzs_2_5_impl_v4(T_Point *xyzs, const float* channel_azimuths, const float* channel_elevations,
    const float* azimuth_adjust, const float* elevation_adjust, const uint8_t azimuth_adjust_interval, const uint8_t elevation_adjust_interval,
    const float* gamma_vec, const float* azimuth_offset_delta, const float* elevation_offset_delta,
    const PointDecodeData* point_data, const uint64_t* sensor_timestamp, const double raw_distance_unit, Transform transform, const uint16_t blocknum, const uint16_t lasernum, const uint16_t packet_index) {
  auto iscan = blockIdx.x;
  auto ichannel = threadIdx.x;
  if (iscan >= packet_index || ichannel >= blocknum * lasernum) return;
  int point_index = iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum));
  float apha =  channel_elevations[0];
  float beta =  channel_elevations[1];
  float gamma =  channel_elevations[2];
  int mirror_i = point_data[point_index].mirror_index / 0x10;
  int turn_index = point_data[point_index].mirror_index & 0x0F;
  gamma = gamma_vec[mirror_i];
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


  float azi_a = azi_h - 90;
  const float azimuth_fov = 120.0f;
  const float elevation_fov = 3.2f;
  const float adjust_interval_resolution = 0.1f;
  // T * (120 / H / 0.1 + 1) * (3.2 / V / 0.1 + 1)
  int azimuth_offset_num = azimuth_fov / (azimuth_adjust_interval * adjust_interval_resolution) + 1;
  int elevation_offset_num = elevation_fov / (elevation_adjust_interval * adjust_interval_resolution) + 1;
  int offset_index1 = (azi_a + azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);      //azi dimension
  int offset_index2 = (elv_h + elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);      //ele dimension
  if (offset_index1 < (azimuth_offset_num - 1) && offset_index2 < (elevation_offset_num - 1) && offset_index1 >= 0 && offset_index2 >= 0) {
    float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi_a - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
    float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - elv_h - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
    // az_offset * ele_offset * m_id
    auto mr_offset = azimuth_offset_num * elevation_offset_num * turn_index;
    auto az_index1 = offset_index1 + offset_index2 * azimuth_offset_num + mr_offset;
    auto az_index2 = offset_index1 + (offset_index2 + 1) * azimuth_offset_num + mr_offset;
    // azimuth_adjust
    float offset1 = coefficient1 * azimuth_adjust[az_index1] + (1 - coefficient1) * azimuth_adjust[az_index1 + 1];
    float offset2 = coefficient1 * azimuth_adjust[az_index2] + (1 - coefficient1) * azimuth_adjust[az_index2 + 1];
    azi_h += (coefficient2 * offset1 + (1 - coefficient2) * offset2);
  }
  azi_a = azi_h - 90;
  offset_index1 = (azi_a + azimuth_fov / 2) /  (azimuth_adjust_interval * adjust_interval_resolution);
  if (offset_index1 < (azimuth_offset_num - 1) && offset_index2 < (elevation_offset_num - 1) && offset_index1 >= 0 && offset_index2 >= 0) {
    float coefficient1 = ((offset_index1 + 1) * (azimuth_adjust_interval * adjust_interval_resolution)  - azi_a - azimuth_fov / 2) / (azimuth_adjust_interval * adjust_interval_resolution);
    float coefficient2 = ((offset_index2 + 1) * (elevation_adjust_interval * adjust_interval_resolution)  - elv_h - elevation_fov / 2) / (elevation_adjust_interval * adjust_interval_resolution);
    // az_offset * ele_offset * m_id
    auto mr_offset = azimuth_offset_num * elevation_offset_num * turn_index;
    auto az_index1 = offset_index1 + offset_index2 * azimuth_offset_num + mr_offset;
    auto az_index2 = offset_index1 + (offset_index2 + 1) * azimuth_offset_num + mr_offset;
    // azimuth_adjust
    float offset1 = coefficient1 * elevation_adjust[az_index1] + (1 - coefficient1) * elevation_adjust[az_index1 + 1];
    float offset2 = coefficient1 * elevation_adjust[az_index2] + (1 - coefficient1) * elevation_adjust[az_index2 + 1];
    elv_h += (coefficient2 * offset1 + (1 - coefficient2) * offset2);
  }
  azi_h += azimuth_offset_delta[mirror_i];
  elv_h += elevation_offset_delta[mirror_i];
  

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
}
template <typename T_Point>
int Udp2_5ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!corrections_loaded_) return int(ReturnCode::CorrectionsUnloaded);          
  cudaSafeCall(cudaMemcpy(point_data_cu_, frame.pointData,
                          frame.block_num * frame.laser_num * frame.packet_num * sizeof(PointDecodeData), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);
  cudaSafeCall(cudaMemcpy(sensor_timestamp_cu_, frame.sensor_timestamp,
                          frame.packet_num * sizeof(uint64_t), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);     
  if (corrections_.min_version == 1) {
    compute_xyzs_2_5_impl_v1<<<frame.packet_num, frame.block_num * frame.laser_num>>>(this->frame_.gpu()->points, channel_azimuths_cu_, channel_elevations_cu_, 
        point_data_cu_, sensor_timestamp_cu_, frame.distance_unit, this->transform_, frame.block_num, frame.laser_num, frame.packet_num);
  } else if (corrections_.min_version == 2 || corrections_.min_version == 3) {
    compute_xyzs_2_5_impl_v2_v3<<<frame.packet_num, frame.block_num * frame.laser_num>>>(this->frame_.gpu()->points, channel_azimuths_cu_, channel_elevations_cu_, 
        channel_azimuths_adjust_cu_, channel_elevations_adjust_cu_, corrections_.azimuth_adjust_interval, corrections_.elevation_adjust_interval,
        point_data_cu_, sensor_timestamp_cu_, frame.distance_unit, this->transform_, frame.block_num, frame.laser_num, frame.packet_num);
  } else if (corrections_.min_version == 4) {
    compute_xyzs_2_5_impl_v4<<<frame.packet_num, frame.block_num * frame.laser_num>>>(this->frame_.gpu()->points, channel_azimuths_cu_, channel_elevations_cu_, 
        channel_azimuths_adjust_cu_, channel_elevations_adjust_cu_, corrections_.azimuth_adjust_interval, corrections_.elevation_adjust_interval,
        gamma_cu, azimuth_offset_delta_cu, elevation_offset_delta_cu, point_data_cu_, sensor_timestamp_cu_, frame.distance_unit, this->transform_, frame.block_num, frame.laser_num, frame.packet_num);
  }
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  this->frame_.DeviceToHost(0, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  std::memcpy(frame.points, this->frame_.cpu()->points, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  return 0;
}
template<typename T_Point>
int Udp2_5ParserGpu<T_Point>::LoadCorrectionString(char *data) {
  if (LoadCorrectionDatData(data) == 0) {
    return 0;
  }
  return LoadCorrectionCsvData(data);
}


// csv ----> correction
template<typename T_Point>
int  Udp2_5ParserGpu<T_Point>::LoadCorrectionCsvData(char *correction_string)
{
  std::string correction_content_str = correction_string;
  std::istringstream ifs(correction_content_str);
  std::string line;

  // skip first line "Laser id,Elevation,Azimuth" or "eeff"
  std::getline(ifs, line);  
  float elevation_list[MAX_LASER_NUM], azimuth_list[MAX_LASER_NUM];
  std::vector<std::string> vfirstLine;
  split_string(vfirstLine, line, ',');
  if (vfirstLine[0] == "EEFF" || vfirstLine[0] == "eeff") {
    // skip second line
    std::getline(ifs, line);  
  }

  int lineCount = 0;
  while (std::getline(ifs, line)) {
    std::vector<std::string> vLineSplit;
    split_string(vLineSplit, line, ',');
    // skip error line or hash value line
    if (vLineSplit.size() < 3) {  
      continue;
    } else {
      lineCount++;
    }
    float elevation, azimuth;
    int laserId = 0;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> laserId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elevation;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (laserId > MAX_LASER_NUM || laserId <= 0) {
      LogFatal("laser id is wrong in correction file. laser Id: %d, line: %d", laserId, lineCount);
      continue;
    }
    if (laserId != lineCount) {
      LogWarning("laser id is wrong in correction file. laser Id: %d, line: %d.  continue", laserId, lineCount);
      continue;
    }
    elevation_list[laserId - 1] = elevation;
    azimuth_list[laserId - 1] = azimuth;
  }

  for (int i = 0; i < lineCount; ++i) {
    corrections_.azimuths[i] = azimuth_list[i];
    corrections_.elevations[i] = elevation_list[i];
    LogDebug("%d %f %f",i, corrections_.azimuths[i], corrections_.elevations[i]);
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
int Udp2_5ParserGpu<T_Point>::LoadCorrectionDatData(char *data) {
  try {
    char *p = data;
    struct ETCorrectionsHeader ETheader = *((struct ETCorrectionsHeader* )p);
    if (0xee == ETheader.delimiter[0] && 0xff == ETheader.delimiter[1]) {
      switch (ETheader.min_version) {
        case 1: {
          ETCorrectionsHeader_V1V2 correction_v1;
          memcpy((void *)&correction_v1, p, sizeof(struct ETCorrectionsHeader_V1V2));
          corrections_.header.getDataFromV1V2(correction_v1);
          corrections_.min_version = corrections_.header.min_version;
          p += sizeof(ETCorrectionsHeader_V1V2);
          auto channel_num = corrections_.header.channel_number;
          uint16_t division = corrections_.header.angle_division;
          if ((channel_num > ET_MAX_CHANNEL_NUM - 3) || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(uint32_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.header.gamma)) / division;
          LogDebug("apha:%f, beta:%f, gamma:%f", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            LogDebug("%d %f %f",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          corrections_.azimuth_adjust_interval = 0;
          corrections_.elevation_adjust_interval = 0;
          
          memcpy((void*)&corrections_.SHA_value, p, 32);
          // successed
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(corrections_.azimuths)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(corrections_.elevations)));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections_.azimuths, sizeof(corrections_.azimuths), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections_.elevations, sizeof(corrections_.elevations), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          return 0;
        } break;
        case 2: {
          ETCorrectionsHeader_V1V2 correction_v1;
          memcpy((void *)&correction_v1, p, sizeof(struct ETCorrectionsHeader_V1V2));
          corrections_.header.getDataFromV1V2(correction_v1);
          corrections_.min_version = corrections_.header.min_version;
          p += sizeof(ETCorrectionsHeader_V1V2);
          auto channel_num = corrections_.header.channel_number;
          uint16_t division = corrections_.header.angle_division;
          if ((channel_num > ET_MAX_CHANNEL_NUM - 3) || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.header.gamma)) / division;
          LogDebug("apha:%f, beta:%f, gamma:%f", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            LogDebug("%d %f %f",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          corrections_.azimuth_adjust_interval = *((char*)p);
          p = p + 1;
          corrections_.elevation_adjust_interval = *((char*)p);
          p = p + 1;
          int angle_offset_len = (120 / (corrections_.azimuth_adjust_interval * 0.5) + 1) * (25 / (corrections_.elevation_adjust_interval * 0.5) + 1);
          memcpy((void*)corrections_.azimuth_adjust, p, sizeof(int16_t) * angle_offset_len);
          p = p + sizeof(int16_t) * angle_offset_len;
          memcpy((void*)corrections_.elevation_adjust, p, sizeof(int16_t) * angle_offset_len); 
          p = p + sizeof(int16_t) * angle_offset_len;
          for (int i = 0; i < angle_offset_len; i++) {
            corrections_.azimuth_adjust_f[i] = 1.f * corrections_.azimuth_adjust[i] / corrections_.header.angle_division;
            corrections_.elevation_adjust_f[i] = 1.f * corrections_.elevation_adjust[i] / corrections_.header.angle_division;
          }
          // int adjustNum = channel_num;
          memcpy((void*)&corrections_.SHA_value, p, 32);
          // successed
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(corrections_.azimuths)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(corrections_.elevations)));
          CUDACheck(cudaMalloc(&channel_azimuths_adjust_cu_, sizeof(corrections_.azimuth_adjust_f)));
          CUDACheck(cudaMalloc(&channel_elevations_adjust_cu_, sizeof(corrections_.elevation_adjust_f)));
          CUDACheck(cudaMemcpy(channel_azimuths_adjust_cu_, corrections_.azimuth_adjust_f, sizeof(corrections_.azimuth_adjust_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_adjust_cu_, corrections_.elevation_adjust_f, sizeof(corrections_.elevation_adjust_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections_.azimuths, sizeof(corrections_.azimuths), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections_.elevations, sizeof(corrections_.elevations), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          return 0;
        } break;
        case 3: {
          memcpy((void *)&corrections_.header, p, sizeof(struct ETCorrectionsHeader));
          p += sizeof(ETCorrectionsHeader);
          corrections_.min_version = corrections_.header.min_version;
          auto channel_num = corrections_.header.channel_number;
          uint16_t division = corrections_.header.angle_division;
          if ((channel_num > ET_MAX_CHANNEL_NUM - 3) || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.header.gamma)) / division;
          LogDebug("apha:%f, beta:%f, gamma:%f", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            LogDebug("%d %f %f ",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          corrections_.azimuth_adjust_interval = *((char*)p);
          p = p + 1;
          corrections_.elevation_adjust_interval = *((char*)p);
          p = p + 1;
          int angle_offset_len = (120 / (corrections_.azimuth_adjust_interval * 0.5) + 1) * (25 / (corrections_.elevation_adjust_interval * 0.5) + 1);
          memcpy((void*)corrections_.azimuth_adjust, p, sizeof(int16_t) * angle_offset_len);
          p = p + sizeof(int16_t) * angle_offset_len;
          memcpy((void*)corrections_.elevation_adjust, p, sizeof(int16_t) * angle_offset_len); 
          p = p + sizeof(int16_t) * angle_offset_len;
          for (int i = 0; i < angle_offset_len; i++) {
            corrections_.azimuth_adjust_f[i] = 1.f * corrections_.azimuth_adjust[i] / corrections_.header.angle_division;
            corrections_.elevation_adjust_f[i] = 1.f * corrections_.elevation_adjust[i] / corrections_.header.angle_division;
          }
          // int adjustNum = channel_num;
          memcpy((void*)&corrections_.SHA_value, p, 32);
          // successed
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(corrections_.azimuths)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(corrections_.elevations)));
          CUDACheck(cudaMalloc(&channel_azimuths_adjust_cu_, sizeof(corrections_.azimuth_adjust_f)));
          CUDACheck(cudaMalloc(&channel_elevations_adjust_cu_, sizeof(corrections_.elevation_adjust_f)));
          CUDACheck(cudaMemcpy(channel_azimuths_adjust_cu_, corrections_.azimuth_adjust_f, sizeof(corrections_.azimuth_adjust_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_adjust_cu_, corrections_.elevation_adjust_f, sizeof(corrections_.elevation_adjust_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections_.azimuths, sizeof(corrections_.azimuths), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections_.elevations, sizeof(corrections_.elevations), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          return 0;
        } break;
        case 4: {
          ETCorrectionsHeader_V3_4 correction_v3_4;
          memcpy((void *)&correction_v3_4, p, sizeof(struct ETCorrectionsHeader_V3_4));
          p += sizeof(ETCorrectionsHeader_V3_4);
          corrections_.header.getDataFromV3_4(correction_v3_4);
          corrections_.turn_number_per_frame = correction_v3_4.turn_number_per_frame;
          corrections_.min_version = corrections_.header.min_version;
          auto N = corrections_.header.channel_number;
          auto M = corrections_.header.mirror_number_reserved3;
          auto T = corrections_.turn_number_per_frame;
          auto division = corrections_.header.angle_division;
          if ((N > ET_MAX_CHANNEL_NUM - 3) || division == 0 || M > 8) {
            LogError("data error: channel_num is %u, division is %u, mirror_number is %u", N, division, M);
            return -1;
          }
          memcpy((void *)&corrections_.gamma, p,
                 sizeof(int16_t) * M);
          p += sizeof(int16_t) * M;
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * N);
          p += sizeof(int16_t) * N;
          memcpy((void *)&corrections_.raw_elevations, p,
               sizeof(int16_t) * N);
          p += sizeof(int16_t) * N;
          memcpy((void *)&corrections_.azimuth_offset_delta, p,
               sizeof(int16_t) * M);
          p += sizeof(int16_t) * M;
          memcpy((void *)&corrections_.elevation_offset_delta, p,
               sizeof(int16_t) * M);
          p += sizeof(int16_t) * M;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          for (int i = 0; i < M; i++) {
            corrections_.gamma_f[i] = ((float)(corrections_.gamma[i])) / division;
            corrections_.azimuth_offset_delta_f[i] = ((float)(corrections_.azimuth_offset_delta[i])) / division;
            corrections_.elevation_offset_delta_f[i] = ((float)(corrections_.elevation_offset_delta[i])) / division;
          }
          for (int i = 0; i < N; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
          }
          corrections_.azimuth_adjust_interval = *((char*)p);
          p = p + 1;
          corrections_.elevation_adjust_interval = *((char*)p);
          p = p + 1;
          int angle_offset_len = T * (120 / (corrections_.azimuth_adjust_interval * 0.1) + 1) * (3.2 / (corrections_.elevation_adjust_interval * 0.1) + 1);
          memcpy((void*)corrections_.azimuth_adjust, p, sizeof(int16_t) * angle_offset_len);
          p = p + sizeof(int16_t) * angle_offset_len;
          memcpy((void*)corrections_.elevation_adjust, p, sizeof(int16_t) * angle_offset_len); 
          p = p + sizeof(int16_t) * angle_offset_len;
          for (int i = 0; i < angle_offset_len; i++) {
            corrections_.azimuth_adjust_f[i] = 1.f * corrections_.azimuth_adjust[i] / corrections_.header.angle_division;
            corrections_.elevation_adjust_f[i] = 1.f * corrections_.elevation_adjust[i] / corrections_.header.angle_division;
          }
          // int adjustNum = channel_num;
          memcpy((void*)&corrections_.SHA_value, p, 32);
          // successed
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(corrections_.azimuths)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(corrections_.elevations)));
          CUDACheck(cudaMalloc(&channel_azimuths_adjust_cu_, sizeof(corrections_.azimuth_adjust_f)));
          CUDACheck(cudaMalloc(&channel_elevations_adjust_cu_, sizeof(corrections_.elevation_adjust_f)));
          CUDACheck(cudaMalloc(&gamma_cu, sizeof(corrections_.gamma_f)));
          CUDACheck(cudaMalloc(&elevation_offset_delta_cu, sizeof(corrections_.elevation_offset_delta_f)));
          CUDACheck(cudaMalloc(&azimuth_offset_delta_cu, sizeof(corrections_.azimuth_offset_delta_f)));
          CUDACheck(cudaMemcpy(channel_azimuths_adjust_cu_, corrections_.azimuth_adjust_f, sizeof(corrections_.azimuth_adjust_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_adjust_cu_, corrections_.elevation_adjust_f, sizeof(corrections_.elevation_adjust_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, corrections_.azimuths, sizeof(corrections_.azimuths), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, corrections_.elevations, sizeof(corrections_.elevations), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(gamma_cu, corrections_.gamma_f, sizeof(corrections_.gamma_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(elevation_offset_delta_cu, corrections_.elevation_offset_delta_f, sizeof(corrections_.elevation_offset_delta_f), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(azimuth_offset_delta_cu, corrections_.azimuth_offset_delta_f, sizeof(corrections_.azimuth_offset_delta_f), cudaMemcpyHostToDevice));
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
int Udp2_5ParserGpu<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
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

