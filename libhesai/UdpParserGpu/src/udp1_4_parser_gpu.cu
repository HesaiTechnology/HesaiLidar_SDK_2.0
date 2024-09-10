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

#include "udp1_4_parser_gpu.h"
#include "safe_call.cuh"
#include "return_code.h"

using namespace hesai::lidar;
template <typename T_Point>
Udp1_4ParserGpu<T_Point>::Udp1_4ParserGpu() {
  corrections_loaded_ = false;
  cudaSafeMalloc(frame_data_cu_, FRAME_DATA_LEN);
}
template <typename T_Point>
Udp1_4ParserGpu<T_Point>::~Udp1_4ParserGpu() {
  cudaSafeFree(frame_data_cu_);
  if (corrections_loaded_) {
    cudaSafeFree(channel_elevations_cu_);
    cudaSafeFree(channel_azimuths_cu_);
    corrections_loaded_ = false;
  }
}
template <typename T_Point>
__global__ void compute_xyzs_1_4_impl(T_Point *xyzs, const float* channel_azimuths, const float* channel_elevations, const uint8_t* frame_data, 
    const double raw_distance_unit, Transform transform, LidarOpticalCenter optical_center, const uint16_t blocknum, const uint16_t lasernum, const uint16_t packet_index) {
  auto iscan = blockIdx.x;
  auto ichannel = threadIdx.x;
  if (iscan >= packet_index || ichannel >= blocknum * lasernum) return;
  float azimuth = AZIMUTH_GET[frame_data, iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))] / kHalfCircleFloat * M_PI;
  float theta = (channel_azimuths[(ichannel % lasernum)] * kResolutionFloat) / kHalfCircleFloat * M_PI;
  float phi = (channel_elevations[(ichannel % lasernum)] * kResolutionFloat) / kHalfCircleFloat * M_PI;
  float rho = DISTANCES_GET[frame_data, iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))] * raw_distance_unit;

  if(rho > 0.09 && optical_center.x != 0) {
    float tx = std::cos(phi) * std::sin(theta);
    float ty = std::cos(phi) * std::cos(theta);
    float tz = std::sin(phi);
    float B = 2 * tx * optical_center.x + 2 * ty * optical_center.y + 2 * tz * optical_center.z;
    float C = optical_center.x * optical_center.x + optical_center.y * optical_center.y + optical_center.z * optical_center.z - rho * rho;
    float d_opitcal = std::sqrt(B * B / 4 - C) - B / 2;
    float x = d_opitcal * tx + optical_center.x;
    float y = d_opitcal * ty + optical_center.y;
    float z = d_opitcal * tz + optical_center.z;
    theta = azimuth + std::atan(x / y);
    phi = std::asin(z / rho);
  } else {
    theta += azimuth;
  }

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
  gpu::setIntensity(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], REFLECTIVITIES_GET[frame_data, iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))]);
  gpu::setTimestamp(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], double(SENSOR_TIMESTAMP_GET[frame_data, iscan]) / kMicrosecondToSecond);
  gpu::setRing(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], ichannel % lasernum);
  gpu::setConfidence(xyzs[iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))], CONFIDENCE_GET[frame_data, iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum))]);
}
template <typename T_Point>
int Udp1_4ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!corrections_loaded_) return int(ReturnCode::CorrectionsUnloaded);          
  cudaSafeCall(cudaMemcpy(frame_data_cu_, frame.total_memory + FRAME_DATA_OFFSET,
                          FRAME_DATA_LEN, cudaMemcpyHostToDevice),
               ReturnCode::CudaMemcpyHostToDeviceError);                             
compute_xyzs_1_4_impl<<<kMaxPacketNumPerFrame, kMaxPointsNumPerPacket>>>(this->frame_.gpu()->points, channel_azimuths_cu_, channel_elevations_cu_, 
  frame_data_cu_, frame.distance_unit, this->transform_, this->optical_center, frame.block_num, frame.laser_num, frame.packet_num);
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  this->frame_.DeviceToHost();
  std::memcpy(frame.points, this->frame_.cpu()->points, sizeof(T_Point) * kMaxPacketNumPerFrame * kMaxPointsNumPerPacket);
  return 0;
}
template <typename T_Point>
int Udp1_4ParserGpu<T_Point>::LoadCorrectionString(char *correction_content) {
  if (corrections_loaded_) {
    return 0;
    if (channel_elevations_cu_) cudaFree(channel_elevations_cu_);
    if (channel_azimuths_cu_) cudaFree(channel_azimuths_cu_);
    corrections_loaded_ = false;
  }
  std::string correction_content_str = correction_content;
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
      LogError("laser id is wrong in correction file. laser Id: %d, line: %d", laserId, lineCount);
    }
    elevation_list[laserId - 1] = elevation;
    azimuth_list[laserId - 1] = azimuth;
  }
  CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(float) * MAX_LASER_NUM));
  CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(float) * MAX_LASER_NUM));
  CUDACheck(cudaMemcpy(channel_azimuths_cu_, azimuth_list, sizeof(float) * MAX_LASER_NUM, cudaMemcpyHostToDevice));
  CUDACheck(cudaMemcpy(channel_elevations_cu_, elevation_list, sizeof(float) * MAX_LASER_NUM, cudaMemcpyHostToDevice));
  corrections_loaded_ = true;
  return 0;
}
template <typename T_Point>
int Udp1_4ParserGpu<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
  int ret = 0;
  LogInfo("load correction file from local correction.csv now!");
  std::ifstream fin(lidar_correction_file);
  if (fin.is_open()) {
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
