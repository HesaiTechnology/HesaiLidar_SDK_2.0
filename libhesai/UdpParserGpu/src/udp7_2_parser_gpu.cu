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
#include "udp7_2_parser_gpu.h"
#include "safe_call.cuh"
#include "return_code.h"

using namespace hesai::lidar;
template <typename T_Point>
Udp7_2ParserGpu<T_Point>::Udp7_2ParserGpu() {
  corrections_loaded_ = false;
  cudaSafeMalloc(point_data_cu_, POINT_DATA_LEN);
  cudaSafeMalloc(sensor_timestamp_cu_, SENSOR_TIMESTAMP_LEN);
}
template <typename T_Point>
Udp7_2ParserGpu<T_Point>::~Udp7_2ParserGpu() {
  cudaSafeFree(point_data_cu_);
  cudaSafeFree(sensor_timestamp_cu_);
  if (corrections_loaded_) {
    cudaSafeFree(channel_elevations_cu_);
    cudaSafeFree(channel_azimuths_cu_);
    corrections_loaded_ = false;
  }
}
template <typename T_Point>
__global__ void compute_xyzs_7_2_impl(T_Point *xyzs, const float* channel_azimuths, const float* channel_elevations, 
    const PointDecodeData* point_data, const uint64_t* sensor_timestamp, const double raw_distance_unit, Transform transform, 
    const int blocknum, const int lasernum, const uint16_t packet_index) {
  auto iscan = blockIdx.x;
  auto ichannel = threadIdx.x;
  if (iscan >= packet_index || ichannel >= blocknum * lasernum) return;
  int point_index = iscan * blocknum * lasernum + (ichannel % (lasernum * blocknum));
  float azimuth = point_data[point_index].azimuth / HALF_CIRCLE * M_PI;
  float elevation = point_data[point_index].elevation / HALF_CIRCLE * M_PI;

  auto rho = point_data[point_index].distances * raw_distance_unit;
  float z = rho * sin(elevation);
  auto r = rho * cosf(elevation);
  float x = r * sin(azimuth);
  float y = r * cos(azimuth);

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
int Udp7_2ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!corrections_loaded_) return int(ReturnCode::CorrectionsUnloaded);      
  cudaSafeCall(cudaMemcpy(point_data_cu_, frame.pointData,
                          frame.block_num * frame.laser_num * frame.packet_num * sizeof(PointDecodeData), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);
  cudaSafeCall(cudaMemcpy(sensor_timestamp_cu_, frame.sensor_timestamp,
                          frame.packet_num * sizeof(uint64_t), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError); 
compute_xyzs_7_2_impl<<<frame.packet_num, frame.block_num * frame.laser_num>>>(this->frame_.gpu()->points, channel_azimuths_cu_, channel_elevations_cu_, 
   point_data_cu_, sensor_timestamp_cu_, frame.distance_unit, this->transform_, frame.block_num, frame.laser_num, frame.packet_num);
  cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  this->frame_.DeviceToHost(0, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  std::memcpy(frame.points, this->frame_.cpu()->points, frame.block_num * frame.laser_num * frame.packet_num * sizeof(T_Point));
  return 0;
}
template<typename T_Point>
int Udp7_2ParserGpu<T_Point>::LoadCorrectionString(char *data) {
  if (LoadCorrectionDatData(data)) {
    return 0;
  }
  return LoadCorrectionCsvData(data);
}

template<typename T_Point>
int Udp7_2ParserGpu<T_Point>::LoadCorrectionCsvData(char *correction_string) {
  std::istringstream ifs(correction_string);
	std::string line;
  // first line "Laser id,Elevation,Azimuth"
	if(std::getline(ifs, line)) {  
		LogInfo("Parse Lidar Correction...");
	}
	int lineCounter = 0;
	std::vector<std::string>  firstLine;
	split_string(firstLine, line, ',');
  float elevations[CHANNEL_MAX][COLUMN_MAX];
  float azimuths[CHANNEL_MAX][COLUMN_MAX];
  while (std::getline(ifs, line)) {
    if(line.length() < strlen("1,1,1,1")) {
      return -1;
    } 
    else {
      lineCounter++;
    }
    float elev, azimuth;
    int lineId = 0;
    int columnId = 0;
    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> columnId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;
    if (lineId > CHANNEL_MAX || lineId <= 0 || columnId > COLUMN_MAX || columnId <= 0){
      LogError("data error, lineId:%d, columnId:%d", lineId, columnId);
      continue;
    }
    elevations[lineId - 1][columnId - 1] = elev * 100;
    azimuths[lineId - 1][columnId - 1] = azimuth * 100;
  }
  CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(azimuths)));
  CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(elevations)));
  CUDACheck(cudaMemcpy(channel_azimuths_cu_, azimuths, sizeof(azimuths), cudaMemcpyHostToDevice));
  CUDACheck(cudaMemcpy(channel_elevations_cu_, elevations, sizeof(elevations), cudaMemcpyHostToDevice));
  corrections_loaded_ = true;
	return 0;
}

template<typename T_Point>
int Udp7_2ParserGpu<T_Point>::LoadCorrectionDatData(char *correction_string) {
  float elevations[CHANNEL_MAX][COLUMN_MAX];
  float azimuths[CHANNEL_MAX][COLUMN_MAX];
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
                  azimuths[col][row] = angles[idx] * fResolution;
              }
          }

          for (int row = 0; row < column_num; row++) {
              for (int col = 0; col < channel_num; col++) {
                  int idx = angleNum + row * channel_num + col;
                  elevations[col][row] = angles[idx] * fResolution;
              }
          }
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(azimuths)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(elevations)));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, azimuths, sizeof(azimuths), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, elevations, sizeof(elevations), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          delete[] angles;
          delete[] hashValue;
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
                  azimuths[col][row] = angles[idx] * fResolution;
              }
          }

          for (int row = 0; row < column_num; row++) {
              for (int col = 0; col < channel_num; col++) {
                  int idx = angleNum + row * channel_num + col;
                  elevations[col][row] = angles[idx] * fResolution;
              }
          }
          CUDACheck(cudaMalloc(&channel_azimuths_cu_, sizeof(azimuths)));
          CUDACheck(cudaMalloc(&channel_elevations_cu_, sizeof(elevations)));
          CUDACheck(cudaMemcpy(channel_azimuths_cu_, azimuths, sizeof(azimuths), cudaMemcpyHostToDevice));
          CUDACheck(cudaMemcpy(channel_elevations_cu_, elevations, sizeof(elevations), cudaMemcpyHostToDevice));
          corrections_loaded_ = true;
          delete[] angles;
          delete[] hashValue;
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
int Udp7_2ParserGpu<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
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
