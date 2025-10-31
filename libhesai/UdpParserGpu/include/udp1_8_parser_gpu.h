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
#ifndef Udp1_8_PARSER_GPU_H_
#define Udp1_8_PARSER_GPU_H_
#include "general_parser_gpu.h"
#include "udp_protocol_v1_8.h"
namespace hesai
{
namespace lidar
{

int compute_1_8_cuda(uint8_t* point_cloud_cu_, CudaPointXYZAER* points_cu_, uint32_t point_cloud_size, const float* correction_azi, 
    const float* correction_ele, const FrameDecodeParam* fParam, LidarOpticalCenter optical_center, 
    uint32_t packet_num, uint16_t block_num, uint16_t channel_num);

// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class Udp1_8ParserGpu: public GeneralParserGpu<T_Point>{
 private:
 public:
  Udp1_8ParserGpu(uint16_t maxPacket, uint16_t maxPoint) 
     : GeneralParserGpu<T_Point>(maxPacket, maxPoint) {
    this->optical_center.setNoFlag(LidarOpticalCenter{-0.00625, 0.010955, 0.003911});
  }
  ~Udp1_8ParserGpu() {
  }

  virtual void LoadFiretimesStruct(void *) {
  }
  virtual void updateFiretimeFile() {
  }

  // compute xyzi of points from decoded packet， use gpu device
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
    if (!*this->get_correction_file_) return int(ReturnCode::CorrectionsUnloaded);  
    if (!this->init_suc_flag_) return int(ReturnCode::CudaInitError);
    this->reMalloc(frame.maxPacketPerFrame, frame.maxPointPerPacket, frame.point_cloud_size);
    cudaSafeCall(cudaMemcpy(this->point_cloud_cu_, frame.point_cloud_raw_data,
                            frame.point_cloud_size * frame.packet_num, 
                            cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);
    this->updateCorrectionFile();
    FrameDecodeParam cuda_Param = frame.fParam;
    int ret = compute_1_8_cuda(this->point_cloud_cu_, this->points_cu_, frame.point_cloud_size, this->correction_azi_cu_, 
      this->correction_ele_cu_, &cuda_Param, this->optical_center, frame.packet_num, frame.block_num, frame.laser_num);
    if (ret != 0) return ret;

    cudaSafeCall(cudaMemcpy(this->points_, this->points_cu_,
                            frame.per_points_num * frame.packet_num * sizeof(CudaPointXYZAER), 
                            cudaMemcpyDeviceToHost), ReturnCode::CudaMemcpyDeviceToHostError);
    for (uint32_t i = 0; i < frame.packet_num; i++) {
      auto &packetData = frame.packetData[i];
      int point_index = i * frame.per_points_num;
      int point_num = 0;
      for (uint32_t channel_index = 0; channel_index < frame.laser_num; channel_index++) {
        if(this->correction_ptr->display[channel_index] == false) {
          continue;
        }
        auto &point = this->points_[point_index + channel_index];
        if (this->IsChannelFovFilter(point.azimuthCalib, channel_index, frame.fParam) == 1) {
          continue;
        }
        int point_index_rerank = point_index + point_num;
        float azi_ = point.azimuthCalib; 
        float elev_ = point.elevationCalib; 
        GeneralParserGpu<T_Point>::DoRemake(azi_, elev_, frame.fParam.remake_config, point_index_rerank);
        if(point_index_rerank >= 0) { 
          auto& ptinfo = frame.points[point_index_rerank]; 
          set_x(ptinfo, point.x);
          set_y(ptinfo, point.y);
          set_z(ptinfo, point.z);
          set_ring(ptinfo, channel_index);
          set_intensity(ptinfo, point.reserved[0]);
          set_timestamp(ptinfo, double(packetData.t.sensor_timestamp) / kMicrosecondToSecond);
          if (frame.fParam.dirty_mapping_reflectance) {
            set_intensity(ptinfo, point.reserved[1] * 64);
          }
          point_num++;
        }
      }
      frame.valid_points[i] = point_num;
    }
    return 0;
  } 
};
}
}
#endif  // Udp1_8_PARSER_GPU_H_
