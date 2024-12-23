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
#pragma once
#include "lidar.h"
#include "udp_parser_gpu.h"
#include "fault_message.h"
namespace hesai
{
namespace lidar
{
template <typename T_Point>
class HesaiLidarSdkGpu
{
private:
  std::thread *runing_thread_ptr_;
  std::function<void(const UdpFrame_t &, double)> pkt_cb_;
  std::function<void(const LidarDecodedFrame<T_Point> &)> point_cloud_cb_;
  std::function<void(const u8Array_t&)> correction_cb_;
  std::function<void(const uint32_t &, const uint32_t &)> pkt_loss_cb_;
  std::function<void(const uint8_t&, const u8Array_t&)> ptp_cb_;
  std::function<void(const FaultMessageInfo&)> fault_message_cb_;
  std::function<void(const LidarImuData&)> imu_cb_;
  bool is_thread_runing_;
  bool packet_loss_tool_; 
  uint32_t device_ip_address_;
  uint16_t device_udp_port_;
  uint16_t device_fault_port_;

public:
  HesaiLidarSdkGpu()
  {
    std::cout << "-------- Hesai Lidar SDK Gpu V" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << " --------" << std::endl;
    runing_thread_ptr_ = nullptr;
    lidar_ptr_ = nullptr;
    is_thread_runing_ = false;
    packet_loss_tool_ = false;
  }
  ~HesaiLidarSdkGpu() {
    Stop();
  }
  hesai::lidar::Lidar<T_Point> *lidar_ptr_;
  UdpParserGpu<T_Point> *gpu_parser_ptr_;

  // stop process thread  
  void Stop()
  {
    if (nullptr != runing_thread_ptr_)
    {
      is_thread_runing_ = false;
      runing_thread_ptr_->join();
      delete runing_thread_ptr_;
      runing_thread_ptr_ = nullptr;
    }

    if (nullptr != lidar_ptr_)
    {
      delete lidar_ptr_;
      lidar_ptr_ = nullptr;
    }
  }
   //init lidar with param. init logger, udp parser, source, ptc client, start receive/parser thread
   //init gpu parser class 
  bool Init(const DriverParam &param)
  {
    /*****************************Init decoder******************************************************/
    lidar_ptr_ = new Lidar<T_Point>;
    gpu_parser_ptr_ = new UdpParserGpu<T_Point>;
    if (nullptr == lidar_ptr_)
    {
      std::cout << "create Lidar fail";
      return false;
    }

    //init lidar with param
    lidar_ptr_->Init(param);

    //set packet_loss_tool
    packet_loss_tool_ = param.decoder_param.enable_packet_loss_tool;

    device_ip_address_ = inet_addr(param.input_param.device_ip_address.c_str());
    if(param.input_param.device_fault_port >= 1024 && param.input_param.device_fault_port <= 65535 && device_ip_address_ != INADDR_NONE){
      device_fault_port_ = param.input_param.device_fault_port;
    }
    else{
      device_fault_port_ = 0;
    }
    if(param.input_param.device_udp_src_port >= 1024 && param.input_param.device_udp_src_port <= 65535 && device_ip_address_ != INADDR_NONE){
      device_udp_port_ = param.input_param.device_udp_src_port;
    }
    else{
      device_udp_port_ = 0;
    }

    //get lidar type from Lidar class
    std::string lidar_type_from_parser = lidar_ptr_->GetLidarType();

    //init gpu parser with lidar type
    gpu_parser_ptr_->SetLidarType(lidar_type_from_parser);
    gpu_parser_ptr_->SetXtSpotCorrection(param.decoder_param.xt_spot_correction && (lidar_type_from_parser == "PandarXT"));
    gpu_parser_ptr_->SetOpticalCenter(lidar_ptr_->GetUdpParser()->GetOpticalCenter());

    //set transform param
    gpu_parser_ptr_->SetTransformPara(param.decoder_param.transform_param.x,
                               param.decoder_param.transform_param.y,
                               param.decoder_param.transform_param.z,
                               param.decoder_param.transform_param.roll,
                               param.decoder_param.transform_param.pitch,
                               param.decoder_param.transform_param.yaw);
    
    /***********************************************************************************/

    /***************************Init source****************************************/


    //load correction file, depanding on source type
    if (param.input_param.source_type == 1)
    {
      u8Array_t sData;
      for (int i = 0; i < 3; i++)
      {
        int ret = lidar_ptr_->ptc_client_->GetCorrectionInfo(sData);
        if (ret != 0 || gpu_parser_ptr_->LoadCorrectionString((char *)sData.data()) != 0)
        {
          gpu_parser_ptr_->LoadCorrectionFile(param.input_param.correction_file_path);
        }
        else
        {
          break;
        }
      }
    }
    else
    {
      gpu_parser_ptr_->LoadCorrectionFile(param.input_param.correction_file_path);
    }
    /********************************************************************************/
    return true;
  }

  // start process thread
  void Start()
  {
    runing_thread_ptr_ = new std::thread(std::bind(&HesaiLidarSdkGpu::Run, this));
  }
  // process thread
  void Run()
  {
    is_thread_runing_ = true;
    UdpFrame_t udp_packet_frame;
    FaultMessageInfo fault_message_info;
    int packet_index = 0;
    uint32_t start = GetMicroTickCount();
    uint32_t total_packet_count;
    uint32_t total_packet_loss_count;    
    lidar_ptr_->frame_.use_timestamp_type = lidar_ptr_->use_timestamp_type_;
    lidar_ptr_->frame_.config.fov_start = lidar_ptr_->fov_start_;
    lidar_ptr_->frame_.config.fov_end = lidar_ptr_->fov_end_;
    while (is_thread_runing_)
    {
      UdpPacket packet;
      //get one packte from origin_packets_buffer_, which receive data from upd or pcap thread
      int ret = lidar_ptr_->GetOnePacket(packet);
      if (ret == -1) continue;

      //get fault message
      if (packet.packet_len == kFaultMessageLength) {
        if(device_fault_port_ != 0){
          if(device_ip_address_ != packet.ip || device_fault_port_ != packet.port){
            continue;
          }
        }
        ret = lidar_ptr_->ParserFaultMessage(packet, fault_message_info);
        if (ret == 0) {
          if (fault_message_cb_)
            fault_message_cb_(fault_message_info);
        }
        continue;
      }

      if(device_udp_port_ != 0 && (packet.is_timeout == false || packet_index == 0)){
        if(device_ip_address_ != packet.ip || device_udp_port_ != packet.port){
          continue;
        }
      }

      //get distance azimuth reflection, etc.and put them into decode_packet
      ret = lidar_ptr_->DecodePacket(lidar_ptr_->frame_, packet);
      if(ret != 0) {
        if (ret == 1) {
          imu_cb_(lidar_ptr_->frame_.imu_config);
        }
        continue;
      }

      //one frame is receive completely, split frame
      if (lidar_ptr_->frame_.scan_complete) {
        // If it's not a timeout split frame, it will be one more packet
        bool last_packet_is_valid = (lidar_ptr_->frame_.packet_num != packet_index);
        lidar_ptr_->frame_.packet_num = packet_index;
        //compute xyzi of points in one frame, using gpu device
        int ret = gpu_parser_ptr_->ComputeXYZI(lidar_ptr_->frame_);  

        //log info, display frame message
        if (ret == 0 && lidar_ptr_->frame_.points_num > kMinPointsOfOneFrame) {
          // printf("frame:%d   points:%u  packet:%d  time:%lf\n", frame.frame_index, frame.points_num, packet_index, frame.points[0].timestamp);
          
          //publish point cloud topic
          if(point_cloud_cb_) point_cloud_cb_(lidar_ptr_->frame_);

          //publish upd packet topic
          if(pkt_cb_) {
            double timestamp = 0;
            getTimestamp(lidar_ptr_->frame_.points[0], timestamp);
            pkt_cb_(udp_packet_frame, timestamp);
          }

          if (pkt_loss_cb_ )
          {
            total_packet_count = lidar_ptr_->udp_parser_->GetGeneralParser()->total_packet_count_;
            total_packet_loss_count = lidar_ptr_->udp_parser_->GetGeneralParser()->seqnum_loss_message_.total_loss_count;
            pkt_loss_cb_(total_packet_count, total_packet_loss_count);
          }
          if (ptp_cb_ && lidar_ptr_->frame_.frame_index % 100 == 1)
          {
            u8Array_t ptp_status;
            u8Array_t ptp_lock_offset;
            int ret_status = lidar_ptr_->ptc_client_->GetPTPDiagnostics(ptp_status, 1); // ptp_query_type = 1
            int ret_offset = lidar_ptr_->ptc_client_->GetPTPLockOffset(ptp_lock_offset);
            if (ret_status != 0 || ret_offset != 0)
            {
              printf("-->%d %d %lu %lu\n", ret_status, ret_offset, ptp_status.size(), ptp_lock_offset.size());
            }
            else
            {
              ptp_cb_(ptp_lock_offset.front(), ptp_status);
            }
          }
          if (correction_cb_ && lidar_ptr_->frame_.frame_index % 1000 == 1)
          {
            correction_cb_(lidar_ptr_->correction_string_);
          }
        }
        //reset frame variable
        lidar_ptr_->frame_.Update();

        //clear udp packet vector
        udp_packet_frame.clear();
        packet_index = 0;

        //if the packet which contains split frame msgs is vaild, it will be the first packet of new frame
        if(last_packet_is_valid) {
          lidar_ptr_->DecodePacket(lidar_ptr_->frame_, packet);
          udp_packet_frame.emplace_back(packet);
          lidar_ptr_->frame_.points_num += lidar_ptr_->frame_.per_points_num;
          packet_index++;
        }
      }
      else
      {
        if(lidar_ptr_->frame_.packet_num != packet_index) {
          udp_packet_frame.emplace_back(packet);
          lidar_ptr_->frame_.points_num += lidar_ptr_->frame_.per_points_num;
          packet_index++;
        }

        //update status manually if start a new frame failedly
        if (packet_index >= kMaxPacketNumPerFrame) {
          packet_index = 0;
          udp_packet_frame.clear();
          lidar_ptr_->frame_.Update();
          LogError("fail to start a new frame\n");
        }
      }
    }
  }

  // assign callback fuction
  void RegRecvCallback(const std::function<void(const LidarDecodedFrame<T_Point> &)>& callback)
  {
    point_cloud_cb_ = callback;
  }

  // assign callback fuction
  void RegRecvCallback(const std::function<void(const UdpFrame_t &, double)>& callback)
  {
    pkt_cb_ = callback;
  }

  // assign callback fuction
  void RegRecvCallback(const std::function<void (const u8Array_t&)>& callback) {
    correction_cb_ = callback;
  }

  void RegRecvCallback(const std::function<void (const uint32_t &, const uint32_t &)>& callback) {
    pkt_loss_cb_ = callback;
  }
  void RegRecvCallback(const std::function<void (const uint8_t&, const u8Array_t&)>& callback) {
    ptp_cb_ = callback;
  }
  void RegRecvCallback(const std::function<void (const FaultMessageInfo&)>& callback) {
    fault_message_cb_ = callback;
  }
  void RegRecvCallback(const std::function<void (const LidarImuData&)>& callback) {
    imu_cb_ = callback;
  }
};

}  // namespace lidar
}  // namespace hesai
