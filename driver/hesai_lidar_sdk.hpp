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
#include "fault_message.h"

namespace hesai
{
namespace lidar
{
template <typename T_Point>
class HesaiLidarSdk 
{
private:
  std::thread *runing_thread_ptr_;
  std::function<void(const UdpFrame_t&, double)> pkt_cb_;
  std::function<void(const LidarDecodedFrame<T_Point>&)> point_cloud_cb_;
  std::function<void(const u8Array_t&)> correction_cb_;
  std::function<void(const uint32_t &, const uint32_t &)> pkt_loss_cb_;
  std::function<void(const uint8_t&, const u8Array_t&)> ptp_cb_;
  bool is_thread_runing_;
  bool packet_loss_tool_;
public:
  HesaiLidarSdk() {
    std::cout << "-------- Hesai Lidar SDK V" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << " --------" << std::endl;
    runing_thread_ptr_ = nullptr;
    lidar_ptr_ = nullptr;
    is_thread_runing_ = false;
    packet_loss_tool_ = false;
  }

  ~HesaiLidarSdk() {
    Stop();
  }
  Lidar<T_Point> *lidar_ptr_;

  //init lidar with param. init logger, udp parser, source, ptc client, start receive/parser thread
  bool Init(const DriverParam& param) 
  {
   /*****************************Init decoder******************************************************/ 
    lidar_ptr_ = new Lidar<T_Point>;
    if (nullptr == lidar_ptr_) {
      std::cout << "create Lidar fail" << std::endl;
      return false;
    }
    //init lidar with param
    lidar_ptr_->Init(param);
    //set packet_loss_tool
    packet_loss_tool_ = param.decoder_param.enable_packet_loss_tool;
    /***********************************************************************************/ 
    return true;
  }

  // stop process thread
  void Stop() {
    if (nullptr != runing_thread_ptr_) {
      is_thread_runing_ = false;
      runing_thread_ptr_->join();
      delete runing_thread_ptr_;
      runing_thread_ptr_ = nullptr;
    }

    if (nullptr != lidar_ptr_) {
      delete lidar_ptr_;
      lidar_ptr_ = nullptr;
    }  
  }
  // start process thread
  void Start() {
    runing_thread_ptr_ = new std::thread(std::bind(&HesaiLidarSdk::Run, this));  
  }

  // process thread
  void Run()
  {
    is_thread_runing_ = true;
    UdpFrame_t udp_packet_frame;
    LidarDecodedPacket<T_Point> decoded_packet;
    decoded_packet.use_timestamp_type = lidar_ptr_->use_timestamp_type_;
    decoded_packet.config.fov_start = lidar_ptr_->fov_start_;
    decoded_packet.config.fov_end = lidar_ptr_->fov_end_;
    int packet_index = 0;
    // uint32_t start = GetMicroTickCount();
    UdpPacket packet;
    FaultMessageInfo fault_message_info;

    uint32_t total_packet_count;
    uint32_t total_packet_loss_count;
    while (is_thread_runing_)
    {

      //get one packte from origin_packets_buffer_, which receive data from upd or pcap thread
      int ret = lidar_ptr_->GetOnePacket(packet);
      if (ret == -1) continue;
      //get fault message
      if (packet.packet_len == kFaultMessageLength) {
        FaultMessageCallback(packet, fault_message_info);
        continue;
      }

      //get distance azimuth reflection, etc.and put them into decode_packet
      lidar_ptr_->DecodePacket(decoded_packet, packet);

      //do not compute xyzi of points if enable packet_loss_tool_
      // if(packet_loss_tool_ == true) continue;

      //one frame is receive completely, split frame
      if(decoded_packet.scan_complete) {
        //waiting for parser thread compute xyzi of points in the same frame
        while(!lidar_ptr_->ComputeXYZIComplete(decoded_packet.packet_index)) std::this_thread::sleep_for(std::chrono::microseconds(100));
        // uint32_t end =  GetMicroTickCount();
        //log info, display frame message
        if (lidar_ptr_->frame_.points_num > kMinPointsOfOneFrame) {
          // LogInfo("frame:%d   points:%u  packet:%d  time:%lf %lf\n",lidar_ptr_->frame_.frame_index,  lidar_ptr_->frame_.points_num, packet_index, lidar_ptr_->frame_.points[0].timestamp, lidar_ptr_->frame_.points[lidar_ptr_->frame_.points_num - 1].timestamp) ;

          //publish point cloud topic
          if(point_cloud_cb_) point_cloud_cb_(lidar_ptr_->frame_);

          //publish upd packet topic
          if(pkt_cb_) pkt_cb_(udp_packet_frame, lidar_ptr_->frame_.points[0].timestamp);

          if (pkt_loss_cb_ )
          {
            total_packet_count = lidar_ptr_->udp_parser_->GetGeneralParser()->total_packet_count_;
            total_packet_loss_count = lidar_ptr_->udp_parser_->GetGeneralParser()->total_loss_count_;
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
              printf("-->%d %d %zu %zu\n", ret_status, ret_offset, ptp_status.size(), ptp_lock_offset.size());
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
        if(decoded_packet.IsDecodedPacketValid()) {
          decoded_packet.packet_index = packet_index;
          lidar_ptr_->ComputeXYZI(decoded_packet);
          udp_packet_frame.emplace_back(packet);
          packet_index++;
        }
      }
      else {
        //new decoded packet of one frame, put it into decoded_packets_buffer_ and compute xyzi of points
        decoded_packet.packet_index = packet_index;
        lidar_ptr_->ComputeXYZI(decoded_packet);
        udp_packet_frame.emplace_back(packet);
        packet_index++;

        //update status manually if start a new frame failedly
        if (packet_index >= kMaxPacketNumPerFrame) {
          packet_index = 0;
          udp_packet_frame.clear();
          lidar_ptr_->frame_.Update();
          printf("fail to start a new frame\n");
        }
      }
    }
  } 
  // assign callback fuction
  void RegRecvCallback(const std::function<void(const LidarDecodedFrame<T_Point>&)>& callback) {
    point_cloud_cb_ = callback;
  }

  // assign callback fuction
  void RegRecvCallback(const std::function<void(const UdpFrame_t&, double)>& callback) {
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
  //parsar fault message
  void FaultMessageCallback(UdpPacket& udp_packet, FaultMessageInfo& fault_message_info) {
     FaultMessageVersion3 *fault_message_ptr = 
      reinterpret_cast< FaultMessageVersion3*> (&(udp_packet.buffer[0]));
    fault_message_ptr->ParserFaultMessage(fault_message_info);
    return;
  }
};

}  // namespace lidar
}  // namespace hesai
