/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

#include "lidar.h"

namespace hesai
{
namespace lidar
{
template <typename T_Point>
class HesaiLidarSdk 
{
private:
  boost::thread *runing_thread_ptr_;
  std::function<void(const UdpFrame_t&)> pkt_cb_;
  std::function<void(const LidarDecodedFrame<T_Point>&)> point_cloud_cb_;
  bool is_thread_runing_;
public:
  HesaiLidarSdk() {
    std::cout << "-------- Hesai Lidar SDK V" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << " --------" << std::endl;
    runing_thread_ptr_ = nullptr;
    lidar_ptr_ = nullptr;
    is_thread_runing_ = false;
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
    runing_thread_ptr_ = new boost::thread(boost::bind(&HesaiLidarSdk::Run, this));  
  }

  // process thread
  void Run()
  {
    is_thread_runing_ = true;
    UdpFrame_t udp_packet_frame;
    LidarDecodedPacket<T_Point> decoded_packet;
    int packet_index = 0;
    uint32_t start =  GetTickCount();
    UdpPacket packet;
    while (is_thread_runing_) {

      //get one packte from origin_packets_buffer_, which receive data from upd or pcap thread
      int ret = lidar_ptr_->GetOnePacket(packet);
      if (ret == -1) continue;

      //get distance azimuth reflection, etc.and put them into decode_packet
      lidar_ptr_->DecodePacket(decoded_packet, packet);

      //one frame is receive completely, split frame
      if(decoded_packet.scan_complete) {
        //waiting for parser thread compute xyzi of points in the same frame
        while(!lidar_ptr_->ComputeXYZIComplete(decoded_packet.packet_index)) usleep(100);
        uint32_t end =  GetTickCount();
        //log info, display frame message
        if (lidar_ptr_->frame_.points_num > kMinPointsOfOneFrame) {
          // LogInfo("frame:%d   points:%u  packet:%d  time:%lf %lf\n",lidar_ptr_->frame_.frame_index,  lidar_ptr_->frame_.points_num, packet_index, lidar_ptr_->frame_.points[0].timestamp, lidar_ptr_->frame_.points[lidar_ptr_->frame_.points_num - 1].timestamp) ;
          
          //publish point cloud topic
          if(point_cloud_cb_) point_cloud_cb_(lidar_ptr_->frame_);

          //publish upd packet topic
          if(pkt_cb_) pkt_cb_(udp_packet_frame);
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
      }
    }
  } 
  // assign callback fuction
  void RegRecvCallback(const std::function<void(const LidarDecodedFrame<T_Point>&)>& callback) {
    point_cloud_cb_ = callback;
  }

  // assign callback fuction
  void RegRecvCallback(const std::function<void(const UdpFrame_t&)>& callback) {
    pkt_cb_ = callback;
  }
};

}  // namespace lidar
}  // namespace hesai
