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

#include "lidar.h"
#include "udp_parser_gpu.h"
namespace hesai
{
namespace lidar
{
template <typename T_Point>
class HesaiLidarSdkGpu
{
private:
  boost::thread *runing_thread_ptr_;
  std::function<void(const UdpFrame_t &)> pkt_cb_;
  std::function<void(const LidarDecodedFrame<T_Point> &)> point_cloud_cb_;
  bool is_thread_runing_;

public:
  HesaiLidarSdkGpu()
  {
    std::cout << "-------- Hesai Lidar SDK Gpu V" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << " --------" << std::endl;
    runing_thread_ptr_ = nullptr;
    lidar_ptr_ = nullptr;
    is_thread_runing_ = false;
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

    //init lidar wiht param
    lidar_ptr_->Init(param);

    //get lidar type from Lidar class
    std::string lidar_type_from_parser = lidar_ptr_->GetLidarType();

    //init gpu parser with lidar type
    gpu_parser_ptr_->SetLidarType(lidar_type_from_parser);

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
    runing_thread_ptr_ = new boost::thread(boost::bind(&HesaiLidarSdkGpu::Run, this));
  }
  // process thread
  void Run()
  {
    is_thread_runing_ = true;
    UdpFrame_t udp_packet_frame;
    LidarDecodedPacket<T_Point> decoded_packet;
    LidarDecodedFrame<T_Point> frame;
    int packet_index = 0;
    uint32_t start = GetTickCount();
    while (is_thread_runing_)
    {
      UdpPacket packet;

      //get one packte from origin_packets_buffer_, which receive data from upd or pcap thread
      int ret = lidar_ptr_->GetOnePacket(packet);
      if (ret == -1) continue;

      //get distance azimuth reflection, etc.and put them into decode_packet
      int res = lidar_ptr_->DecodePacket(decoded_packet, packet);

      //one frame is receive completely, split frame
      if (decoded_packet.scan_complete)
      {
        //compute xyzi of points in one frame, using gpu device
        int ret = gpu_parser_ptr_->ComputeXYZI(frame);  
        frame.packet_num = packet_index;

        //log info, display frame message
        if (frame.points_num > kMinPointsOfOneFrame) {
          // LogInfo("frame:%d   points:%u  packet:%d  time:%lf\n", frame.frame_index, frame.points_num, packet_index, frame.points[0].timestamp);
          
          //publish point cloud topic
          if(point_cloud_cb_) point_cloud_cb_(frame);

          //publish upd packet topic
          if(pkt_cb_) pkt_cb_(udp_packet_frame);
        }

        //reset frame variable
        frame.Update();

        //clear udp packet vector
        udp_packet_frame.clear();
        packet_index = 0;

        //if the packet which contains split frame msgs is vaild, it will be the first packet of new frame
        if(decoded_packet.IsDecodedPacketValid()) {
          decoded_packet.packet_index = packet_index;
          udp_packet_frame.emplace_back(packet);
          //copy distances, reflection, azimuth, elevation from decoded packet to frame container
          memcpy((frame.distances + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.distances), decoded_packet.block_num * decoded_packet.laser_num * sizeof(uint16_t));
          memcpy((frame.reflectivities + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.reflectivities), decoded_packet.block_num * decoded_packet.laser_num * sizeof(uint8_t));
          memcpy((frame.azimuth + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.azimuth), decoded_packet.block_num * decoded_packet.laser_num * sizeof(float));
          memcpy((frame.elevation + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.elevation), decoded_packet.block_num * decoded_packet.laser_num * sizeof(float));
          frame.distance_unit = decoded_packet.distance_unit;
          frame.sensor_timestamp[packet_index] = decoded_packet.sensor_timestamp;
          frame.points_num = frame.points_num + decoded_packet.block_num * decoded_packet.laser_num;
          packet_index++;
        }
        
      }
      else
      {
        //new decoded packet of one frame, put it into frame container
        decoded_packet.packet_index = packet_index;
        udp_packet_frame.emplace_back(packet);
        //copy distances, reflection, azimuth, elevation from decoded packet to frame container
        memcpy((frame.distances + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.distances), decoded_packet.block_num * decoded_packet.laser_num * sizeof(uint16_t));
        memcpy((frame.reflectivities + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.reflectivities), decoded_packet.block_num * decoded_packet.laser_num * sizeof(uint8_t));
        memcpy((frame.azimuth + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.azimuth), decoded_packet.block_num * decoded_packet.laser_num * sizeof(float));
        memcpy((frame.elevation + packet_index * decoded_packet.block_num * decoded_packet.laser_num), (decoded_packet.elevation), decoded_packet.block_num * decoded_packet.laser_num * sizeof(float));
        frame.distance_unit = decoded_packet.distance_unit;
        frame.sensor_timestamp[packet_index] = decoded_packet.sensor_timestamp;
        frame.points_num = frame.points_num + decoded_packet.block_num * decoded_packet.laser_num;
        if (decoded_packet.block_num != 0 && decoded_packet.laser_num != 0) {
            frame.laser_num = decoded_packet.laser_num;
            frame.block_num = decoded_packet.block_num;
        }
        packet_index++;
        //update status manually if start a new frame failedly
        if (packet_index > kMaxPacketNumPerFrame) {
          packet_index = 0;
          udp_packet_frame.clear();
          lidar_ptr_->frame_.Update();
          printf("fail to start a new frame\n");
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
  void RegRecvCallback(const std::function<void(const UdpFrame_t &)>& callback)
  {
    pkt_cb_ = callback;
  }
};

}  // namespace lidar
}  // namespace hesai
