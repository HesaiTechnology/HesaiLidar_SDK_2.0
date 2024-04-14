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

/*
 * File:       lidar.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Define Lidar class 
 */

#ifndef Lidar_H
#define Lidar_H
#include <stdint.h>
#include <time.h>
#include "lidar_types.h"
#include "ptc_client.h"
#include <iostream>
#include <vector>
#include <thread>
#include "tcp_client.h"
#include "udp_parser.h"
#include "pcap_source.h"
#include "socket_source.h"
#include "blocking_ring.h"
#include "ring.h"
#include "ptc_client.h"
#include "driver_param.h"
#ifndef _MSC_VER
#include <endian.h>
#include <semaphore.h>
#endif
#define PKT_SIZE_40P (1262)
#define PKT_SIZE_AC (1256)
#define PKT_SIZE_64 (1194)
#define PKT_SIZE_20 (1270)
#define AT128E2X_PACKET_LEN (1180)
#define FAULTMESSAGE_PACKET_LEN (99)
#define GPS_PACKET_LEN (512)
#define CMD_SET_STANDBY_MODE (0x1c)
#define CMD_SET_SPIN_SPEED (0x17)

namespace hesai
{
namespace lidar
{

// class Lidar
// the Lidar class will start a udp data recieve thread and parser thread when init()
// udp packet data will be recived in udp data recieve thread and put into origin_packets_buffer_
// you can get origin udp packet data from origin_packets_buffer_ when you need
// you can put decoded packet into decoded_packets_buffer_ , parser thread will convert it pointsxyzi info in variable frame_
// you can control lidar with variable ptc_client_
template <typename T_Point>
class Lidar
{
public:
  ~Lidar();
  Lidar();
  Lidar(const Lidar &orig) = delete;
  // init lidar with param. init logger, udp parser, source, ptc client, start receive/parser thread
  int Init(const DriverParam& param);
  int GetGeneralParser(GeneralParser<T_Point> **parser);
  // set lidar type for Lidar object and udp parser, udp parser will initialize the corresponding subclass
  int SetLidarType(std::string lidar_type);
  // get one packet from origin_packets_buffer_, 
  int GetOnePacket(UdpPacket &packet);
  // start to record pcap file with name record_path,the record source is udp data recieve from source
  int StartRecordPcap(std::string record_path);
  // stop record pcap
  int StopRecordPcap();
  // start to record pcap file with name record_path,the record source is the parameter UdpFrame_t
  // port is the udp port recorded in pcap
  int SaveUdpPacket(const std::string &record_path, const UdpFrame_t &packets,
                    int port = 2368);
  // start to record pcap file with name record_path,the record source is the parameter UdpFrameArray_t
  // port is the udp port recorded in pcap                   
  int SaveUdpPacket(const std::string &record_path,
                    const UdpFrameArray_t &packets, int port = 2368);
  /* this founction whill put a LidarDecodedPacket into decoded_packets_buffer_, and then the parser thread will 
  convert decoded packet dato into pointxyzi info*/
  int ComputeXYZI(LidarDecodedPacket<T_Point> &packet);
  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  int DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udp_packet);
  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udp_packet);
  // Determine whether all pointxyzi info is parsed in this frame
  bool ComputeXYZIComplete(int index);
  // save lidar correction file from ptc
  int SaveCorrectionFile(std::string correction_save_path);
  // get lidar correction file from ptc,and pass to udp parser
  int LoadCorrectionForUdpParser();
  // get lidar correction file from local file,and pass to udp parser 
  void LoadCorrectionFile(std::string correction_path); 
  int LoadCorrectionString(char *correction_string);
  // get lidar firetime correction file from local file,and pass to udp parser     
  void LoadFiretimesFile(std::string firetimes_path);
  void SetUdpParser(UdpParser<T_Point> *udp_parser);
  UdpParser<T_Point> *GetUdpParser();
  void EnableRecordPcap(bool bRecord);
  // set the parser thread number
  void SetThreadNum(int thread_num);
  void SetSource(Source **source);
  std::string GetLidarType();
  // get pcap status
  bool IsPlayEnded();
  // set standby mode
  int SetStandbyMode(PtcClient *Ptc_client, int standby_mode);
  // set spin speed
  int SetSpinSpeed(PtcClient *Ptc_client, int speed);
  // load correction file from Ros bag 
  int LoadCorrectionFromROSbag();
  UdpParser<T_Point> *udp_parser_;
  Source *source_;
  PtcClient *ptc_client_;
  LidarDecodedFrame<T_Point> frame_;
  BlockingRing<UdpPacket, kPacketBufferSize> origin_packets_buffer_;
  uint16_t use_timestamp_type_ = 0;
  int fov_start_ = 0;
  int fov_end_ = 0;
  u8Array_t correction_string_;

private:
  uint16_t ptc_port_;
  uint16_t udp_port_;
  std::string device_ip_address_;
  std::string host_ip_address_;
  // recieve udp data thread, this function will recieve udp data in while(),exit when variable running_ is false
  void RecieveUdpThread();
  /* parser decoded packet data thread. when thread num <  2, this function will parser decoded packet data in this thread
  when thread num >= 2,decoded packet data will be put in handle_thread_packet_buffer_, and the decoded packet will be parered in handle_thread*/
  void ParserThread();
  /* this function will parser decoded packet data  in handle_thread_packet_buffer_ into pointxyzi info  */
  void HandleThread(int thread_num);
  BlockingRing<LidarDecodedPacket<T_Point>, kPacketBufferSize> decoded_packets_buffer_;

  // this variable is the exit condition of udp/parser thread
  bool running_;
  // this variable decide whether recieve_packet_thread will run
  bool udp_thread_running_;
  // this variable decide whether parser_thread will run
  bool parser_thread_running_;
  std::thread *recieve_packet_thread_ptr_;
  std::thread *parser_thread_ptr_;
  std::mutex *mutex_list_;
  std::vector<std::list<LidarDecodedPacket<T_Point>>> handle_thread_packet_buffer_;
  std::vector<std::thread *> handle_thread_vec_;
  uint32_t handle_buffer_size_;
  int handle_thread_count_;
  bool is_record_pcap_;
  std::string lidar_type_;
  bool is_timeout_ = false;

};
}  // namespace lidar
}  // namespace hesai

#include "lidar.cc"
#endif /* Lidar_H */


