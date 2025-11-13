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
#include <string>
#include "logger.h"  
#include "hs_com.h"
namespace hesai
{
namespace lidar
{

#define NULL_TOPIC  "your topic name"

enum SourceType
{
  DATA_FROM_LIDAR = 1,
  DATA_FROM_PCAP = 2,
  DATA_FROM_ROS_PACKET = 3,
  DATA_FROM_SERIAL = 4,
  DATA_FROM_LIDAR_TCP = 5,
};

enum PtcMode
{
  tcp = 0, 
  tcp_ssl = 1
};

enum UseTimestampType
{
  point_cloud_timestamp = 0,
  sdk_recv_timestamp = 1,
};

///< LiDAR decoder parameter
typedef struct DecoderParam  
{
  ///< Used to transform points
  TransformParam transform_param;    
  int thread_num = 1;
  bool enable_udp_thread = true;
  bool enable_parser_thread = true;
  bool pcap_play_synchronization = true;
  float play_rate_ = 1.0;
  bool pcap_play_in_loop = false;
  //start a new frame when lidar azimuth greater than frame_start_azimuth
  //range:[0-360), set frame_start_azimuth less than 0 if you do want to use it
  float frame_start_azimuth = 0;
  // enable the udp packet loss detection tool
  // it forbiddens parser udp packet while trun on this tool
  bool enable_packet_loss_tool = false;
  bool enable_packet_timeloss_tool = false;
  bool packet_timeloss_tool_continue = false;
  // 0 use point cloud timestamp
  // 1 use sdk receive timestamp
  uint16_t use_timestamp_type = point_cloud_timestamp;
  int fov_start = -1;
  int fov_end = -1;
  bool distance_correction_flag = false;
  bool xt_spot_correction = false;
  bool et_blooming_filter_flag = false;
  RemakeConfig remake_config;
  uint32_t socket_buffer_size = 0;
  std::string channel_fov_filter_path = "";  // correction/config/channel_fov_filter.txt
  std::string multi_fov_filter_ranges = "";  // multiple fov filter ranges, for all channels
  float frame_frequency = 0;
  float default_frame_frequency = DEFAULT_MAX_MULTI_FRAME_NUM;
  bool update_function_safety_flag = false;
  uint16_t echo_mode_filter = 0;
} DecoderParam;

///< The LiDAR input parameter
typedef struct InputParam  
{
  // PTC mode
  PtcMode ptc_mode = PtcMode::tcp;
  SourceType source_type = DATA_FROM_PCAP;
  // Ip of Lidar
  std::string device_ip_address = "";   
  // tcp port
  uint16_t device_tcp_src_port = 5120;
  ///< Address of multicast
  std::string multicast_ip_address = "";  
  ///< Address of host
  std::string host_ip_address = ""; 
  ///< port filter
  uint16_t device_udp_src_port = 0;
  uint16_t device_fault_port = 0;
  // udp packet destination port number       
  uint16_t udp_port = 2368;   
  uint16_t fault_message_port = 0;
  ///< ptc packet port number     
  bool use_ptc_connected = true;         
  uint16_t host_ptc_port = 0;  // 0 is not used
  uint16_t ptc_port = 9347;
  ///< serial port and baudrate
  std::string rs485_com = "/dev/ttyUSB0";
  std::string rs232_com = "/dev/ttyUSB1";
  int point_cloud_baudrate = 3125000;
  int rs485_baudrate = 115200;   
  int rs232_baudrate = 9600;          
  std::string pcap_path = "Your pcap file path";  ///< Absolute path of pcap file
  std::string correction_file_path = "Your correction file path";   ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string firetimes_path = "Your firetime file path";  ///< Path of firetime files(angle.csv).
  std::string dcf_file_path = "Your dcf file path";
  std::string correction_save_path = "";
  /// certFile          Represents the path of the user's certificate
  std::string certFile = "";
  /// privateKeyFile    Represents the path of the user's private key
  std::string privateKeyFile = "";
  /// caFile            Represents the path of the root certificate
  std::string caFile = "";
  /// standby_mode    set the standby_mode of lidar
  int standby_mode = -1;
  /// speed             set the rotational speed of lidar
  int speed = -1;
  // timeout
  float recv_point_cloud_timeout = -1; //(s), <0 : not timeout 
  float ptc_connect_timeout = -1; //(s), <0 : not timeout 


  bool send_packet_ros;
  bool send_point_cloud_ros;
  bool send_imu_ros;
  std::string frame_id;

  std::string ros_send_packet_topic = NULL_TOPIC;
  std::string ros_send_point_topic = NULL_TOPIC;
  std::string ros_send_packet_loss_topic = NULL_TOPIC; 
  std::string ros_send_ptp_topic = NULL_TOPIC;
  std::string ros_send_correction_topic = NULL_TOPIC;
  std::string ros_send_firetime_topic = NULL_TOPIC;
  std::string ros_send_imu_topic = NULL_TOPIC;

  std::string ros_recv_correction_topic = NULL_TOPIC;
  std::string ros_recv_packet_topic = NULL_TOPIC;


} InputParam;

///< The LiDAR driver parameter
typedef struct DriverParam  
{
  ///< Input parameter
  InputParam input_param;  
  ///< Decoder parameter        
  DecoderParam decoder_param;  
  ///< The frame id of LiDAR message    
  std::string frame_id = "hesai";  
  bool use_gpu = false;
  uint8_t log_level = HESAI_LOG_INFO | HESAI_LOG_WARNING | HESAI_LOG_ERROR | HESAI_LOG_FATAL; //
  uint8_t log_Target = HESAI_LOG_TARGET_CONSOLE;
  std::string log_path = "./log.log";
} DriverParam;
}  // namespace lidar
}  // namespace hesai