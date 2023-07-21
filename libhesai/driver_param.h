

#pragma once
#include <string>
#include "logger.h"  
namespace hesai
{
namespace lidar
{
enum SourceType
{
  DATA_FROM_LIDAR = 1,
  DATA_FROM_PCAP = 2,
  DATA_FROM_ROS_PACKET = 3,
};

///< The Point transform parameter
typedef struct TransformParam  
{
  ///< unit, m
  float x = 0.0f; 
  ///< unit, m     
  float y = 0.0f;
  ///< unit, m      
  float z = 0.0f;  
  ///< unit, radian    
  float roll = 0.0f;  
  ///< unit, radian 
  float pitch = 0.0f;  
  ///< unit, radian
  float yaw = 0.0f;    
} TransformParam;

///< LiDAR decoder parameter
typedef struct DecoderParam  
{
  // float max_distance = 200.0f;                                       ///< Max distance of point cloud range
  // float min_distance = 0.2f;                                         ///< Minimum distance of point cloud range
  // float start_angle = 0.0f;                                          ///< Start angle of point cloud
  // float end_angle = 360.0f;                                          ///< End angle of point cloud
  
  ///< Used to transform points
  TransformParam transform_param;    
  int thread_num = 1;
  bool enable_udp_thread = true;
  bool enable_parser_thread = true;
  bool pcap_play_synchronization = true;
  //start a new frame when lidar azimuth greater than frame_start_azimuth
  //range:1-359, set frame_start_azimuth less than 0 if you do want to use it
  float frame_start_azimuth = 359;
} DecoderParam;

///< The LiDAR input parameter
typedef struct InputParam  
{
  SourceType source_type = DATA_FROM_PCAP;
   ///< Ip of LiDAR
  std::string device_ip_address = "Your lidar ip";   
  ///< Address of multicast
  std::string multicast_ip_address = "";  
  ///< Address of host
  std::string host_ip_address = "Your host ip"; 
  ///< udp packet port number       
  uint16_t udp_port = 2368;   
  ///< ptc packet port number                
  uint16_t ptc_port = 9347;                 
  bool read_pcap = true;          ///< true: The driver will process the pcap through pcap_path. false: The driver will
                                   ///< Get data from online LiDAR
  std::string pcap_path = "Your pcap file path";  ///< Absolute path of pcap file
  std::string correction_file_path = "Your correction file path";   ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string firetimes_path = "Your firetime file path";  ///< Path of firetime files(angle.csv).
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
  ///< Lidar type
  std::string lidar_type = "AT128";  
  int log_level = LOG_DEBUG | LOG_INFO; //
  int log_Target = LOG_TARGET_CONSOLE | LOG_TARGET_FILE;
  std::string log_path = "./log.log";
} DriverParam;
}  // namespace lidar
}  // namespace hesai