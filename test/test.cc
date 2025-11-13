#include "hesai_lidar_sdk.hpp"

// #define LIDAR_PARSER_TEST
// #define SERIAL_PARSER_TEST
#define PCAP_PARSER_TEST
// #define EXTERNAL_INPUT_PARSER_TEST
// #define LIDAR_PARSER_TCP_TEST

uint32_t last_frame_time = 0;
uint32_t cur_frame_time = 0;
bool running = true;

//log info, display frame message
void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>  &frame) {  
  cur_frame_time = GetMicroTickCount();
  if (last_frame_time == 0) last_frame_time = GetMicroTickCount();
  uint32_t diff = (frame.fParam.IsMultiFrameFrequency() == 0) ?  kMaxTimeInterval : kMaxTimeInterval * frame.multi_rate;
  if (cur_frame_time - last_frame_time > diff) {
    printf("Time between last frame and cur frame is: %u us\n", (cur_frame_time - last_frame_time));
  }
  last_frame_time = cur_frame_time;
  if (frame.fParam.IsMultiFrameFrequency() == 0) {
    printf("%ld -> frame:%d points:%u packet:%u start time:%lf end time:%lf\n", GetMicroTimeU64(), frame.frame_index, frame.points_num, frame.packet_num, frame.frame_start_timestamp, frame.frame_end_timestamp);
  } else {
    printf("%ld -> frame:%d points:%u packet:%u start time:%lf end time:%lf\n", GetMicroTimeU64(), frame.multi_frame_index, frame.multi_points_num, frame.multi_packet_num, frame.multi_frame_start_timestamp, frame.multi_frame_end_timestamp);
  }
}

void faultMessageCallback(const FaultMessageInfo& fault_message_info) {
  // Use fault message messages to make some judgments
  // fault_message_info.Print();
  return;
}

// Determines whether the PCAP is finished playing
bool IsPlayEnded(HesaiLidarSdk<LidarPointXYZICRT>& sdk)
{
  return sdk.lidar_ptr_->IsPlayEnded();
}

void packetProducer(HesaiLidarSdk<LidarPointXYZICRT>& sample, const UdpFrame_t& udp_packet_frame) {
  size_t index = 0;
  while (running && index < udp_packet_frame.size()) {
    if (sample.OriginPacketIsBufferFull()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    sample.PushOriginPacketBuffer(udp_packet_frame[index].buffer, udp_packet_frame[index].packet_len, udp_packet_frame[index].recv_timestamp);
    index++;
  }
  running = false;
}

int main(int argc, char *argv[])
{
#ifndef _MSC_VER
  if (system("sudo sh -c \"echo 562144000 > /proc/sys/net/core/rmem_max\"") == -1) {
    printf("Command execution failed!\n");
  }
#endif
  HesaiLidarSdk<LidarPointXYZICRT> sample;
  DriverParam param;

  param.use_gpu = (argc > 1);
  // assign param
#ifdef LIDAR_PARSER_TEST
  param.input_param.source_type = DATA_FROM_LIDAR;
  param.input_param.device_ip_address = "192.168.1.201";  // lidar ip
  param.input_param.ptc_port = 9347; // lidar ptc port
  param.input_param.udp_port = 2368; // point cloud destination port
  param.input_param.multicast_ip_address = "";

  param.input_param.ptc_mode = PtcMode::tcp;
  param.input_param.use_ptc_connected = true;  // true: use PTC connected, false: recv correction from local file
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";

  param.input_param.host_ip_address = ""; // point cloud destination ip, local ip
  param.input_param.fault_message_port = 0; // fault message destination port, 0: not use
  // PtcMode::tcp_ssl use
  param.input_param.certFile = "";
  param.input_param.privateKeyFile = "";
  param.input_param.caFile = "";


#elif defined (SERIAL_PARSER_TEST)
  param.input_param.source_type = DATA_FROM_SERIAL;
  param.input_param.rs485_com = "Your serial port name for receiving point cloud";
  param.input_param.rs232_com = "Your serial port name for sending cmd";
  param.input_param.point_cloud_baudrate = 3125000;
  param.input_param.correction_save_path = "";
  param.input_param.correction_file_path = "Your correction file path";
  
#elif defined (PCAP_PARSER_TEST)
  param.input_param.source_type = DATA_FROM_PCAP;
  param.input_param.pcap_path = "Your pcap file path";
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";


  param.decoder_param.pcap_play_synchronization = true;
  param.decoder_param.play_rate_ = 1.0;
  param.decoder_param.pcap_play_in_loop = false; // pcap palyback

#elif defined (EXTERNAL_INPUT_PARSER_TEST)
  param.input_param.source_type = DATA_FROM_ROS_PACKET;
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";

#elif defined (LIDAR_PARSER_TCP_TEST)
  param.input_param.source_type = DATA_FROM_LIDAR_TCP;
  param.input_param.device_ip_address = "192.168.1.201";  // lidar ip
  param.input_param.device_tcp_src_port = 5121; // lidar pointcloud tcp port
  param.input_param.ptc_port = 9347; // lidar ptc port
  param.input_param.use_ptc_connected = true;  // true: use PTC connected, false: recv correction from local file
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";
#endif

  param.decoder_param.enable_packet_loss_tool = false;
  param.decoder_param.socket_buffer_size = 262144000;
  //init lidar with param
  sample.Init(param);

#ifdef EXTERNAL_INPUT_PARSER_TEST
  UdpFrame_t udp_packet_frame;  // Please implement the data import yourself.
  std::thread producer_thread(packetProducer, std::ref(sample), udp_packet_frame);
#endif

  //assign callback fuction
  sample.RegRecvCallback(lidarCallback);
  sample.RegRecvCallback(faultMessageCallback);

  sample.Start();
  if (sample.lidar_ptr_->GetInitFinish(FailInit)) {
#ifdef EXTERNAL_INPUT_PARSER_TEST
    running = false;
    if (producer_thread.joinable()) {
        producer_thread.join();
    }
#endif
    sample.Stop();
    return -1;
  }

  // You can select the parameters in while():
  // 1.[IsPlayEnded(sample)]: adds the ability for the PCAP to automatically quit after playing the program
  //   [GetMicroTickCount() - last_frame_time < 1000000]: Ensure that the point cloud is fully resolved
  // 2.[1                  ]: the application will not quit voluntarily
  while (!IsPlayEnded(sample) || GetMicroTickCount() - last_frame_time < 1000000)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
#ifdef EXTERNAL_INPUT_PARSER_TEST
    if (running == false) break;
#endif
  }

#ifdef EXTERNAL_INPUT_PARSER_TEST
  if (producer_thread.joinable()) {
      producer_thread.join();
  }
#endif
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  printf("The PCAP file has been parsed and we will exit the program.\n");
  return 0;
}