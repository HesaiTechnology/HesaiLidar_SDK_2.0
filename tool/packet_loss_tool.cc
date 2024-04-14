#include "hesai_lidar_sdk.hpp"

uint32_t last_frame_time;
uint32_t cur_frame_time;

//log info, display frame message
void lidarCallback(const LidarDecodedFrame<LidarPointXYZIRT>  &frame) {  
  cur_frame_time = GetMicroTickCount();
  if (cur_frame_time - last_frame_time > kMaxTimeInterval) {
    printf("Time between last frame and cur frame is: %d\n us", (cur_frame_time - last_frame_time));
  }
}

int main(int argc, char *argv[])
{
#ifndef _MSC_VER
  if (system("sudo sh -c \"echo 562144000 > /proc/sys/net/core/rmem_max\"") == -1) {
     printf("Command execution failed!\n");
  }
#endif
  HesaiLidarSdk<LidarPointXYZIRT> sample;
  DriverParam param;

  // assign param
  param.input_param.source_type = DATA_FROM_LIDAR;
  param.decoder_param.enable_packet_loss_tool = true;
  param.input_param.pcap_path = "Your pcap file path";
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";

  param.input_param.device_ip_address = "192.168.1.201";
  param.input_param.ptc_port = 9347;
  param.input_param.udp_port = 2368;
  param.input_param.host_ip_address = "192.168.1.100";
  param.input_param.multicast_ip_address = "";

  //init lidar with param
  sample.Init(param);
  float socket_buffer = 262144000;
  if (argc > 2) socket_buffer = atof(argv[2]);
  sample.lidar_ptr_->source_->SetSocketBufferSize(socket_buffer);

  //assign callback fuction
  sample.RegRecvCallback(lidarCallback);

  //star process thread
  last_frame_time = GetMicroTickCount();
  float run_time = 15;
  if (argc > 1 ) run_time = atof(argv[1]);


  sample.Start();

  uint64_t start = GetMicroTickCountU64();
  while (1)
  {
    uint64_t now = GetMicroTickCountU64();

    if (now <= start) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    //packet loss tool run time
    if (int(now -start) < int(run_time * kMicrosecondToSecondInt)) std::this_thread::sleep_for(std::chrono::microseconds(1000));
    else {
      break;
    }
  }

  uint64_t end = GetMicroTickCountU64();
  uint32_t total_packet_count = sample.lidar_ptr_->udp_parser_->GetGeneralParser()->total_packet_count_;
  uint32_t total_packet_loss_count = sample.lidar_ptr_->udp_parser_->GetGeneralParser()->total_loss_count_;
  printf("total recevice packet time: %lums\n",(end -start) / 1000);
  printf("total receviced packet count: %u\n",total_packet_count);
  printf("total loss packet count: %u\n",total_packet_loss_count);
  if (float(total_packet_loss_count) / float(total_packet_count) > 0.01) {
    printf("Error: Packet loss rate exceeds 1%%\n");
  }
  sample.Stop();

}