#include "hesai_lidar_sdk_gpu.cuh"
uint32_t last_frame_time;
uint32_t cur_frame_time;std::mutex mex_viewer;


//log info, display frame message
void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>  &frame) {
  cur_frame_time = GetMicroTickCount();  
  if (last_frame_time == 0) last_frame_time = GetMicroTickCount();
  if (cur_frame_time - last_frame_time > kMaxTimeInterval) {
    printf("Time between last frame and cur frame is: %d us\n", (cur_frame_time - last_frame_time));
  }
  last_frame_time = cur_frame_time;    
  printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;
}

void faultMessageCallback(const FaultMessageInfo& fault_message_info) {
  // Use fault message messages to make some judgments
  // fault_message_info.ATX_Print();
  return;
}

// Determines whether the PCAP is finished playing
bool IsPlayEnded(HesaiLidarSdkGpu<LidarPointXYZICRT>& sdk)
{
  return sdk.lidar_ptr_->IsPlayEnded();
}

int main(int argc, char *argv[])
{
  HesaiLidarSdkGpu<LidarPointXYZICRT> sample;
  DriverParam param;
  // assign param
  param.lidar_type = "";
  param.input_param.source_type = DATA_FROM_LIDAR;
  param.input_param.pcap_path = "Your pcap file path";
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";

  param.input_param.device_ip_address = "192.168.1.201";
  param.input_param.ptc_port = 9347;
  param.input_param.udp_port = 2368;
  param.input_param.host_ip_address = "";
  param.input_param.multicast_ip_address = "";
  param.decoder_param.distance_correction_lidar_flag = false;
  param.decoder_param.socket_buffer_size = 262144000;


  //init lidar with param
  sample.Init(param);

  //assign callback fuction
  sample.RegRecvCallback(lidarCallback);
  sample.RegRecvCallback(faultMessageCallback);

  //star process thread
  sample.Start();
  while (!IsPlayEnded(sample) || GetMicroTickCount() - last_frame_time < 100000)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  printf("The PCAP file has been parsed and we will exit the program.\n");
  return 0;
}