#include "hesai_lidar_sdk.hpp"

// <------- eCAL ---------->
#define eCAL_ON
#ifdef eCAL_ON
// eCAL
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
// proto i/o
#include "pcl.pb.h"
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "pointcloud/PointCloudHandler.h"

eCAL::protobuf::CPublisher<pcl::PointCloud2> publisher_pcl2;

void init_eCAL(int argc, char** argv) {
  // Initialize eCAL
  eCAL::Initialize(argc, argv, "Hesai AT128");
  // create publisher
///  publisher_pcl2 = eCAL::protobuf::CPublisher<pcl::PointCloud2>("MRL360");
  publisher_pcl2 = eCAL::protobuf::CPublisher<pcl::PointCloud2>("meta_pcl");
  printf("Ecal publisher created\n");
  // set eCAL state to healthy (--> eCAL Monitor)
  eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "AT128 eCAL publishers initialized");
}

void publish_pointloud2(const LidarDecodedFrame<LidarPointXYZICRT> &frame, eCAL::protobuf::CPublisher<pcl::PointCloud2>& publisher_pcl2) {
  // Create a protobuf message object
  pcl::PointCloud2 at128_pointcloud;
  std::vector<float> pts;
  for (uint32_t i = 0; i < frame.points_num; i++)
  {
    pts.push_back(frame.points[i].x);
    pts.push_back(frame.points[i].y);
    pts.push_back(frame.points[i].z);
    pts.push_back(frame.points[i].intensity);
    if (i < 1000) printf("%f\n",frame.points[i].x);
  }
//  printf("\n\n");
  // fill the protobuf message object
  setPointCloud(&at128_pointcloud, { "x","y","z","intensity" }, pts);
//  data->timestamp
//  mrl360_pointcloud.mutable_header()->mutable_stamp()->set_secs(p.secs);
//  mrl360_pointcloud.mutable_header()->mutable_stamp()->set_nsecs(p.nsecs);
  // Send the message
  publisher_pcl2.Send(at128_pointcloud);
};
#endif // eCAL_ON
// <------- eCAL ---------->


uint32_t last_frame_time = 0;
uint32_t cur_frame_time = 0;

//log info, display frame message
void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>  &frame) {  
  cur_frame_time = GetMicroTickCount();
  if (last_frame_time == 0) last_frame_time = GetMicroTickCount();
  if (cur_frame_time - last_frame_time > kMaxTimeInterval) {
    printf("Time between last frame and cur frame is: %u us\n", (cur_frame_time - last_frame_time));
  }
  last_frame_time = cur_frame_time;
  printf("frame:%d points:%u packet:%u start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;

  publish_pointloud2(frame, publisher_pcl2);
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

int main(int argc, char *argv[])
{
#ifndef _MSC_VER
  if (system("sudo sh -c \"echo 562144000 > /proc/sys/net/core/rmem_max\"") == -1) {
    printf("Command execution failed!\n");
  }
#endif
  HesaiLidarSdk<LidarPointXYZICRT> sample;
  DriverParam param;

  printf("Hesai AT128 --> eCAL\n");
#ifdef eCAL_ON
  init_eCAL(argc, argv);
#endif

// assign param
  // param.decoder_param.enable_packet_loss_tool = true;
  param.lidar_type = "";
  param.input_param.source_type = DATA_FROM_LIDAR;
  param.input_param.pcap_path = "Your pcap file path";
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";

  param.input_param.device_ip_address = "192.168.1.201";
  param.input_param.ptc_port = 9347;
  param.input_param.udp_port = 2368;
  param.input_param.rs485_com = "Your serial port name for receiving point cloud";
  param.input_param.rs232_com = "Your serial port name for sending cmd";
  param.input_param.host_ip_address = "";
  param.input_param.multicast_ip_address = "";
  param.decoder_param.distance_correction_lidar_flag = false;
  param.decoder_param.socket_buffer_size = 262144000;

  //init lidar with param
  sample.Init(param);

  //assign callback fuction
  sample.RegRecvCallback(lidarCallback);
  sample.RegRecvCallback(faultMessageCallback);

  sample.Start();

  // You can select the parameters in while():
  // 1.[IsPlayEnded(sample)]: adds the ability for the PCAP to automatically quit after playing the program
  // 2.[1                  ]: the application will not quit voluntarily
  while (!IsPlayEnded(sample) || GetMicroTickCount() - last_frame_time < 1000000)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  printf("The PCAP file has been parsed and we will exit the program.\n");

  eCAL::Finalize(); // finalize eCAL API

  return 0;
}