#include "hesai_lidar_sdk_gpu.cuh"
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

// #define SAVE_PCD_FILE
// #define ENABLE_VIEWER

// using namespace hesai::lidar;
struct PointXYZIT {
  //添加pcl里xyz
  PCL_ADD_POINT4D   
  float intensity;
  double timestamp;
  uint16_t ring;                   
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;                   

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))


using namespace pcl::visualization;
std::shared_ptr<PCLVisualizer> pcl_viewer;
uint32_t last_frame_time;
uint32_t cur_frame_time;std::mutex mex_viewer;


//log info, display frame message
void lidarCallback(const LidarDecodedFrame<PointXYZIT>  &frame) {
  cur_frame_time = GetTickCount();  
  if (cur_frame_time - last_frame_time > kMaxTimeInterval) {
    printf("Time between last frame and cur frame is: %d\n", (cur_frame_time - last_frame_time));
  }
  last_frame_time = cur_frame_time;    
  printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;
  pcl::PointCloud<PointXYZIT>::Ptr pcl_pointcloud(new pcl::PointCloud<PointXYZIT>);
  mex_viewer.lock();
  if (frame.points_num == 0) return;
  pcl_pointcloud->clear();
  pcl_pointcloud->resize(frame.points_num);
  pcl_pointcloud->points.assign(frame.points, frame.points + frame.points_num);
  pcl_pointcloud->height = 1;
  pcl_pointcloud->width = frame.points_num;
  pcl_pointcloud->is_dense = false;
  pcl::PCDWriter writer;
  std::string file_name = "./PointCloudFrame" + std::to_string(frame.frame_index) + ".pcd";

//save point cloud with pcd file if define SAVE_PCD_FILE
#ifdef SAVE_PCD_FILE
  writer.writeASCII(file_name, *pcl_pointcloud);
#endif  

//display point cloud with pcl if define ENABLE_VIEWER
#ifdef ENABLE_VIEWER   
  PointCloudColorHandlerGenericField<PointXYZIT> point_color_handle(pcl_pointcloud, "intensity");
  pcl_viewer->updatePointCloud<PointXYZIT>(pcl_pointcloud, point_color_handle, "pandar");
#endif
mex_viewer.unlock();
}

//display point cloud with pcl if define ENABLE_VIEWER
void PclViewerInit(std::shared_ptr<PCLVisualizer> pcl_viewer) {
  pcl_viewer = std::make_shared<PCLVisualizer>("HesaiPointCloudViewer");
  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  pcl_viewer->addCoordinateSystem(1.0);
  pcl::PointCloud<PointXYZIT>::Ptr pcl_pointcloud(new pcl::PointCloud<PointXYZIT>);
  pcl_viewer->addPointCloud<PointXYZIT>(pcl_pointcloud, "pandar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "pandar");
  return;
}

int main(int argc, char *argv[])
{
  #ifdef ENABLE_VIEWER   
  PclViewerInit(pcl_viewer);
  #endif 
  HesaiLidarSdkGpu<PointXYZIT> sample;
  DriverParam param;
  // assign param
  param.decoder_param.enable_parser_thread = false;
  param.input_param.source_type = DATA_FROM_LIDAR;
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

  //assign callback fuction
  sample.RegRecvCallback(lidarCallback);

  //star process thread
  sample.Start();
  while (1)
  {

#ifdef ENABLE_VIEWER   
    mex_viewer.lock();
    if(pcl_viewer->wasStopped()) break;
    pcl_viewer->spinOnce();
    mex_viewer.unlock();
#endif     
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

  }
}