
#include "pcl.pb.h"

#ifdef USE_LEGACY_POINTCLOUD
  #define POINTCLOUD_CLASS pcl::PointCloud2v08
#else
  #define POINTCLOUD_CLASS pcl::PointCloud2
#endif
