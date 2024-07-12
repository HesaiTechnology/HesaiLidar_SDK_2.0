#ifndef LIDAR_SKD_H
#define LIDAR_SKD_H
// #ifdef __cplusplus
// #define EXTERNC extern "C"
// #else
// #define EXTERNC
// #endif

//the NVidia lidar detection algorithm expects this many points
#define MAX_LIDAR_POINTS 204800 //280000

#define ALL_FRAMES -1

typedef struct{
  float x;
  float y;
  float z;
  float intensity;
} LidarPoint;


extern void initCloudAcquisition_2_0(void* lidarPtr, 
                          void(*callback) (void* lidarPtr, LidarPoint *point_cloud, size_t numPoints, int debug_frameIndex),
                          const std::string & debug_PCAP_file,
                          const std::string & debug_save_path,
                          int debug_maxFrames);
extern void startCloudAcquisition();
extern void stopCloudAcquisition();
extern void finalizeCloudAcquisition();

// #undef EXTERNC

#endif //LIDAR_SKD_H