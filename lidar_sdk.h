#ifndef LIDAR_SKD_H
#define LIDAR_SKD_H
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#define MAX_LIDAR_POINTS 250000

typedef struct{
  float x;
  float y;
  float z;
  float intensity;
} LidarPoint;

EXTERNC void initCloudAcquisition(void* lidarPtr, 
                          void(*callback) (void* lidarPtr, LidarPoint *point_cloud, int numPoints, int frameIndex), 
                          int debug_FromPCAP);
EXTERNC void startCloudAcquisition();
EXTERNC void stopCloudAcquisition();
EXTERNC void finalizeCloudAcquisition();

#undef EXTERNC

#endif //LIDAR_SKD_H