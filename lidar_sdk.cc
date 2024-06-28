/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <stdio.h>
#include <thread>

#include "driver/hesai_lidar_sdk.hpp"
#include "lidar_sdk.h"
#include <mutex>
#include <condition_variable>
// #include <semaphore>

#ifdef __cplusplus
extern "C" {
#endif

//NOTE: it is ASSUMED that there's ONE lidar
static HesaiLidarSdk<LidarPointXYZIRT>  * lidar_sdk;
static void             * lidar_ptr;
static void            (* lidar_callback) (void* lidarPtr, LidarPoint *point_cloud, int numPoints, int frameIndex);
static LidarPoint         buffer[MAX_LIDAR_POINTS];
static std::thread        lidar_sdk_acquisituionThread;
static bool               lidar_sdk_continueAcquisition = true;
static std::mutex              firstFrameMutex;
static std::condition_variable firstFrameArrived;
static int                     frameIndex=0;
// std::binary_semaphore     firstFrameArrived{0};

#ifdef DEBUG_PLUGIN
static void
sdk_create_pcd_header(FILE *f_pcd, size_t size)
{
  fprintf(f_pcd, "# .PCD v0.7 - Point Cloud Data file format\n" \
                 "VERSION 0.7\n" 
                 "FIELDS x y z intensity timestamp ring\n"   //
                 "SIZE 4 4 4 4 8 2\n" //
                 "TYPE F F F F F U\n" //
                 "COUNT 1 1 1 1 1 1\n" //
                 "WIDTH %ld\n" 
                 "HEIGHT 1\n" 
                 "VIEWPOINT 0 0 0 1 0 0 0\n" //
                 "POINTS %ld\n" 
                 "DATA ascii\n", size, size);
}
#endif //DEBUG_PLUGIN

std::chrono::high_resolution_clock::time_point prevTime;

void lidarAcquisitionCallback(const LidarDecodedFrame<LidarPointXYZIRT>  &frame) {
    frameIndex = 0;
    firstFrameArrived.notify_one();
    #ifdef DEBUG_PLUGIN
    // printf("point_size: %ld\n", cloud->size());
    std::string fileName = "PointCloudFrame" + std::to_string(frameIndex) + ".pcd";
    FILE *    f_pcd = fopen(fileName.c_str(), "w");
    sdk_create_pcd_header(f_pcd, cloud->size());
    #endif //DEBUG_PLUGIN

    auto now = std::chrono::system_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count();
    prevTime = now;

    printf("* skd frame %d ms: %ld\n", frame.frame_index, (long)milliseconds); //cloud->size()
    
    LidarPoint* p = buffer;
    for (uint32_t i = 0 ; i<frame.points_num; i++)
    {
        const auto& point = frame.points[i];
        if( p - buffer >= MAX_LIDAR_POINTS )
        {
          printf("ERROR: lidarAcquisitionCallback. numPoints >= MAX_LIDAR_POINTS. Please contact the Dev. team\n");
          break;
        }
        // if(count++ %100==0)
        //   printf("-- %d\n", count);
        p->x = point.x;
        p->y = point.y;
        p->z = point.z;
        p++ ->intensity = point.intensity;
        #ifdef DEBUG_PLUGIN
        fprintf(f_pcd, "%5.3f %5.3f %5.3f %5.3f 0 0\n", point.x, point.y, point.z, point.intensity);
        #endif //DEBUG_PLUGIN
    }

    #ifdef DEBUG_PLUGIN
    fclose(f_pcd);
    #endif //DEBUG_PLUGIN

    //synchronization is done inside this call, i.e. this thread is blocked until 
    // the buffer data is transferred, consistency of the of the data in the buffer
    // is guaranteed
    lidar_callback(lidar_ptr, buffer, p - buffer, frame.frame_index);
}

void acquisituionFunction()
{
    lidar_sdk_continueAcquisition = true;
    while (lidar_sdk_continueAcquisition) {
        sleep(100);
    }
    lidar_sdk->Stop();
    delete(lidar_sdk);
    // return NULL;
}

void initCloudAcquisition(void* lidarPtr, 
                          void(*callback) (void* lidarPtr, LidarPoint *point_cloud, int numPoints, int frameIndex), 
                          int debug_FromPCAP)
{
  lidar_callback = callback;
  lidar_ptr      = lidarPtr; 
    
  lidar_sdk = new HesaiLidarSdk<LidarPointXYZIRT>();
  
  DriverParam params;
  if (debug_FromPCAP)
  {
    params.input_param.source_type = DATA_FROM_PCAP;
    params.input_param.pcap_path = "/media/ubuntu/myDev/my/XT32M2X-road_test.pcap";
  }
  else
  {
    params.input_param.source_type = DATA_FROM_LIDAR;
//   params.input_param.correction_file_path = "Your correction file path";
//   params.input_param.firetimes_path = "Your firetime file path";
    params.input_param.device_ip_address = "192.168.1.201";
    params.input_param.ptc_port = 9347;
    params.input_param.udp_port = 2368;
    params.input_param.host_ip_address = "";
    params.input_param.multicast_ip_address = "";
  }
  //init lidar with params
  lidar_sdk->Init(params);
  float socket_buffer = 262144000;
  lidar_sdk->lidar_ptr_->source_->SetSocketBufferSize(socket_buffer);

  //assign callback fuction
  lidar_sdk->RegRecvCallback(lidarAcquisitionCallback);
  lidar_sdk->Start();

  std::unique_lock<std::mutex> lock(firstFrameMutex);
  firstFrameArrived.wait(lock, [] { return frameIndex > -1; });

  printf("=== initCloudAcquisition continued");
}

void startCloudAcquisition()
{
  lidar_sdk_acquisituionThread = std::thread(acquisituionFunction);
}

void stopCloudAcquisition()
{
    lidar_sdk_continueAcquisition = false;
}

void finalizeCloudAcquisition()
{
    delete lidar_sdk;
}

#ifdef __cplusplus
}
#endif