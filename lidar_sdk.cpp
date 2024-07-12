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
#include <climits>

#include "driver/hesai_lidar_sdk.hpp"
#include "lidar_sdk.h"

// #ifdef __cplusplus
// extern "C" {
// #endif
//NOTE: the following is based on the assumption there's ONE lidar
static HesaiLidarSdk<LidarPointXYZIRT>  * lidar_sdk;

static void             * lidar_ptr;
static void            (* lidar_callback) (void* lidarPtr, LidarPoint *point_cloud, size_t numPoints, int debug_FrameIndex);
static LidarPoint         buffer[MAX_LIDAR_POINTS];
static std::thread        lidar_sdk_acquisituionThread;
static bool               lidar_sdk_continueAcquisition = true;
static sem_t              firstFrameArrived;

// #define DEBUG_SDK

#ifdef DEBUG_SDK
static std::string debug_SavePath;
static int debug_MaxFiles;
static int debug_FrameIndex = 0;
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
#endif //DEBUG_SDK

void lidarAcquisitionCallback(const LidarDecodedFrame<LidarPointXYZIRT>  &frame) {
    #ifdef DEBUG_SDK
    std::string fileName = std::string(debug_SavePath) + "/PointCloudFrame" + std::to_string(debug_FrameIndex) + ".pcd";
    debug_FrameIndex = (debug_FrameIndex+1) % debug_MaxFiles;

    FILE * f_pcd = fopen(fileName.c_str(), "w");
    sdk_create_pcd_header(f_pcd, frame.points_num);
    #endif //DEBUG_SDK

    sem_post(&firstFrameArrived);

    memset(buffer, 0, sizeof(LidarPoint) * MAX_LIDAR_POINTS); //clear prev. frame
    LidarPoint* p = buffer;
    // int count=0;
    
    for (uint32_t i = 0 ; i<frame.points_num; i++)
    {
        const auto& point = frame.points[i];

        //the NVidia lidar detection algorithm expects this many points. 
        //TODO: cutting off the remainder is the most primitive way to do this.
        // Replace the following with somethig more sophisticated, e.g. cut most 
        // remote points from the center
        if( p - buffer >= MAX_LIDAR_POINTS ) 
        {
          break;
        }
        // if(count++ %100==0)
        //   printf("-- %d\n", count);
        p->x = point.x;
        p->y = point.y;
        p->z = point.z;
        p++ ->intensity = point.intensity;
        #ifdef DEBUG_SDK
        fprintf(f_pcd, "%5.3f %5.3f %5.3f %5.3f 0 0\n", point.x, point.y, point.z, point.intensity);
        #endif //DEBUG_SDK
    }

    int frameIndex = -1;

    #ifdef DEBUG_SDK
    frameIndex = debug_FrameIndex;
    fclose(f_pcd);
    #endif //DEBUG_SDK

    //synchronization is done inside this call, i.e. this thread is blocked until 
    // the buffer data is transferred, consistency of the of the data in the buffer
    // is guaranteed
    lidar_callback(lidar_ptr, buffer, frame.points_num, frameIndex);
}

// void lidarAlgorithmCallback(HS_Object3D_Object_List* object_t) {
//     HS_Object3D_Object* object;
// #ifdef PRINT_FLAG
//       printf("----------------------\n");
//       printf("total objects: %d\n",object_t->valid_size);
//       for (size_t i = 0; i < object_t->valid_size; i++) {
//           object = &object_t->data[i];
//           printf("id: %u, type: %u\n",object->data.id, object->type);
//       }
//       printf("----------------------\n");
// #endif
// }

void gpsCallback(double timestamp) {
#ifdef PRINT_FLAG
    printf("gps: %d\n", timestamp);
#endif    
}

void acquisituionFunction()
{
    lidar_sdk_continueAcquisition = true;
    while (lidar_sdk_continueAcquisition) {
        sleep(100);
    }
    lidar_sdk->Stop();
    delete(lidar_sdk);
}

void initCloudAcquisition_2_0(void* lidarPtr, 
                          void(*callback) (void* lidarPtr, LidarPoint *point_cloud, size_t numPoints, int debug_frameIndex),
                          const std::string & debug_PCAP_file,
                          const std::string & debug_save_path,
                          int debug_maxFrames)
{
    lidar_callback = callback;
    lidar_ptr      = lidarPtr; 
    #ifdef DEBUG_SDK
    debug_SavePath = debug_save_path;
    debug_MaxFiles = debug_max_frames == ALL_FRAMES? INT_MAX-100 : debug_max_frames;
    #endif //DEBUG_SDK
  
    lidar_sdk = new HesaiLidarSdk<LidarPointXYZIRT>();

    DriverParam params;
    if (!debug_PCAP_file.empty())
    {
        params.input_param.source_type = DATA_FROM_PCAP;
        params.input_param.pcap_path = debug_PCAP_file;
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

    sem_init(&firstFrameArrived, 0, 1);
    sem_wait(&firstFrameArrived);

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

// #ifdef __cplusplus
// }
// #endif