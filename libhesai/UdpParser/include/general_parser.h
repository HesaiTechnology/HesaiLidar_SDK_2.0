/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/

/*
 * File:       general_parser.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Declare GeneralParser class
*/

#ifndef GENERAL_PARSER_H_
#define GENERAL_PARSER_H_
#define CIRCLE (36000)
#define MAX_LASER_NUM (512)
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// #include "udp_parser.h"
#ifndef _MSC_VER
#include <semaphore.h>
#endif
#include <list>
#include <cmath>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include "lidar_types.h"
#include "plat_utils.h"
namespace hesai
{
namespace lidar
{

#define DEFINE_MEMBER_CHECKER(member)                                                                                  \
  template <typename T, typename V = bool>                                                                             \
  struct has_##member : std::false_type                                                                                \
  {                                                                                                                    \
  };                                                                                                                   \
  template <typename T>                                                                                                \
  struct has_##member<                                                                                                 \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type>          \
      : std::true_type                                                                                                 \
  {                                                                                                                    \
  };
#define PANDAR_HAS_MEMBER(C, member) has_##member<C>::value
DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)

template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
  point.x = value;
}

template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
  point.y = value;
}

template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
  point.z = value;
}

template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.intensity = value;
}

template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
  point.ring = value;
}

template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                      const double& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                     const double& value)
{
  point.timestamp = value;
}

inline float deg2Rad(float deg)
{
    return (float)(deg * 0.01745329251994329575);
}

inline float rad2Deg(float rad)
{
    return (float)(rad * 57.29577951308232087721);
}

struct Transform {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};

// class GeneralParser
// the GenneralParser class is a base class for parsering packets and computing points
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template <typename T_Point>
class GeneralParser {
 public:
  GeneralParser();
  virtual ~GeneralParser();

  // get lidar correction file from local file,and pass to udp parser 
  virtual void LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);

  // get lidar firetime correction file from local file,and pass to udp parser 
  virtual void LoadFiretimesFile(std::string firetimes_path);
  virtual int LoadFiretimesString(char *firetimes_string);

  // compute lidar firetime correciton
  virtual double GetFiretimesCorrection(int laserId, double speed);

  // compute lidar distance correction
  virtual void GetDistanceCorrection(double &azimuth, double &elevation, double &distance);
  void SetEnableFireTimeCorrection(bool enable);
  void SetEnableDistanceCorrection(bool enable);
  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  virtual int DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket); 

  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket); 
   
  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet);

  //set frame azimuth
  virtual void SetFrameAzimuth(float frame_start_azimuth);

  //set enable_packet_loss_tool_
  virtual void EnablePacketLossTool(bool enable);

  void TransformPoint(float& x, float& y, float& z);
  void SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw);
  void EnableUpdateMonitorInfo();
  void DisableUpdateMonitorInfo();
  uint16_t *GetMonitorInfo1();
  uint16_t *GetMonitorInfo2();
  uint16_t *GetMonitorInfo3();
  std::vector<double> elevation_correction_{0};
  std::vector<double> azimuth_collection_{0};
  uint32_t total_start_seqnum_;
  uint32_t total_loss_count_;
  uint32_t current_seqnum_;
  uint32_t total_packet_count_;
  
 protected:
  uint16_t monitor_info1_[256];
  uint16_t monitor_info2_[256];
  uint16_t monitor_info3_[256];
  float cos_all_angle_[CIRCLE];
  float sin_all_angle_[CIRCLE];
  bool enable_update_monitor_info_;
  int return_mode_;
  int motor_speed_;
  bool get_correction_file_;
  bool get_firetime_file_;
  bool is_dual_return_;
  uint16_t spin_speed_;
  static const uint16_t kAzimuthTolerance = 1000;
  bool use_angle_ = true;
  int32_t last_azimuth_;
  int32_t last_last_azimuth_;
  uint32_t start_seqnum_;
  uint32_t last_seqnum_;
  uint32_t loss_count_;
  uint32_t start_time_;
  double firetime_correction_[512];
  bool enable_firetime_correction_;
  bool enable_distance_correction_;
  bool enable_packet_loss_tool_;
  Transform transform_;
  float frame_start_azimuth_;
};
}
}
#include "general_parser.cc"

#endif  // GENERAL_PARSER_H_
