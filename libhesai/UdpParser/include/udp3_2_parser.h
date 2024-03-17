
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
 * File:       udp3_2_parser.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Declare Udp3_2Parser class
*/

#ifndef UDP3_2_PARSER_H_
#define UDP3_2_PARSER_H_
#define HS_LIDAR_QT128_LASER_NUM (128)
#define HS_LIDAR_QT128_LOOP_NUM (4)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG (0.0354)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT (-0.0072)
#include <fstream>
#include "general_parser.h"
#include "udp_protocol_v3_2.h"
namespace hesai
{
namespace lidar
{

struct PandarQTChannelConfig {
 public:
  uint16_t sob;
  uint8_t major_version;
  uint8_t min_version;
  uint8_t laser_num;
  uint8_t m_u8BlockNum;
  std::vector<std::vector<int>> m_vChannelConfigTable;
  std::string m_sHashValue;
  bool m_bIsChannelConfigObtained;
};
// class Udp3_2Parser
// parsers packets and computes points for PandarQT128
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template<typename T_Point>
class Udp3_2Parser : public GeneralParser<T_Point> {
 public:
  Udp3_2Parser();
  virtual ~Udp3_2Parser();

  // get lidar firetime correction file from local file,and pass to udp parser 
  virtual int LoadFiretimesString(char *firetimes_string);
  virtual void LoadFiretimesFile(std::string firetimes_path);

  using GeneralParser<T_Point>::GetFiretimesCorrection;
  // compute lidar distance correction
  double GetFiretimesCorrection(int laserId, double speed, int loopIndex);

  // compute lidar distance correction
  void GetDistanceCorrection(double &azimuth, double &elevation, double &distance);

  // get lidar correction file from local file,and pass to udp parser 
  int LoadChannelConfigString(char *channel_config_content);
  void LoadChannelConfigFile(std::string channel_config_path);
  // virtual int ComputeXYZI(LidarDecodedFrame &frame, LidarDecodedPacket &packet);

  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  virtual int DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket);    

  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);

  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet); 
  
  // determine whether frame splitting is needed
  bool IsNeedFrameSplit(uint16_t azimuth);               

 private:
  std::array<std::array<float, HS_LIDAR_QT128_LASER_NUM>,
             HS_LIDAR_QT128_LOOP_NUM>
      qt128_firetime_;
  PandarQTChannelConfig pandarQT_channel_config_;
};
}  // namespace lidar
}  // namespace hesai

#include "udp3_2_parser.cc"

#endif  // UDP3_2_PARSER_H_
