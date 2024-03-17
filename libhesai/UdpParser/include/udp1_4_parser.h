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
 * File:       udp1_4_parser.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Declare Udp1_4Parser class
*/

#ifndef UDP1_4_PARSER_H_
#define UDP1_4_PARSER_H_

#include "general_parser.h"
namespace hesai
{
namespace lidar
{

struct FiretimeSectionValues {
    struct SectionValue {
        std::array<int, 2> firetime;
    };
    std::array<SectionValue, 8> section_values;
};
// class Udp1_4Parser
// parsers packets and computes points for Pandar128
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template<typename T_Point>
class Udp1_4Parser : public GeneralParser<T_Point> {
 public:
  Udp1_4Parser();
  virtual ~Udp1_4Parser();

  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  virtual int DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket);

  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);

  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame      
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet);

  // get lidar firetime correction file from local file,and pass to udp parser 
  virtual void LoadFiretimesFile(std::string firetimes_path);

  using GeneralParser<T_Point>::GetFiretimesCorrection;
  // compute lidar firetime correciton
  double GetFiretimesCorrection(int laserId, double speed, uint8_t optMode, uint8_t angleState,uint16_t dist);
  
  // determine whether frame splitting is needed
  bool IsNeedFrameSplit(uint16_t azimuth); 

  using GeneralParser<T_Point>::GetDistanceCorrection;
  // compute lidar distance correction
  void GetDistanceCorrection(int laser_id, float distance, int& azimuth, int& elevation);
//   virtual int ComputeXYZI(LidarDecodedFrame &frame, LidarDecodedPacket &packet);

 private:
  static const int kLaserNum = 128;
  double section_distance;
  std::array<FiretimeSectionValues, kLaserNum> firetime_section_values;
  float distance_correction_para_a_;
  float distance_correction_para_b_; 
  float distance_correction_para_h_; 
  float distance_correction_para_c_; 
  float distance_correction_para_d_; 
  bool use_frame_start_azimuth_ = true;
};
}  // namespace lidar
}  // namespace hesai

#include "udp1_4_parser.cc"

#endif  // UDP1_4_PARSER_H_
