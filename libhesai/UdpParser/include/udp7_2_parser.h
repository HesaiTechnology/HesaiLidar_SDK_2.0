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
 * File:       udp7_2_parser.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Declare Udp7_2Parser class
*/

#ifndef UDP7_2_PARSER_H_
#define UDP7_2_PARSER_H_

#include "general_parser.h"
#include "lidar_types.h"
namespace hesai
{
namespace lidar
{

constexpr int CHANNEL_MAX = 256;
constexpr int COLUMN_MAX = 384;
constexpr int HASH_BYTES_LENGTH = 64;

struct PandarFTCorrectionsHeader {
    uint8_t pilot[2];
    uint8_t version[2];
    uint8_t reversed[2];
    uint8_t column_number;
    uint8_t channel_number;
    uint8_t resolution;
    PandarFTCorrectionsHeader() 
    : resolution(1)
    {}
};
struct PandarFTCorrections {
public:
    using ColumnFloatArray = std::array<float, COLUMN_MAX>;
    using CorrectionMatrix = std::array<ColumnFloatArray, CHANNEL_MAX>;
public:
    std::array<ColumnFloatArray, CHANNEL_MAX> elevations, azimuths;
    uint8_t major_version;
    uint8_t min_version;
    std::string hash_value;
};
// class Udp7_2Parser
// parsers packets and computes points for PandarFT120
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template<typename T_Point>
class Udp7_2Parser : public GeneralParser<T_Point> {
 public:
  Udp7_2Parser();
  virtual ~Udp7_2Parser();

  // get lidar correction file from local file,and pass to udp parser 
  virtual void LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  int LoadCorrectionDatData(char *correction_string);
  int LoadCorrectionCsvData(char *correction_string);

  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  virtual int DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket);  

  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);

  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame         
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet);

  // determine whether frame splitting is needed
  bool IsNeedFrameSplit(uint16_t column_id, uint16_t total_column);    

 private:
 int last_cloumn_id_;
 PandarFTCorrections corrections_;
};
}  // namespace lidar
}  // namespace hesai

#include "udp7_2_parser.cc"

#endif  // UDP7_2_PARSER_H_
