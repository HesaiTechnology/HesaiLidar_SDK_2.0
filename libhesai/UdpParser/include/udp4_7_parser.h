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

#ifndef UDP4_7_PARSER_H_
#define UDP4_7_PARSER_H_

#define ATX_LASER_NUM (512)
#define ATX_AZIMUTHS_EVEN    (38452)
#define ATX_AZIMUTHS_ODD     (7631) 
#define ATX_ACCEPTANCE_ANGLE (200)

#include "general_parser.h"
#include "lidar_types.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
namespace hesai
{
namespace lidar
{
#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

struct ATXFiretimesHeader {
  uint8_t delimiter[2];
  uint8_t version[2];
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t channel_number;
  uint16_t angle_division;
} PACKED;

struct ATXFiretimes {
  ATXFiretimesHeader header;
  uint16_t raw_even_firetime_correction_[ATX_LASER_NUM];
  uint16_t raw_odd_firetime_correction_[ATX_LASER_NUM];
  double even_firetime_correction_[ATX_LASER_NUM];
  double odd_firetime_correction_[ATX_LASER_NUM];
  uint8_t SHA_value[32];
};

struct ATXCorrectionsHeader {
  uint8_t delimiter[2];
  uint8_t version[2];
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t channel_number;
  uint16_t angle_division;
} PACKED;

struct ATXCorrections {
  ATXCorrectionsHeader header;
  int16_t raw_azimuths[ATX_LASER_NUM];
  int16_t raw_azimuths_even[ATX_LASER_NUM];
  int16_t raw_azimuths_odd[ATX_LASER_NUM];
  int16_t raw_elevations[ATX_LASER_NUM];
  int16_t raw_elevations_adjust[ATX_LASER_NUM];
  float azimuth[ATX_LASER_NUM];
  float azimuth_even[ATX_LASER_NUM];
  float azimuth_odd[ATX_LASER_NUM];
  float elevation[ATX_LASER_NUM];
  float elevation_adjust[ATX_LASER_NUM];
  uint8_t SHA_value[32];
  static constexpr float kBegElevationAdjust = 20.0;
  static constexpr float kStepElevationAdjust = 2.0;
  static constexpr uint32_t kLenElevationAdjust = 70;
  ATXCorrections() : header()
  {
    memset(raw_azimuths, 0, sizeof(raw_azimuths));
    memset(raw_azimuths_even, 0, sizeof(raw_azimuths_even));
    memset(raw_azimuths_odd, 0, sizeof(raw_azimuths_odd));
    memset(raw_elevations, 0, sizeof(raw_elevations));
    memset(raw_elevations_adjust, 0, sizeof(raw_elevations_adjust));
    memset(azimuth, 0, sizeof(azimuth));
    memset(azimuth_even, 0, sizeof(azimuth_even));
    memset(azimuth_odd, 0, sizeof(azimuth_odd));
    memset(elevation, 0, sizeof(elevation));
    memset(elevation_adjust, 0, sizeof(elevation_adjust));
    memset(SHA_value, 0, sizeof(SHA_value));
  }
  double GetEndElevationAdjust() {return kBegElevationAdjust + kStepElevationAdjust * double(kLenElevationAdjust - 1); }
  float ElevationAdjust(int azi)
  {
    azi = (CIRCLE + azi) % CIRCLE;
    float azimuth_angle = (float)azi / kAllFineResolutionFloat;
    if (azimuth_angle < kBegElevationAdjust || azimuth_angle > GetEndElevationAdjust())
    {
      return 0;
    }
    int index = int((azimuth_angle - kBegElevationAdjust) / kStepElevationAdjust);
    if (index == kLenElevationAdjust - 1){
      return elevation_adjust[index];
    }
    float left_percent = (azimuth_angle - kBegElevationAdjust - index * kStepElevationAdjust) / kStepElevationAdjust;
    return elevation_adjust[index] * (1 - left_percent) + elevation_adjust[index + 1] * left_percent;
  }
};

// class Udp4_7Parser
// parsers packets and computes points for ATX
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template<typename T_Point>
class Udp4_7Parser : public GeneralParser<T_Point> {
public:
  Udp4_7Parser();

  virtual ~Udp4_7Parser();                         

  int16_t GetVecticalAngle(int channel); 

  bool IsNeedFrameSplit(uint16_t frame_id);  
                                
  virtual void LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  virtual void LoadFiretimesFile(std::string firetimes_path);
  virtual int LoadFiretimesString(char *firetimes_string);

  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket); 
    
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index);

  virtual void ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info);
  ATXCorrections m_ATX_corrections;
  ATXFiretimes m_ATX_firetimes;

protected:
  // std::string pcap_file_;
  // std::string lidar_correction_file_;
  bool get_correction_file_;
  int last_frameid_ = -1;
};
}  // namespace lidar
}  // namespace hesai
#ifdef _MSC_VER
#pragma pack(pop)
#endif
#include "udp4_7_parser.cc"
#endif // end of UDP4_7_PARSER_H_