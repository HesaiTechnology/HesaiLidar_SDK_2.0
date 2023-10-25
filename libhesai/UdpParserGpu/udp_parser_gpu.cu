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
#include "udp_parser_gpu.h"
#include "udp_p40_parser_gpu.h"
#include "udp_p64_parser_gpu.h"
#include "udp4_3_parser_gpu.h"
#include "udp1_4_parser_gpu.h"
#include "udp3_1_parser_gpu.h"
#include "udp3_2_parser_gpu.h"
#include "udp6_1_parser_gpu.h"
#include "udp7_2_parser_gpu.h"
#include "udp2_5_parser_gpu.h"

using namespace hesai::lidar;
template <typename T_Point>
int UdpParserGpu<T_Point>::LoadCorrectionString(char *correction_content) {
  if (m_generalParserGpu != nullptr) {
    return m_generalParserGpu->LoadCorrectionString(correction_content);
  }
  else {
    printf("m_generalParserGpu is nullptr\n");
    return -1;
  }
  return 0;
}
template <typename T_Point>
int UdpParserGpu<T_Point>::LoadCorrectionFile(std::string correction_path) {
  if (m_generalParserGpu != nullptr) {
    return m_generalParserGpu->LoadCorrectionFile(correction_path);
  }
  else {
    printf("m_generalParserGpu is nullptr\n");
    return -1;
  }
  return 0;
}


template <typename T_Point>
int UdpParserGpu<T_Point>::LoadFiretimesString(char *correction_string) {
  if (m_generalParserGpu != nullptr) {
    m_generalParserGpu->LoadFiretimesString(correction_string);
  }
  else {
    printf("m_generalParserGpu is nullptr\n");
    return -1;
  }
  return 0;
}
template <typename T_Point>
void UdpParserGpu<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  if (m_generalParserGpu != nullptr) {
    m_generalParserGpu->LoadFiretimesFile(firetimes_path);
  }
  else {
    printf("m_generalParserGpu is nullptr\n");
  }
}
template <typename T_Point>
void UdpParserGpu<T_Point>::CreatGeneralParser(uint8_t major, uint8_t minor) {
  // QMutexLocker locker(&this->m_mutex);
  if (m_generalParserGpu != nullptr) {
    return;
  }

  switch (major) {
    // AT128
    case 4:  
    {
      switch (minor) {
        case 1:
        case 3:
          m_generalParserGpu = new Udp4_3ParserGpu<T_Point>();
          break;
        default:
          break;
      }

    } break;
    // Pandar128E3X
    case 1:  
    {
      switch (minor) {
        case 1:
        case 4:
          m_generalParserGpu = new Udp1_4ParserGpu<T_Point>();
          break;
        default:
          break;
      }

    } break;
    // ET
    case 2:  
    {
      switch (minor) {
        case 5:
          m_generalParserGpu = new Udp2_5ParserGpu<T_Point>();
          break;
        default:
          break;
      }

    } break;
    // PandarQt
    case 3:  
    {
      switch (minor) {
        case 1:
          m_generalParserGpu = new Udp3_1ParserGpu<T_Point>();
        case 2:
          m_generalParserGpu = new Udp3_2ParserGpu<T_Point>();
          break;
        default:
          break;
      }

    } break;
    // PandarXt
    case 6:  
    {
      switch (minor) {
        case 1:
          m_generalParserGpu = new Udp6_1ParserGpu<T_Point>();
          break;
        default:
          break;
      }

    } break;
    // PandarFt
    case 7:  
    {
      switch (minor) {
        case 2:
          m_generalParserGpu = new Udp7_2ParserGpu<T_Point>();
          break;
        default:
          break;
      }

    } break;
    default:
      break;
  }
}
template <typename T_Point>
int UdpParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (m_generalParserGpu == nullptr) {
    this->CreatGeneralParser(frame.major_version, frame.minor_version);
  }
  m_generalParserGpu->ComputeXYZI(frame);
  return 0;
}
template <typename T_Point>
void UdpParserGpu<T_Point>::SetLidarType(std::string lidar_type) {
  if (m_generalParserGpu != nullptr) {
    return;
  }
  if (lidar_type == "Pandar40" || lidar_type == "Pandar40P") {
    m_generalParserGpu = new UdpP40ParserGpu<T_Point>();
  } 
  if (lidar_type == "Pandar64") {
    m_generalParserGpu = new UdpP64ParserGpu<T_Point>();
  }
  if (lidar_type == "AT128" || lidar_type == "AT128E2X" || lidar_type == "AT128E3X") {
    m_generalParserGpu = new Udp4_3ParserGpu<T_Point>();
  }
  if (lidar_type == "Pandar128E3X" || lidar_type == "Pandar128") {
    m_generalParserGpu = new Udp1_4ParserGpu<T_Point>();
  }
  if (lidar_type == "Pandar90" || lidar_type == "Pandar90E3X") {
    m_generalParserGpu = new Udp1_4ParserGpu<T_Point>();
  }
  if (lidar_type == "Pandar64S" || lidar_type == "Pandar64E3X") {
    m_generalParserGpu = new Udp1_4ParserGpu<T_Point>();
  }
  if (lidar_type == "Pandar40S" || lidar_type == "Pandar40E3X") {
    m_generalParserGpu = new Udp1_4ParserGpu<T_Point>();
  }
  if (lidar_type == "PandarQT") {
    m_generalParserGpu = new Udp3_1ParserGpu<T_Point>();
  } 
  if (lidar_type == "PandarQT128" || lidar_type == "QT128C2X") {
    m_generalParserGpu = new Udp3_2ParserGpu<T_Point>();
  } 
  if (lidar_type == "PandarXT") {
    m_generalParserGpu = new Udp6_1ParserGpu<T_Point>();
  }
  if (lidar_type == "PandarXT16" || lidar_type == "PandarXT-16") {
    m_generalParserGpu = new Udp6_1ParserGpu<T_Point>();
  }
  if (lidar_type == "PandarXT32" || lidar_type == "PandarXT-32") {
    m_generalParserGpu = new Udp6_1ParserGpu<T_Point>();
  }
  if (lidar_type == "PandarXTM" || lidar_type == "XT32M2X") {
    m_generalParserGpu = new Udp6_1ParserGpu<T_Point>();
  }
  if (lidar_type == "PandarFT120" || lidar_type == "FT120C1X") {
    m_generalParserGpu = new Udp7_2ParserGpu<T_Point>();
  }
  if (lidar_type == "ET" || lidar_type == "ET25" || lidar_type == "ET25-E2X") {
    m_generalParserGpu = new Udp2_5ParserGpu<T_Point>();
  }           

}

template<typename T_Point>
int UdpParserGpu<T_Point>::SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw) {
  if (m_generalParserGpu != nullptr) {
    m_generalParserGpu->SetTransformPara(x, y, z, roll, pitch, yaw);
    return 0;
  }
  return -1;
}

