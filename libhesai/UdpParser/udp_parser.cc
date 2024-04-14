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
#include "udp_parser.h"
#include "general_parser.h"
using namespace hesai::lidar;


template<typename T_Point>
UdpParser<T_Point>::UdpParser(uint8_t major, uint8_t minor) {
  parser_ = nullptr;
  pcap_saver_ = nullptr;
  fisrt_packet_ = true;
  pcap_time_synchronization_ = true;
  this->CreatGeneralParser(major, minor);
}

template<typename T_Point>
UdpParser<T_Point>::UdpParser(UdpPacket& packet) {
  parser_ = nullptr;
  pcap_saver_ = nullptr;
  fisrt_packet_ = true;
  pcap_time_synchronization_ = true;
  this->CreatGeneralParser(packet);
}

template<typename T_Point>
UdpParser<T_Point>::UdpParser(std::string lidar_type) {
  parser_ = nullptr;
  pcap_saver_ = nullptr;
  fisrt_packet_ = true;
  pcap_time_synchronization_ = true;
  this->CreatGeneralParser(lidar_type);
}

template<typename T_Point>
UdpParser<T_Point>::UdpParser() {
  parser_ = nullptr;
  pcap_saver_ = nullptr;
  fisrt_packet_ = true;
  pcap_time_synchronization_ = true;
}

template<typename T_Point>
UdpParser<T_Point>::~UdpParser() {
  if (parser_ != nullptr) {
    delete parser_;
    parser_ = nullptr;
  }

  if (pcap_saver_ != nullptr) {
    delete pcap_saver_;
    pcap_saver_ = nullptr;
  }
}

template<typename T_Point>
void UdpParser<T_Point>::LoadCorrectionFile(std::string correction_path) {
  if (parser_ != nullptr) {
    parser_->LoadCorrectionFile(correction_path);
  }
  return;
}

template<typename T_Point>
int UdpParser<T_Point>::LoadCorrectionString(char *correction_string) {
  if (parser_ != nullptr) {
    return parser_->LoadCorrectionString(correction_string);
  }
  return -1;
}

template<typename T_Point>
void UdpParser<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  if (parser_ != nullptr) {
    parser_->LoadFiretimesFile(firetimes_path);
  }
  return;
}

template<typename T_Point>
void UdpParser<T_Point>::CreatGeneralParser(uint8_t major, uint8_t minor) {
  if (parser_ != nullptr) {
    return;
  }
  switch (major) {
    case 1:  // Pandar128
    {
      switch (minor) {
        case 4:
          parser_ = new Udp1_4Parser<T_Point>();
          lidar_type_decoded_ = "Pandar128";
          break;
        default:
          break;
      }
    } break;
    case 2:  // ET
    {
      switch (minor) {
        case 4:
          parser_ = new Udp2_4Parser<T_Point>();
          lidar_type_decoded_ = "ET25-E1X";
        case 5:
          parser_ = new Udp2_5Parser<T_Point>();  // ET25
          lidar_type_decoded_ = "ET25-E2X";
          break;
        default:
          break;
      }

    } break;
    case 3:  // QT
    {
      switch (minor) {
        case 1:
          parser_ = new Udp3_1Parser<T_Point>();  // QT32 QT64
          lidar_type_decoded_ = "PandarQT";
          break;
        case 2:
          parser_ = new Udp3_2Parser<T_Point>();  // QT128
          lidar_type_decoded_ = "PandarQT128";
          break;
        default:
          break;
      }

    } break;
    case 4:  // AT128
    {
      switch (minor) {
        case 1:
        case 3:
          parser_ = new Udp4_3Parser<T_Point>();
          lidar_type_decoded_ = "AT128";
          break;
        default:
          break;
      }

    } break;
    case 6:  // PandarXT
    {
      switch (minor) {
        case 1:
          parser_ = new Udp6_1Parser<T_Point>();
          lidar_type_decoded_ = "PandarXT";
          break;
        default:
          break;
      }

    } break;
    case 7:  // PandarFT
    {
      switch (minor) {
        case 2:
          parser_ = new Udp7_2Parser<T_Point>();
          lidar_type_decoded_ = "PandarFT120";
          break;
        default:
          break;
      }

    } break;
    default:
      break;
  }
  return ;
}

template<typename T_Point>
void UdpParser<T_Point>::CreatGeneralParser(const UdpPacket& packet) {
  if (parser_ != nullptr) {
    return;
  }
  if (PKT_SIZE_40P == packet.packet_len ||
      PKT_SIZE_40P + 4 == packet.packet_len || PKT_SIZE_AC == packet.packet_len ||
      PKT_SIZE_AC + 4 == packet.packet_len) {

    // Pandar40
    parser_ = new UdpP40Parser<T_Point>();  
    lidar_type_decoded_ = "Pandar40";
    return;
  }
  if (PKT_SIZE_64 == packet.packet_len || PKT_SIZE_64 + 4 == packet.packet_len ||
      PKT_SIZE_20 == packet.packet_len || PKT_SIZE_20 + 4 == packet.packet_len) {

    // Pandar64
    parser_ = new UdpP64Parser<T_Point>();  
    lidar_type_decoded_ = "Pandar64";
    return;
  }
  if (packet.buffer[0] != 0xEE && packet.buffer[1] != 0xFF) {
    printf("Packet with invaild delimiter\n");
    return;
  }
  uint8_t UdpMajorVersion = packet.buffer[2];
  uint8_t UdpMinorVersion = packet.buffer[3];
  this->CreatGeneralParser(UdpMajorVersion, UdpMinorVersion);
  return;
}
template<typename T_Point>
void UdpParser<T_Point>::CreatGeneralParser(std::string lidar_type) {
  if (parser_ != nullptr) {
    return;
  }

  if (lidar_type == "AT128" || lidar_type == "AT128E2X" || lidar_type == "AT128E3X") {
    parser_ = new Udp4_3Parser<T_Point>();
  } else if (lidar_type == "Pandar128E3X" || lidar_type == "Pandar128") {
    parser_ = new Udp1_4Parser<T_Point>();
  } else if (lidar_type == "Pandar40S" || lidar_type == "Pandar40E3X") {
    parser_ = new Udp1_4Parser<T_Point>();
  } else if (lidar_type == "Pandar60S" || lidar_type == "Pandar64E3X") {
    parser_ = new Udp1_4Parser<T_Point>();
  } else if (lidar_type == "Pandar90" || lidar_type == "Pandar90E3X") {
    parser_ = new Udp1_4Parser<T_Point>();  
  } else if (lidar_type == "PandarXT") {
    parser_ = new Udp6_1Parser<T_Point>();
  } else if (lidar_type == "PandarXT16" || lidar_type == "PandarXT-16") {
    parser_ = new Udp6_1Parser<T_Point>();
  } else if (lidar_type == "PandarXT32" || lidar_type == "PandarXT-32") {
    parser_ = new Udp6_1Parser<T_Point>();
  } else if (lidar_type == "PandarXTM" || lidar_type == "XT32M2X") {
    parser_ = new Udp6_1Parser<T_Point>();  
  } else if (lidar_type == "PandarQT") {
    parser_ = new Udp3_1Parser<T_Point>();
  } else if (lidar_type == "PandarQT128" || lidar_type == "QT128C2X") {
    parser_ = new Udp3_2Parser<T_Point>();
  } else if (lidar_type == "Pandar64") {
    parser_ = new UdpP64Parser<T_Point>();
  } else if (lidar_type == "Pandar40" || lidar_type == "Pandar40P") {
    parser_ = new UdpP40Parser<T_Point>();
  } else if (lidar_type == "PandarFT120" || lidar_type == "FT120C1X") {
    parser_ = new Udp7_2Parser<T_Point>();
  } else if ( lidar_type == "ET25-E1X" ) {
    parser_ = new Udp2_4Parser<T_Point>();
  } else if (lidar_type == "ET25-E2X" || lidar_type == "ET25" || lidar_type == "ET") {
    parser_ = new Udp2_5Parser<T_Point>();
  }
}
template<typename T_Point>
PcapSaver *UdpParser<T_Point>::GetPcapSaver() {
  if (pcap_saver_ == nullptr) {
    pcap_saver_ = new PcapSaver;
  }
  return pcap_saver_;
}
template<typename T_Point>
GeneralParser<T_Point> *UdpParser<T_Point>::GetGeneralParser() { return parser_; }

template<typename T_Point>
void UdpParser<T_Point>::SetGeneralParser(GeneralParser<T_Point> *parser) { parser_ = parser; }

template<typename T_Point>
void UdpParser<T_Point>::EnableUpdateMonitorInfo() {
  if (parser_ != nullptr) {
    parser_->EnableUpdateMonitorInfo();
  }
  return;
}

template<typename T_Point>
void UdpParser<T_Point>::DisableUpdateMonitorInfo() {
  if (parser_ != nullptr) {
    parser_->DisableUpdateMonitorInfo();
  }
  return;
}

template<typename T_Point>
uint16_t *UdpParser<T_Point>::GetMonitorInfo1() {
  if (parser_ != nullptr) {
    return parser_->GetMonitorInfo1();
  }
}

template<typename T_Point>
uint16_t *UdpParser<T_Point>::GetMonitorInfo2() {
  if (parser_ != nullptr) {
    return parser_->GetMonitorInfo2();
  }
}

template<typename T_Point>
uint16_t *UdpParser<T_Point>::GetMonitorInfo3() {
  if (parser_ != nullptr) {
    return parser_->GetMonitorInfo3();
  }
}

template<typename T_Point>
int UdpParser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  if (parser_ == nullptr) {
    return -1;
  } else {
    return parser_->ComputeXYZI(frame, packet);
  }
}

template<typename T_Point>
int UdpParser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  if(pcap_saver_ == nullptr){
    pcap_saver_ = new PcapSaver;
  }
  if (parser_ == nullptr) {
    // Udp raw_udp_packet
    this->CreatGeneralParser(udpPacket);
    return 0;
  } else {
    int res = parser_->DecodePacket(output, udpPacket);
    //data from pcap and play rate synchronize with the host time
    if (source_type_ == 2 && pcap_time_synchronization_ == true) {
      if(fisrt_packet_ == true) {
        last_host_timestamp_ = output.host_timestamp;
        last_sensor_timestamp_ = output.sensor_timestamp;
        packet_count_ = 1;
        fisrt_packet_ = false;
      } else {
        packet_count_ += 1;
        if (packet_count_ >= kPcapPlaySynchronizationCount) {
          int reset_time = static_cast<int>((output.sensor_timestamp - last_sensor_timestamp_) - (output.host_timestamp - last_host_timestamp_));
          last_host_timestamp_ = output.host_timestamp;
          last_sensor_timestamp_ = output.sensor_timestamp;
          packet_count_ = 0;
          if (reset_time > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(reset_time));
          }
        }
      }
    }
    return res;
  }
}

template<typename T_Point>
int UdpParser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  if (parser_ == nullptr) {
    uint8_t UdpMajorVersion = udpPacket.buffer[2];
    uint8_t UdpMinorVersion = udpPacket.buffer[3];
    this->CreatGeneralParser(UdpMajorVersion, UdpMinorVersion);
  }
  if(pcap_saver_ == nullptr){
    pcap_saver_ = new PcapSaver;
  }
  if (parser_ != nullptr) {
    return parser_->DecodePacket(frame, udpPacket);
  }

}


template<typename T_Point>
int UdpParser<T_Point>::GetGeneralParser(GeneralParser<T_Point> **parser) {
  if (parser_ != nullptr) {
    *parser = parser_;
    return 1;
  } else {
    return -1;
  }
}

template<typename T_Point>
int UdpParser<T_Point>::SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw) {
  if (parser_ != nullptr) {
    parser_->SetTransformPara(x, y, z, roll, pitch, yaw);
    return 0;
  }
  return -1;
}

template<typename T_Point>
void UdpParser<T_Point>::SetPcapPlay(bool pcap_time_synchronization, int source_type) {
  pcap_time_synchronization_ = pcap_time_synchronization;
  source_type_ = source_type;
}

template<typename T_Point>
void UdpParser<T_Point>::SetFrameAzimuth(float frame_start_azimuth) {
  if (parser_ != nullptr) {
    parser_->SetFrameAzimuth(frame_start_azimuth);
  } else {
    printf("parser is nullptr\n");
  }
  return;
}
