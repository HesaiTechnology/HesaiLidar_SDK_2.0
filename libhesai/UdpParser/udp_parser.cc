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
UdpParser<T_Point>::UdpParser(const UdpPacket& packet) {
  last_host_timestamp_ = 0;
  last_sensor_timestamp_ = 0;
  packet_count_ = 0;
  source_type_ = -1;
  parser_ = nullptr;
  pcap_saver_ = nullptr;
  fisrt_packet_ = true;
  pcap_time_synchronization_ = true;
  this->CreatGeneralParser(packet);
}

template<typename T_Point>
UdpParser<T_Point>::UdpParser(const std::string &lidar_type) {
  last_host_timestamp_ = 0;
  last_sensor_timestamp_ = 0;
  packet_count_ = 0;
  source_type_ = -1;
  parser_ = nullptr;
  pcap_saver_ = nullptr;
  fisrt_packet_ = true;
  pcap_time_synchronization_ = true;
  this->CreatGeneralParser(lidar_type);
}

template<typename T_Point>
UdpParser<T_Point>::UdpParser() {
  last_host_timestamp_ = 0;
  last_sensor_timestamp_ = 0;
  packet_count_ = 0;
  source_type_ = -1;
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
  if (packet.buffer[0] != 0xEE || packet.buffer[1] != 0xFF) {
    LogWarning("Packet with invaild delimiter");
    return;
  }
  uint8_t major = packet.buffer[2];
  uint8_t minor = packet.buffer[3];

  switch (major) {
    case 1:
    {
      switch (minor) {
        case 4: {
          uint8_t statusInfoVersion = packet.buffer[4];
          if (statusInfoVersion > 127) {
            lidar_type_decoded_ = "OT128";
            parser_ = new Udp1_4Parser<T_Point>(STR_OT128);
          } else {
            lidar_type_decoded_ = "Pandar128";
            parser_ = new Udp1_4Parser<T_Point>(STR_PANDARN);
          }
        } break;
        case 8:
          parser_ = new Udp1_8Parser<T_Point>();
          lidar_type_decoded_ = "JT16";
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
          break;
        case 5:
          parser_ = new Udp2_5Parser<T_Point>();  // ET25
          lidar_type_decoded_ = "ET25-E2X";
          break;
        case 6:
          parser_ = new Udp2_6Parser<T_Point>();
          lidar_type_decoded_ = "ET25-HA2";
          break;
        case 7:
          parser_ = new Udp2_7Parser<T_Point>();
          lidar_type_decoded_ = "ET30-HA2";
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
        case 7:
          parser_ = new Udp4_7Parser<T_Point>();
          lidar_type_decoded_ = "ATX";
          break;
        default:
          break;
      }

    } break;
    case 6:  // PandarXT
    {
      switch (minor) {
        case 1: {
          uint8_t blockNum = packet.buffer[7];
          if (blockNum == 8) {
            lidar_type_decoded_ = "PandarXT";
            parser_ = new Udp6_1Parser<T_Point>(STR_XTM1);
          } else if (blockNum == 6) {
            lidar_type_decoded_ = "PandarXTM";
            parser_ = new Udp6_1Parser<T_Point>(STR_XTM2);
          } else {
            // It doesn't normally go here.
            lidar_type_decoded_ = "PandarXTM";
            parser_ = new Udp6_1Parser<T_Point>(STR_XTM2);
            LogError("XT version unknown, unable to set distance correction coordinates");
          }
        } break;
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
  return;
}
template<typename T_Point>
void UdpParser<T_Point>::CreatGeneralParser(const std::string& lidar_type) {
  if (parser_ != nullptr) {
    return;
  }

  if (lidar_type == "Pandar64") {
    parser_ = new UdpP64Parser<T_Point>();
    lidar_type_decoded_ = "Pandar40";
  } else if (lidar_type == "Pandar40" || lidar_type == "Pandar40P") {
    parser_ = new UdpP40Parser<T_Point>();
    lidar_type_decoded_ = "Pandar64";
  } else if (lidar_type == "Pandar128E3X" || lidar_type == "Pandar128" || 
             lidar_type == "Pandar40S" || lidar_type == "Pandar40E3X" ||
             lidar_type == "Pandar60S" || lidar_type == "Pandar64E3X" ||
             lidar_type == "Pandar90" || lidar_type == "Pandar90E3X") 
  {
    parser_ = new Udp1_4Parser<T_Point>(STR_PANDARN);
    lidar_type_decoded_ = "Pandar128";
  } else if (lidar_type == "OT128") {
    parser_ = new Udp1_4Parser<T_Point>(STR_OT128);
    lidar_type_decoded_ = "OT128";
  } else if ( lidar_type == "JT16" ) {
    parser_ = new Udp1_8Parser<T_Point>();
    lidar_type_decoded_ = "JT16";
  } else if (lidar_type == "ET25-E1X" ) {
    parser_ = new Udp2_4Parser<T_Point>();
    lidar_type_decoded_ = "ET25-E1X";
  } else if (lidar_type == "ET25-E2X") {
    parser_ = new Udp2_5Parser<T_Point>();
    lidar_type_decoded_ = "ET25-E2X";
  } else if (lidar_type == "ET25-HA2") {
    parser_ = new Udp2_6Parser<T_Point>();
    lidar_type_decoded_ = "ET25-HA2";
  } else if (lidar_type == "ET30-HA2") {
    parser_ = new Udp2_7Parser<T_Point>();
    lidar_type_decoded_ = "ET30-HA2";
  } else if (lidar_type == "PandarQT") {
    parser_ = new Udp3_1Parser<T_Point>();
    lidar_type_decoded_ = "PandarQT";
  } else if (lidar_type == "PandarQT128" || lidar_type == "QT128C2X") {
    parser_ = new Udp3_2Parser<T_Point>();
    lidar_type_decoded_ = "PandarQT128";
  } else if (lidar_type == "AT128" || lidar_type == "AT128E2X" || lidar_type == "AT128E3X") {
    parser_ = new Udp4_3Parser<T_Point>();
    lidar_type_decoded_ = "AT128";
  } else if ( lidar_type == "ATX" ) {
    parser_ = new Udp4_7Parser<T_Point>();
    lidar_type_decoded_ = "ATX";
  } else if (lidar_type == "PandarXT" || lidar_type == "PandarXT16" || lidar_type == "PandarXT-16" || 
             lidar_type == "PandarXT32" || lidar_type == "PandarXT-32") 
  {
    parser_ = new Udp6_1Parser<T_Point>(STR_XTM1);
    lidar_type_decoded_ = "PandarXT";
  } else if (lidar_type == "PandarXTM" || lidar_type == "XT32M2X") {
    parser_ = new Udp6_1Parser<T_Point>(STR_XTM2);  
    lidar_type_decoded_ = "PandarXTM";
  } else if (lidar_type == "PandarFT120" || lidar_type == "FT120C1X") {
    parser_ = new Udp7_2Parser<T_Point>();
    lidar_type_decoded_ = "PandarFT120";
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
int UdpParser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  if (parser_ == nullptr) {
    return -1;
  } else {
    return parser_->ComputeXYZI(frame, packet_index);
  }
}

template<typename T_Point>
int UdpParser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  if(pcap_saver_ == nullptr){
    pcap_saver_ = new PcapSaver;
  }
  if (udpPacket.packet_len < 6) {   // sizeof(HS_LIDAR_PRE_HEADER)
    frame.scan_complete = false;
    return -1;
  }
  if (parser_ == nullptr) {
    // Udp raw_udp_packet
    this->CreatGeneralParser(udpPacket);
    return 0;
  } else {
    int res = parser_->DecodePacket(frame, udpPacket);
    //data from pcap and play rate synchronize with the host time
    if (source_type_ == 2 && pcap_time_synchronization_ == true && res == 0) {
      if(fisrt_packet_ == true) {
        last_host_timestamp_ = frame.host_timestamp;
        last_sensor_timestamp_ = frame.sensor_timestamp[frame.packet_num - 1];
        packet_count_ = 1;
        fisrt_packet_ = false;
      } else {
        packet_count_ += 1;
        if (packet_count_ >= kPcapPlaySynchronizationCount) {
          int reset_time = static_cast<int>((frame.sensor_timestamp[frame.packet_num - 1] - last_sensor_timestamp_) - (frame.host_timestamp - last_host_timestamp_));
          last_host_timestamp_ = frame.host_timestamp;
          last_sensor_timestamp_ = frame.sensor_timestamp[frame.packet_num - 1];
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
int UdpParser<T_Point>::ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info) {
  if (parser_ == nullptr) {
    return -1;
  } else {
    parser_->ParserFaultMessage(udp_packet, fault_message_info);
    return 0;
  }
  return 0;
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
    LogWarning("parser is nullptr");
  }
  return;
}
