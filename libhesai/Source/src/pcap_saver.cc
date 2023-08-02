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

#include "pcap_saver.h"

#include <stdexcept>
#include <cassert>
#include <cstring>
#include <boost/iostreams/device/mapped_file.hpp>
#include <iostream>
using namespace hesai::lidar;
static constexpr uint32_t pcap_magic_number = 0xa1b2c3d4;


PcapSaver::PcapSaver()
    : ofs_()
    , pcap_path_("")
    , dumping_(false)
    , dumping_blocked_(false)
    , tcp_dumped_(false)
    , packets_cache_(new Container)
{}

PcapSaver::~PcapSaver()
{
    PcapSaver::close();
}

std::string PcapSaver::pcap_path() const {
    return pcap_path_;
}
void PcapSaver::SetPcapPath(std::string pcap_path) {
  pcap_path_ = pcap_path;
}


int PcapSaver::Save() {
    try
    {
    
    PcapSaver::close();
    ofs_.open(pcap_path_, std::ios::binary);
    if(!ofs_.good()) {
        throw std::runtime_error(std::string("Fail to open .pcap file: \"") + (pcap_path_)+"\"");
    }
    PcapHeader pcap_header{
        pcap_magic_number,
        2,
        4,
        0,
        0,
        0xffff,
        1,
    };
    ofs_.write((char*)&pcap_header, sizeof(pcap_header));
    // pcap_path_ = pcap_path;
    {
        dumping_ = true;
        dumping_thread_ = std::thread([this]() {
            using namespace std::chrono_literals;
            while (dumping_ || packets_cache_->not_empty()) {
                while (!dumping_blocked_ && packets_cache_->not_empty()) {
                    auto pkt = packets_cache_->pop_front();
                    auto& len = pkt.size;
                    std::array<uint8_t, 1500> data_with_fake_header;
                    *(PcapUDPHeader*)data_with_fake_header.data() = PcapUDPHeader(len, pkt.port);
                    std::memcpy(data_with_fake_header.data() + sizeof(PcapUDPHeader), pkt.buffer, len);
                    PcapRecord pcap_record;
                    Duration since_begin = Clock::now() - Time();
                    Duration time_of_day = since_begin - int64_t(since_begin / 24h) * 24h;
                    pcap_record.ts_sec = time_of_day / 1s;
                    pcap_record.ts_usec = (time_of_day - pcap_record.ts_sec * 1s) / 1us;
                    pcap_record.orig_len = len + sizeof(PcapUDPHeader);
                    pcap_record.incl_len = len + sizeof(PcapUDPHeader);
                    ofs_.write((char*)&pcap_record, sizeof(pcap_record));
                    ofs_.write((char*)&data_with_fake_header, pcap_record.incl_len);
                }
                std::this_thread::sleep_for(1ms);
            }
        });
    }
    return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
    
}

void PcapSaver::Dump(const uint8_t* data, uint32_t len, uint16_t port) {
    packets_cache_->push_back(PandarPacket(data, len, port));
}
void PcapSaver::TcpDump(const uint8_t* data, uint32_t data_len, uint32_t max_pkt_len, uint16_t port) {
    dumping_blocked_ = true;
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms); // delay to make sure Dump successful

    using namespace std::chrono_literals;
    int remain_len = data_len, pkt_len = max_pkt_len;
    int i = 0;
    while ( remain_len > 0 ) {
        pkt_len = (remain_len > max_pkt_len) ? max_pkt_len : remain_len;
            
        std::array<uint8_t, 1500> data_with_fake_header;
        *(PcapTCPHeader*)data_with_fake_header.data() = PcapTCPHeader(pkt_len, port);
        std::memcpy(data_with_fake_header.data() + sizeof(PcapTCPHeader), data + i * max_pkt_len, pkt_len);
        PcapRecord pcap_record;
        Duration since_begin = Clock::now() - Time();
        Duration time_of_day = since_begin - int64_t(since_begin / 24h) * 24h;
        pcap_record.ts_sec = time_of_day / 1s;
        pcap_record.ts_usec = (time_of_day - pcap_record.ts_sec * 1s) / 1us;
        pcap_record.orig_len = pkt_len + sizeof(PcapTCPHeader);
        pcap_record.incl_len = pkt_len + sizeof(PcapTCPHeader);
        ofs_.write((char*)&pcap_record, sizeof(pcap_record));
        ofs_.write((char*)&data_with_fake_header, pcap_record.incl_len);

        remain_len -= pkt_len;
        i++;
    }

    dumping_blocked_ = false;
}

void PcapSaver::close() {
    dumping_ = false;
    if (dumping_thread_.joinable()) {
        dumping_thread_.join();
    }
    if (ofs_.is_open()) {
        ofs_.close();
    }
    // pcap_path_ = "";
    tcp_dumped_ = false;
}

int PcapSaver::Save(const std::string& recordPath, const UdpFrame_t& packets,
                    int port) {
  try {
    std::ofstream ofs;
    struct stat buf;
    if (stat(recordPath.c_str(), &buf) != 0) {
      ofs.open(recordPath, std::ios::binary | std::ios_base::app);
      if (!ofs.good()) {
        throw std::runtime_error(std::string("Fail to open .pcap file: \"") +
                                 (pcap_path_) + "\"");
      }
      PcapHeader pcap_header{
          pcap_magic_number, 2, 4, 0, 0, 0xffff, 1,
      };
      ofs.write((char*)&pcap_header, sizeof(pcap_header));
    } else {
      ofs.open(recordPath, std::ios::binary | std::ios_base::app);
      if (!ofs.good()) {
        throw std::runtime_error(std::string("Fail to open .pcap file: \"") +
                                 (pcap_path_) + "\"");
      }
    }
    // static unsigned int time_begin = GetMicroTickCount();
    for (int i = 0; i < packets.size(); i++) {
      auto pkt = PandarPacket(packets[i].buffer, packets[i].packet_len, port);
      auto& len = packets[i].packet_len;
      std::array<uint8_t, 1500> data_with_fake_header;
      *(PcapUDPHeader*)data_with_fake_header.data() =
          PcapUDPHeader(len, pkt.port);
      std::memcpy(data_with_fake_header.data() + sizeof(PcapUDPHeader),
                  pkt.buffer, len);
      PcapRecord pcap_record;
    //   unsigned int time_now = GetMicroTickCount();
    //   pcap_record.ts_sec = (time_now - time_begin) / 1000000;
    //   pcap_record.ts_usec = (time_now - time_begin - pcap_record.ts_sec);
      pcap_record.orig_len = len + sizeof(PcapUDPHeader);
      pcap_record.incl_len = len + sizeof(PcapUDPHeader);
      ofs.write((char*)&pcap_record, sizeof(pcap_record));
      ofs.write((char*)&data_with_fake_header, pcap_record.incl_len);
    }
    ofs.close();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }
}

int PcapSaver::Save(const std::string& recordPath,
                    const UdpFrameArray_t& packets, int port) {
  int ret = 0;
  for (int i = 0; i < packets.size(); i++) {
    ret = Save(recordPath, packets[i], port);
    if (ret != 0) {
      break;
    }
  }

  return ret;
}



