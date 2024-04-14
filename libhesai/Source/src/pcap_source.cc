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
#include "pcap_source.h"

#include <stdexcept>
#include <cassert>
#include <cstring>
#include <iostream>

using namespace hesai::lidar;

static constexpr uint32_t pcap_magic_number = 0xa1b2c3d4;

PcapIPHeader::PcapIPHeader(uint8_t protocol, uint16_t pkt_len)
    : ether{ {0xff,0xff,0xff,0xff,0xff,0xff}, {0x00, 0x0a, 0x35, 0x00, 0x1e, 0x53}, 0x0008 }
    , ip{ 0x5, 0x4, 0x00, uint16_t((((pkt_len + 20) & 0xff) << 8) | ((pkt_len + 20) >> 8)), 0xf17a, 0x0040, 0x40, protocol, 0x80f8, 0xc901a8c0, 0xffffffff }
{}

PcapIPv6Header::PcapIPv6Header(uint8_t protocol, uint16_t pkt_len)
    : ether{ {0xff,0xff,0xff,0xff,0xff,0xff}, {0x00, 0x0a, 0x35, 0x00, 0x1e, 0x53}, 0xdd86 }
    , ipv6 { 0x6, 0, 0, 0, uint8_t((((pkt_len + 40) & 0xff) << 8) | ((pkt_len + 40) >> 8)), protocol, }
{}

PcapUDPHeader::PcapUDPHeader(uint16_t pkt_len, uint16_t port)
    : PcapIPHeader::PcapIPHeader(0x11, pkt_len + 8)
    , udp{ 0x1027, uint16_t(((port & 0xff) << 8) | (port >> 8)), uint16_t((((pkt_len + 8) & 0xff) << 8) | ((pkt_len + 8) >> 8)), 0x0000 }
{}

PcapUDPv6Header::PcapUDPv6Header(uint16_t pkt_len, uint16_t port)
    : PcapIPv6Header::PcapIPv6Header(0x11, pkt_len + 8)
    , udp{ 0x1027, uint16_t(((port & 0xff) << 8) | (port >> 8)), uint16_t((((pkt_len + 8) & 0xff) << 8) | ((pkt_len + 8) >> 8)), 0x0000 }
{}

PcapTCPHeader::PcapTCPHeader(uint16_t pkt_len, uint16_t port)
    : PcapIPHeader::PcapIPHeader(0x06, pkt_len + 20)
    , tcp{ 0x1027, uint16_t(((port & 0xff) << 8) | (port >> 8)), 0x000000000, 0x00000000, 0x4, 0x0, 0x0, 0x00, 0x0000, 0x0000, 0x0000 }
{}

PcapTCPv6Header::PcapTCPv6Header(uint16_t pkt_len, uint16_t port)
    : PcapIPv6Header::PcapIPv6Header(0x06, pkt_len + 20)
    , tcp{ 0x1027, uint16_t(((port & 0xff) << 8) | (port >> 8)), 0x000000000, 0x00000000, 0x4, 0x0, 0x0, 0x00, 0x0000, 0x0000, 0x0000 }
{}

class PcapSource::Private {
    std::ifstream _fpcap;
    size_t _fpos;
    std::vector<char> _fileData;

public:
    Private() : _fpos(0) {}
    ~Private() { close(); }

    void open(const std::string& pcap_path) {
        close();
        _fpcap.open(pcap_path, std::ios::binary);
        if (_fpcap.is_open()) {
            _fpcap.seekg(0, std::ios::end);
            _fileData.resize(_fpcap.tellg());
            _fpcap.seekg(0, std::ios::beg);
            _fpcap.read(&_fileData[0], _fileData.size());
        }
    }

    bool is_open() const { return _fpcap.is_open(); }

    void close() {
        if (is_open()) {
            _fpcap.close();
            _fileData.clear();
        }
        _fpos = 0;
    }

    inline bool eof() const { return _fpos == _fileData.size(); }

    inline size_t fpos() const { return _fpos; }

    inline void fpos(size_t new_fpos) { _fpos = new_fpos; }

    template<typename T>
    bool read(T& value) {
        if (_fpos + sizeof(T) > _fileData.size()) return false;
        //std::memcpy(&value, &_fileData[_fpos], sizeof(T));
        value = *(T*)(&_fileData[_fpos]);
        _fpos += sizeof(T);
        return true;
    }

    bool read(void* dst, size_t length) {
        if (_fpos + length > _fileData.size()) return false;
        std::memcpy(dst, &_fileData[_fpos], length);
        _fpos += length;
        return true;
    }

    bool move(size_t length) {
        if (_fpos + length > _fileData.size()) return false;
        _fpos += length;
        return true;
    }
};

PcapSource::PcapSource(std::string path, int packet_interval)
    : _p(new Private)
    , pcap_udp_header_{0, 0, 0, 0}
{
    pcap_path_ = path;
    packet_interval_ = packet_interval;
}

PcapSource::~PcapSource() {
    PcapSource::Close();
    delete _p;
}

PcapSource::Callback PcapSource::callback() const {
    return udp_callback_;
}

void PcapSource::callback(Callback callback) {
    udp_callback_ = callback;
}
PcapSource::Callback PcapSource::tcp_callback() const {
    return tcp_callback_;
}

void PcapSource::tcp_callback(Callback callback) {
    tcp_callback_ = callback;
}

size_t PcapSource::fpos() const
{
    return _p->fpos();
}

void PcapSource::fpos(size_t new_fpos)
{
    _p->fpos(new_fpos);
}

bool PcapSource::Open() {

    try
    {
        PcapSource::Close();
        _p->open(pcap_path_);
        if (!_p->is_open()) {
            throw std::runtime_error(std::string("Fail to open .pcap file: \"") + (pcap_path_)+"\"");
        }
        _p->read(pcap_header_);
        if (pcap_header_.magic_number != pcap_magic_number) {
            throw std::runtime_error(std::string("Not a valid .pcap file: \"") + (pcap_path_)+"\"");
        }
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }

}

std::string PcapSource::pcap_path() const {
    return pcap_path_;
}

int PcapSource::next(UdpPacket& udpPacket, uint16_t u16Len, int flags,
                      int timeout) {                 
    if(_p->eof()) {
        is_pcap_end = true;
        return -1;
    }
    bool ret = _p->read(pcap_record_);
    if (!ret) return -1;
    if (pcap_record_.incl_len <= 1500) {
        ret = _p->read(payload_.data(), pcap_record_.incl_len);
        if (!ret) return -1;
        switch (((Ethernet*)payload_.data())->ether_type)
        {
        case 0x0008: // ipv4
            switch (((PcapIPHeader*)payload_.data())->ip.protocol)
            {
            case 17:
                pcap_udp_header_ = *(const UDP*)(payload_.data() + sizeof(PcapIPHeader));
                u16Len = pcap_record_.incl_len - sizeof(PcapUDPHeader);
                memcpy(udpPacket.buffer, payload_.data() + sizeof(PcapUDPHeader), pcap_record_.incl_len - sizeof(PcapUDPHeader));
                return pcap_record_.incl_len - sizeof(PcapUDPHeader);
                break;
            case 6:
                pcap_tcp_header_ = *(const TCP*)(payload_.data() + sizeof(PcapIPHeader));
                if (tcp_callback_)
                    return pcap_record_.incl_len - sizeof(PcapTCPHeader);
                else
                    return 2;
                break;
            default:
                return 2;
                break;
            }
            break;
        case 0xdd86: // ipv6 
            switch (((PcapIPv6Header*)payload_.data())->ipv6.next_header)
            {
            case 17:
                pcap_udp_header_ = *(const UDP*)(payload_.data() + sizeof(PcapIPv6Header));
                memcpy(udpPacket.buffer, payload_.data() + sizeof(PcapUDPv6Header), pcap_record_.incl_len - sizeof(PcapUDPv6Header));
                return pcap_record_.incl_len - sizeof(PcapUDPv6Header);
                break;
            case 6:
                pcap_tcp_header_ = *(const TCP*)(payload_.data() + sizeof(PcapIPv6Header));
                if (tcp_callback_)
                    return pcap_record_.incl_len - sizeof(PcapTCPv6Header);
                else
                    return 2;
                break;
            default:
                return 2;
                break;
            }
            break;
        case 0x0081:  // vlan tag
        switch (*((uint16_t*)(payload_.data() + sizeof(Ethernet) + 2)))
            {
            case 0x0008:
                switch (*((uint8_t*)(payload_.data() + sizeof(Ethernet) + 13)))
                {
                case 17:
                    pcap_udp_header_ = *(const UDP*)(payload_.data() + sizeof(PcapIPHeader));
                    memcpy(udpPacket.buffer, payload_.data() + sizeof(PcapUDPHeader) + 4, pcap_record_.incl_len - sizeof(PcapUDPHeader) - 4);
                    return pcap_record_.incl_len - sizeof(PcapUDPHeader) - 4;
                    break;
                case 6:
                    pcap_tcp_header_ = *(const TCP*)(payload_.data() + sizeof(PcapIPHeader));
                    if (tcp_callback_)
                        return pcap_record_.incl_len - sizeof(PcapTCPHeader) - 4;
                    else
                        return 2;
                    break;
                default:
                    return 2;
                    break;
                }
                break;
            case 0xdd86:
                switch (*((uint8_t*)(payload_.data() + sizeof(Ethernet) + 13)))
                {
                case 17:
                    pcap_udp_header_ = *(const UDP*)(payload_.data() + sizeof(PcapIPv6Header));
                    return pcap_record_.incl_len - sizeof(PcapUDPv6Header) - 4;
                    break;
                case 6:
                    pcap_tcp_header_ = *(const TCP*)(payload_.data() + sizeof(PcapIPv6Header));
                    if (tcp_callback_)
                        return pcap_record_.incl_len - sizeof(PcapTCPv6Header) - 4;
                    else
                        return 2;
                    break;
                default:
                    return 2;
                    break;
                }
            default:
                return 2;
                break;
            }
            break;
        default:
        printf("can not parser Ethernet data, type = %x \n", ((Ethernet*)payload_.data())->ether_type);
            break;
        }

    }
    else {
        ret = _p->move(pcap_record_.incl_len);
        if (!ret) return -1;
        return 1;
    }
    return 1;
}
int PcapSource::distinationPort() {
    uint16_t port = pcap_udp_header_.distination_port;
    return ((port & 0xff) << 8) | ((port >> 8) & 0xff);
}

void PcapSource::Close() {
    _p->close();
}


bool PcapSource::IsOpened() {
  return _p->is_open();
}

int PcapSource::Receive(UdpPacket& udpPacket, uint16_t u16Len, int flags,
                       int iTimeout) {
  return next(udpPacket, u16Len, flags, iTimeout);
}

int PcapSource::Send(uint8_t* u8Buf, uint16_t u16Len, int flags) {
  return u16Len;
}

void PcapSource::setPcapPath(std::string path) {
    pcap_path_ = path;
}

void PcapSource::setPacketInterval(int microsecond) {
    packet_interval_ = microsecond;
}