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
 * File:   socket_source.h
 * Author: Felix Zou<zouke@hesaitech.com>
 *
 * Created on Sep 5, 2019, 10:46 AM
 */

#ifndef SOCKETSTREAMER_H
#define SOCKETSTREAMER_H

#include <stdint.h>
#include <string>
#include "source.h"
namespace hesai
{
namespace lidar
{

class SocketSource : public Source{
 public:
  SocketSource(uint16_t port = kUdpPort, std::string multicastIp = "");
  ~SocketSource();

  virtual bool Open();
  virtual void Close();
  virtual bool IsOpened();
  virtual int Send(uint8_t* u8Buf, uint16_t u16Len, int flags = 0);
  virtual int Receive(UdpPacket& udpPacket, uint16_t u16Len, int flags = 0,
                      int timeout = 1000);
  virtual void SetSocketBufferSize(uint32_t u32BufSize);
private:
  std::string multicast_ip_;
  std::string client_ip_;
  uint16_t udp_port_;
  int udp_sock_;
  bool is_select_;
  static const int32_t kUDPBufferSize = 26214400;  // udp buffer size
  static const uint16_t kUdpPort = 2368;
};
}  // namespace lidar
}  // namespace hesai
#endif /* SOCKETSTREAMER_H */
