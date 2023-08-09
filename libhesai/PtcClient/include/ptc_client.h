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
 * File:   ptc_client.h
 * Author: Felix Zou<zouke@hesaitech.com>
 *
 * Created on Jun 20, 2019, 10:46 AM
 */

#ifndef PtcClient_H
#define PtcClient_H

#include <endian.h>
#include <semaphore.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include "tcp_client.h"
#include "lidar_types.h"
#define PKT_SIZE_40P (1262)
#define PKT_SIZE_AC (1256)
#define PKT_SIZE_64 (1194)
#define PKT_SIZE_20 (1270)
#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif
namespace hesai
{
namespace lidar
{

struct PTCHeader {
  uint8_t m_u8Identifier0;
  uint8_t m_u8Identifier1;
  uint8_t m_u8Cmd;
  uint8_t m_u8RspCode;
  uint32_t m_u32Len;  //有效载荷长度

  static const uint8_t kIdentifier0 = 0x47;
  static const uint8_t kIdentifier1 = 0x74;

  bool IsValidIdentifier() const {
    return m_u8Identifier0 == kIdentifier0 && m_u8Identifier1 == kIdentifier1;
  }

  static uint8_t Identifier0() { return kIdentifier0; }
  static uint8_t Identifier1() { return kIdentifier1; }
  //判断返回位是否是有效  默认返回值是false  &&具有短路的功效
  bool IsValidRsp() const { return IsValidIdentifier() && m_u8RspCode == 0; }
  uint8_t GetRspCode() const { return m_u8RspCode; }

  void Init(uint8_t u8Cmd) {
    m_u8Identifier0 = kIdentifier0;
    m_u8Identifier1 = kIdentifier1;
    m_u8Cmd = u8Cmd;
    m_u8RspCode = 0;
    m_u32Len = 0;
  }
  //设置有效载荷长度/获取有效载荷长度
  void SetPayloadLen(uint32_t u32Len) { m_u32Len = htobe32(u32Len); }
  uint32_t GetPayloadLen() const { return be32toh(m_u32Len); }
} PACKED;


class PtcClient : public TcpClient {
 public:
  static const uint8_t kPTCGetLidarCalibration = 0x05;
  static const uint8_t kPTCGetLidarFiretimes = 0xA9;
  static const uint8_t kPTCGetLidarChannelConfig = 0xA8;
  PtcClient(std::string IP = kLidarIPAddr,
                       uint16_t u16TcpPort = kTcpPort);
  ~PtcClient() {}

  PtcClient(const PtcClient &orig) = delete;

  bool PTCEncode(u8Array_t &byteStreamIn, u8Array_t &byteStreamOut,
                 uint8_t u8Cmd);
  bool PTCDecode(u8Array_t &byteStreamIn, u8Array_t &byteStreamOut);
  bool IsValidRsp(u8Array_t &byteStreamIn);

  void TcpFlushIn();
  int QueryCommand(u8Array_t &byteStreamIn, u8Array_t &byteStreamOut, uint8_t u8Cmd );
  int SendCommand(u8Array_t &byteStreamIn, uint8_t u8Cmd);

  u8Array_t GetCorrectionInfo();
  int GetCorrectionInfo(u8Array_t &dataOut);
  int GetFiretimesInfo(u8Array_t &dataOut);
  int GetChannelConfigInfo(u8Array_t &dataOut);
  int SetSocketTimeout(uint32_t u32RecMillisecond, uint32_t u32SendMillisecond);
  void CRCInit();
  uint32_t CRCCalc(uint8_t *bytes, int len); 
  uint32_t m_CRCTable[256];                                              

 private:
  static const std::string kLidarIPAddr;
  static const uint16_t kTcpPort = 9347;
  uint16_t m_u16PtcPort;
  bool running_;
};
}
}
#ifdef _MSC_VER
#pragma pack(pop)
#endif
#endif /* PtcClient_H */
