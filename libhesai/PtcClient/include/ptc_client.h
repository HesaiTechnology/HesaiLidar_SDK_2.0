/*
 * Copyright (C) 2019 Hesai Tech<http://www.hesaitech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

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
} __attribute__((packed));


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

#endif /* PtcClient_H */
