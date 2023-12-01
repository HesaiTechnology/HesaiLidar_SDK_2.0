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

#include <vector>
#include "tcp_client.h"
#include "tcp_ssl_client.h"
#include "lidar_types.h"
#include "driver_param.h"
#include "ptc_parser.h"

#define PKT_SIZE_40P (1262)
#define PKT_SIZE_AC (1256)
#define PKT_SIZE_64 (1194)
#define PKT_SIZE_20 (1270)
namespace hesai
{
namespace lidar
{

const uint8_t kPTCGetLidarCalibration = 0x05;
const uint8_t kPTCGetInventoryInfo = 0x07;
const uint8_t kPTCGetLidarFiretimes = 0xA9;
const uint8_t kPTCGetLidarChannelConfig = 0xA8;

class PtcClient {
 public:
  PtcClient(std::string IP = kLidarIPAddr
            , uint16_t u16TcpPort = kTcpPort
            , bool bAutoReceive = false
            , PtcMode client_mode = PtcMode::tcp
            , uint8_t ptc_version = 1
            , const char* cert = nullptr
            , const char* private_key = nullptr
            , const char* ca = nullptr);
  ~PtcClient() {}

  PtcClient(const PtcClient &orig) = delete;

  bool IsValidRsp(u8Array_t &byteStreamIn);

  void TcpFlushIn();
  int QueryCommand(u8Array_t &byteStreamIn, u8Array_t &byteStreamOut, uint8_t u8Cmd );
  int SendCommand(u8Array_t &byteStreamIn, uint8_t u8Cmd);
  bool GetValFromOutput(uint8_t cmd, uint8_t retcode, const u8Array_t &payload, int start_pos, int length, u8Array_t &res);

  u8Array_t GetCorrectionInfo();
  int GetCorrectionInfo(u8Array_t &dataOut);
  int GetFiretimesInfo(u8Array_t &dataOut);
  int GetChannelConfigInfo(u8Array_t &dataOut);
  int SetSocketTimeout(uint32_t u32RecMillisecond, uint32_t u32SendMillisecond);
  void CRCInit();
  uint32_t CRCCalc(uint8_t *bytes, int len); 

 public:
  uint32_t m_CRCTable[256];                                              

 private:
  static const std::string kLidarIPAddr;
  static const uint16_t kTcpPort = 9347;
  uint16_t m_u16PtcPort;
  bool running_;
  PtcMode client_mode_;
  uint8_t ptc_version_;
  std::shared_ptr<ClientBase> client_;
  std::shared_ptr<PtcParser> ptc_parser_;
};
}
}

#endif /* PtcClient_H */