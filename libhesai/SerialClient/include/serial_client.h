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
 * File:   serial_client.h
 * Author: chang xingshuo<changxingshuo@hesaitech.com>
 *
 * Created on Sep 10, 2024, 19:56 PM  
 */

#ifndef SERIAL_CLIENT_H
#define SERIAL_CLIENT_H

#include "serial_source.h"
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <random>
#include <algorithm>
#include <functional>
#include <memory>
#include <string.h>

namespace hesai
{
namespace lidar
{

enum ErrorCode {
  kInvalidEquipment  = 1,
  kInvalidData       = 2,
  kFileNotExit       = 3,
  kInvalidCmd        = 4,
  kTimeout           = 5,
  kExecutionError    = 6,
  kSwipeError        = 7,
  kDataError         = 8,
  kPackCrcError      = 9,
  kUndefine          = 10,
  kPacketLoss        = 11,
  kChecksumFailure   = 12,
  kUnrecognisedFormat= 13,
  kReadTimeout       = 14,
  kLengthError       = 15,
  kIdDiscontinuity   = 16,
  kFailedCalibration = 11,
};

enum CmdType {
  kCmd = 1,
  kOta = 2,
};

typedef std::vector<uint8_t> u8Array_t;
#pragma pack(push, 1)
struct SerialHeader {
  uint8_t start_flag_[7];    
  
  SerialHeader() {
    start_flag_[0] = 0;
    start_flag_[1] = 0;
    start_flag_[2] = 0;
    start_flag_[3] = 0;
    start_flag_[4] = 0;
    start_flag_[5] = 0;
    start_flag_[6] = 0;
  }
  void InitCmd() {
    start_flag_[0] = 0x24;
    start_flag_[1] = 0x4C;
    start_flag_[2] = 0x44;
    start_flag_[3] = 0x43;
    start_flag_[4] = 0x4D;
    start_flag_[5] = 0x44;
    start_flag_[6] = 0x2C;
  }
  void InitOta() {
    start_flag_[0] = 0x24;
    start_flag_[1] = 0x4C;
    start_flag_[2] = 0x44;
    start_flag_[3] = 0x4F;
    start_flag_[4] = 0x54;
    start_flag_[5] = 0x41;
    start_flag_[6] = 0x2C;
  }
};
#pragma pack(pop)
class SerialClient {
 public:
  SerialClient();
  ~SerialClient();

  SerialClient(const SerialClient &orig) = delete;
  int QueryCommand(const uint8_t cmd, const u8Array_t &payload, u8Array_t &byteStreamOut, uint32_t timeout);
  void SetSerial(Source* source_send, Source* source_recv);

  int ChangeUpgradeMode();
  int GetCorrectionInfo(u8Array_t &dataOut);
  int GetSnInfo(u8Array_t &dataOut);
  static const uint16_t crc_begin = 7;
  uint32_t CRCCalc(const uint8_t *bytes, int len, int zeros_num);

  uint32_t m_CRCTable[256];  
 protected:
  void AddEndStreamEncode(u8Array_t &byteStreamOut, const CmdType type);
  void SerialStreamEncode(const CmdType type, u8Array_t &byteStream);
  bool SerialStreamDecode(const CmdType type, const u8Array_t &byteStreamIn, u8Array_t &byteStreamOut);
  void CRCInit();
  uint32_t GetRandom();
  int CmdErrorCode2RetCode(uint8_t error_code);

  Source* source_send_;  
  Source* source_recv_;  
};

}
}


#endif