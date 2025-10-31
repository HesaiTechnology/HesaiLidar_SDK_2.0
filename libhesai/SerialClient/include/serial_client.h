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
#include "inner_com.h"
#include "byte_printer.h"

namespace hesai
{
namespace lidar
{

enum ErrorCode {
  kInvalidEquipment  = -1,
  kInvalidData       = -2,
  kReadTimeout       = -3,
  kInvalidDataHeader = -4,
  kSerialOpenError   = -5,
  kInPblNotUpgrade   = -6,
  kFailedCalibration = -7,
};

enum CmdType {
  kCmd = 1,
  kOta = 2,
};

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
  using CallbackType = std::function<void(const std::string&)>; // 函数指针 std::function
  SerialClient();
  ~SerialClient();

  SerialClient(const SerialClient &orig) = delete;
  int QueryCommand(const uint8_t cmd, const u8Array_t &payload, u8Array_t &byteStreamOut, uint32_t timeout);
  int RecvSpecialAckData(uint8_t &status, uint8_t &ret_code, int timeout);
  void SetSerial(Source* source_send, Source* source_recv);
  int ChangeUpgradeMode();
  int ChangeMode(uint8_t mode, uint8_t reserved = 0x00);
  int GetCorrectionInfo(u8Array_t &dataOut);
  int GetSnInfo(u8Array_t &dataOut);
  int GetLidarVersion(u8Array_t &dataOut, uint8_t type);
  int GetLidarFaultState(u8Array_t &dataOut);
  int GetPblVersionIdInPbl(u8Array_t &byteStreamOut);
  int RequestUpgradeLargePackage(uint8_t type);
  int OtaQueryCommand(const uint32_t all_num, const uint32_t num, const uint32_t len, const uint8_t *payload, uint8_t &status, uint8_t &ret_code, uint8_t type);
  static const uint16_t crc_begin = 7;
  uint32_t CRCCalc(const uint8_t *bytes, int len, int zeros_num);
  void SetCallback(CallbackType callback) { this->log_message_handler_callback_ = callback; }
  void ProduceLogMessage(const std::string& message); 

  uint32_t m_CRCTable[256];  
  uint8_t m_now_mode = 0;
  int UpgradeLidar(u8Array_t data, int mode, int &upgrade_progress);
 protected:
  void AddEndStreamEncode(u8Array_t &byteStreamOut, const CmdType type);
  void SerialStreamEncode(const CmdType type, u8Array_t &byteStream);
  bool SerialStreamDecode(const CmdType type, const u8Array_t &byteStreamIn, u8Array_t &byteStreamOut);
  void CRCInit();
  uint32_t GetRandom();

  CallbackType log_message_handler_callback_ = nullptr;
  Source* source_send_;  
  Source* source_recv_;  
};

}
}


#endif