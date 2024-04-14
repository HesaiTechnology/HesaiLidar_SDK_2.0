/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File:       ptc_1_0_parser.h
 * Author:     Zhang Xu <int_zhangxu@hesaitech.com>
 * Description: ptc 1.0 protocol parser class.
 */

#ifndef PTC_1_0_PARSER_H_
#define PTC_1_0_PARSER_H_
#ifdef _MSC_VER
#endif
#include <iostream>
#include <fstream>
#include "general_ptc_parser.h"
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
struct PTCHeader_1_0 {
  uint8_t identifier0_;     // protocol identifier
  uint8_t identifier1_;     // protocol identifier
  uint8_t cmd_;             // command code
  uint8_t return_code_;     // return code
  uint32_t payload_len_;    // length of payload for the command
  
  PTCHeader_1_0() {
    identifier0_ = 0x47;
    identifier1_ = 0x74;
    return_code_ = 0;
    payload_len_ = 0;
  }
  void init(uint8_t cmd) {
    identifier0_ = 0x47;
    identifier1_ = 0x74;
    cmd_ = cmd;
    return_code_ = 0;
    payload_len_ = 0;
  }

  bool IsValidIdentifier() const { return identifier0_ == 0x47 && identifier1_ == 0x74; }
  bool IsValidReturnCode() const { return IsValidIdentifier() && return_code_ == 0; }
  uint8_t GetIdentifier0() { return identifier0_; }
  uint8_t GetIdentifier1() { return identifier1_; }
  uint8_t GetReturnCode() const { return return_code_; }
  uint8_t GetCmd() const {return cmd_; }
  uint32_t GetPayloadLen() const { return ntohl(payload_len_); }
  void SetPayloadLen(uint32_t u32Len) { payload_len_ = htonl(u32Len); }


} PACKED;
#ifdef _MSC_VER
#pragma pack(pop)
#endif

class Ptc_1_0_parser : public GeneralPtcParser {
public:
  ~Ptc_1_0_parser(){};
  // 字节流的打包。
  // 因为要将header和payload进行组装
  bool PtcStreamEncode(const u8Array_t &payload, u8Array_t &byteStreamOut, uint8_t u8Cmd);

  // 字节流的解析（拆包）
  bool PtcStreamDecode(uint8_t cmd, uint8_t retcode, const u8Array_t &payload, int start_pos, int length, u8Array_t &res);

  uint8_t GetHeaderIdentifier0() { return header_.identifier0_; }
  uint8_t GetHeaderIdentifier1() { return header_.identifier1_; }
  int GetHeaderSize() { return sizeof(PTCHeader_1_0); }
  uint8_t GetHeaderReturnCode() const { return header_.return_code_; }
  uint8_t GetHeaderCmd() const {return header_.cmd_; }
  uint32_t GetHeaderPayloadLen() const { return header_.GetPayloadLen();}

private:
  PTCHeader_1_0 header_;
};
}
}
#endif