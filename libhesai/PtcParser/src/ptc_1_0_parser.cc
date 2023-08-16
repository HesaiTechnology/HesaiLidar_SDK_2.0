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
 * File:       ptc_1_0_parser.cc
 * Author:     Zhang Xu <int_zhangxu@hesaitech.com>
 */
#include "ptc_1_0_parser.h"

using namespace hesai::lidar;

// 字节流的打包
// 对只需要一个包的payload进行封装成包
bool Ptc_1_0_parser::PtcStreamEncode(const u8Array_t &payload, u8Array_t &byteStreamOut, uint8_t u8Cmd) {
  byteStreamOut.resize(sizeof(PTCHeader_1_0) + payload.size()); //设置byteStreamOut长度
  // std::cout << sizeof(PTCHeader_1_0) << std::endl;
  PTCHeader_1_0 *pHeader = (PTCHeader_1_0 *)byteStreamOut.data();
  // std::cout << "pHeader's identifier0 : " << pHeader->GetIdentifier0() << std::endl;
  // std::cout << "pHeader's identifier1 : " << pHeader->GetIdentifier1() << std::endl;
  // std::cout << "pHeader's payload length : " << pHeader->GetPayloadLen() << std::endl;
  // std::cout << "-------Ptc_1_0_parser::PtcStreamEncode()" << std::endl;
  pHeader->init(u8Cmd);
  pHeader->SetPayloadLen(payload.size());
  pHeader++;

  memcpy((void *)pHeader, payload.data(), payload.size());
  return true;
}

// 字节流的解析（拆包）
// 根据协议解析字节流，并且取出对应的结果
// 给定所要取的值在payload中的起始位置和长度，将值取出赋给res
bool Ptc_1_0_parser::PtcStreamDecode(uint8_t cmd, uint8_t retcode, const u8Array_t &payload, int start_pos, int length, u8Array_t &res) {
  // PTCHeader_1_0 *header = dynamic_cast<PTCHeader_1_0*>(base_header);
  // if(header == nullptr) {
  //   std::cout << "Ptc_1_0_parser::PtcStreamDecode failed! header is not a 1.0 version!" << std::endl;
  //   return false;
  // }
  std::cout << length << " " << payload.size() << std::endl;
  res = u8Array_t(payload.begin() + start_pos, payload.begin() + start_pos + length);

  // for(int i = 0; i < res.size(); i++) {
  //   printf("%c", char(res[i]));
  // }

  std::cout << std::endl << "Ptc_1_0_parser::PtcStreamDecode success!" << std::endl;
  return true;
}