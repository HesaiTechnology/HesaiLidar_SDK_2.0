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
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCH DAMAGE.
************************************************************************************************/

/*
 * File:       byte_parser.h
 * Author:     Jinghuan Xie <int_jinghuanxie@hesaitech.com>
 * Description: print bytes in uint8_t
 */

#ifndef BYTE_PRINTER_H_
#define BYTE_PRINTER_H_
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include "lidar_types.h"

class BytePrinter {
private:
  BytePrinter() {} // 私有化构造函数，防止外部实例化

public:
  static BytePrinter& getInstance() {
    static BytePrinter instance; // 单例模式，静态局部变量确保只创建一次
    return instance;
  }

  template <typename T>
  void printByte(const T &byte) {
    std::cout << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte);
  }

  void printByteArray(const hesai::lidar::u8Array_t &bytes) {
    for (const auto &byte : bytes) {
      std::cout << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte) << ' ';
    }
    std::cout << '\n';
  }

  template <typename T>
  std::string printByteToString(const T &byte) {
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte);
    return oss.str();
  }

  std::string printByteArrayToString(const hesai::lidar::u8Array_t &bytes) {
    std::ostringstream oss;
    for (const auto &byte : bytes) {
      oss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte) << ' ';
    }
    return oss.str();
  }
};

#endif