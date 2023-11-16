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
 * File:   ptc_client.cc
 * Author: Felix Zou<zouke@hesaitech.com>
 */

#include "ptc_client.h"

#include <plat_utils.h>
#ifdef _MSC_VER
#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT (0x40)
#endif
#else
#include <sched.h>
#include <sys/socket.h>
#endif

 
// #include "udp_protocol_v1_4.h"
// #include "udp_protocol_p40.h"
// #include "udp_protocol_p64.h"
// #include "udp_protocol_v4_3.h"
// #include "udp_protocol_v6_1.h"
using namespace hesai::lidar;
const std::string PtcClient::kLidarIPAddr("192.168.1.201");

PtcClient::PtcClient(std::string ip
                    , uint16_t u16TcpPort
                    , bool bAutoReceive
                    , PtcMode client_mode
                    , uint8_t ptc_version
                    , const char* cert
                    , const char* private_key
                    , const char* ca)
  : client_mode_(client_mode)
  , ptc_version_(ptc_version) {
  std::cout << "PtcClient::PtcClient()" << ip.c_str()
           << u16TcpPort << std::endl;
  // TcpClient::Open(ip, u16TcpPort);
  if(client_mode == PtcMode::tcp) {
    client_ = std::make_shared<TcpClient>();
    client_->Open(ip, u16TcpPort, bAutoReceive);
  } else if(client_mode == PtcMode::tcp_ssl) {
    client_ = std::make_shared<TcpSslClient>();
    client_->Open(ip, u16TcpPort, bAutoReceive, cert, private_key, ca);
  }

  // init ptc parser
  ptc_parser_ = std::make_shared<PtcParser>(ptc_version);

  CRCInit();
}

bool PtcClient::IsValidRsp(u8Array_t &byteStreamIn) {
  bool ret = true;
  // 根据ptc version来检查输入数据的前两位是否是对应版本的标记位
  while (byteStreamIn.size() >= 2 &&
         (byteStreamIn.at(0) != ptc_parser_->GetHeaderIdentifier0() ||
          byteStreamIn.at(1) != ptc_parser_->GetHeaderIdentifier1())) {
    byteStreamIn.erase(byteStreamIn.begin());
  }
  //输入数据长度小于头长度 没有数据
  if (byteStreamIn.size() < ptc_parser_->GetPtcParserHeaderSize()) ret = false;

  if (ret) {
    if(ptc_version_ == 1) {
      PTCHeader_1_0 *pHeader = (PTCHeader_1_0 *)byteStreamIn.data();
      if (!pHeader->IsValidReturnCode()) ret = false;
    } else {
      PTCHeader_2_0 *pHeader = (PTCHeader_2_0 *)byteStreamIn.data();
      if (!pHeader->IsValidReturnCode()) ret = false;
    }
  }
  return ret;
}

//接收数据  非阻塞模式
void PtcClient::TcpFlushIn() {
  u8Array_t u8Buf(1500, 0);
  int len = 0;
  do {
    len = client_->Receive(u8Buf.data(), u8Buf.size(), MSG_DONTWAIT);
    if (len > 0) {
      std::cout << "TcpFlushIn, len" << len << std::endl;
      for(int i = 0; i<u8Buf.size(); i++) {
        printf("%x ", u8Buf[i]);
      }
    }
  } while (len > 0);
}

int PtcClient::QueryCommand(u8Array_t &byteStreamIn,
                                       u8Array_t &byteStreamOut,
                                       uint8_t u8Cmd) {
  int ret = 0;
  u8Array_t encoded;
  uint32_t tick = GetMicroTickCount();
  ptc_parser_->PtcStreamEncode(byteStreamIn, encoded, u8Cmd);
  // TcpFlushIn();

  int nLen = client_->Send(encoded.data(), encoded.size());
  if (nLen != encoded.size()) {
    // qDebug("%s: send failure, %d.", __func__, nLen);
    ret = -1;
  }

  //开始接收数据
  int nOnceRecvLen = 0;
  u8Array_t u8RecvHeaderBuf(ptc_parser_->GetPtcParserHeaderSize());
  u8Array_t::pointer pRecvHeaderBuf = u8RecvHeaderBuf.data();
  u8Array_t u8HeaderBuf(ptc_parser_->GetPtcParserHeaderSize());
  u8Array_t::pointer pHeaderBuf = u8HeaderBuf.data();
  //还要接收的数据长度
  int nLeft = ptc_parser_->GetPtcParserHeaderSize();
  //是否已经找到连续的0x47和0x74
  bool bHeaderFound = false;
  int nValidDataLen = 0;
  int nPayLoadLen = 0;

  if (ret == 0) {
    while (nLeft > 0) {
      nOnceRecvLen = client_->Receive(pRecvHeaderBuf, nLeft);
      if (nOnceRecvLen <= 0) break;

      if (!bHeaderFound) {
        for (int i = 0; i < nOnceRecvLen - 1; i++) {
          if (pRecvHeaderBuf[i] == ptc_parser_->GetHeaderIdentifier0() &&
              pRecvHeaderBuf[i + 1] == ptc_parser_->GetHeaderIdentifier1()) {
            nValidDataLen = nOnceRecvLen - i;
            bHeaderFound = true;
            memcpy(pHeaderBuf, pRecvHeaderBuf + i, nValidDataLen);
            //剩下还需接收的长度
            nLeft -= nValidDataLen;
            break;
          }
        }
      } else {
        //已经收到PTC正确的头部 只是还没有达到8位
        memcpy(pHeaderBuf + nValidDataLen, pRecvHeaderBuf, nOnceRecvLen);
        nValidDataLen += nOnceRecvLen;
        nLeft -= nOnceRecvLen;
      }
    }
    //因超时退出而导致数据未接收完全
    if (nLeft > 0) {
      // std::cout << "PtcClient::QueryCommand, invalid Header, nLeft:"
      //          << nLeft << ", u8HeaderBuf:" << std::hex << u8HeaderBuf;
      ret = -1;
    } else {
      //开始接收body
      if(ptc_version_ == 1) {
        PTCHeader_1_0 *pPTCHeader = reinterpret_cast<PTCHeader_1_0 *>(u8HeaderBuf.data());
        if (!pPTCHeader->IsValidReturnCode() || pPTCHeader->cmd_ != u8Cmd) {
          std::cout << "PtcClient::QueryCommand,RspCode invalid:" << pPTCHeader->return_code_ << ", u8HeaderBuf: " << std::endl;
          ret = -1;
        } else {
          nPayLoadLen = pPTCHeader->GetPayloadLen();
        }
      } else {
        PTCHeader_2_0 *pPTCHeader = reinterpret_cast<PTCHeader_2_0 *>(u8HeaderBuf.data());
        if (!pPTCHeader->IsValidReturnCode() || pPTCHeader->cmd_ != u8Cmd) {
          std::cout << "PtcClient::QueryCommand,RspCode invalid:" << pPTCHeader->return_code_ << ", u8HeaderBuf: " << std::endl;
          ret = -1;
        } else {
          nPayLoadLen = pPTCHeader->GetPayloadLen();
        }
      }
    }
  }
  // std::cout << "nPayLoadLen = " << nPayLoadLen << std::endl;
  if (ret == 0 && nPayLoadLen > 0) {
    //对payLoad进行一个判断，避免负载过长申请过大的空间 目前指定为10K
    const int kMaxPayloadBuffSize = 10240;
    if (nPayLoadLen > kMaxPayloadBuffSize)
      std::cout << "PtcClient::QueryCommand, warning, nPayLoadLen too large:" << nPayLoadLen << std::endl;

    // tmp code to allow LiDAR bug
    // const int kExtraBufLen = 4;
    // u8Array_t u8BodyBuf(nPayLoadLen + kExtraBufLen);
    u8Array_t u8BodyBuf(nPayLoadLen);
    u8Array_t::pointer pBodyBuf = u8BodyBuf.data();

    nLeft = nPayLoadLen;
    while (nLeft > 0) {
      nOnceRecvLen = client_->Receive(pBodyBuf, nLeft);
      if (nOnceRecvLen <= 0) {
        break;
      }

      nLeft -= nOnceRecvLen;
      pBodyBuf += nOnceRecvLen;
    }

    if (nLeft > 0) {
      // std::cout << "PtcClient::QueryCommand,Body "
      //           <<"incomplete:nPayLoadLen:"
      //           << nPayLoadLen << " nLeft:" << nLeft << std::endl;
      ret = -1;
    } else {
      //将收到的bodyBuf拷贝到最终要传出的buf
      byteStreamOut.resize(nPayLoadLen);
      pBodyBuf = u8BodyBuf.data();
      memcpy(byteStreamOut.data(), pBodyBuf, nPayLoadLen);
    }
  }

  // std::cout << "PtcClient::QueryCommand,rsp, header:" << std::hex
  //          << u8HeaderBuf << "byteStreamOut:" << byteStreamOut;

  printf("exec time: %fms\n", (GetMicroTickCount() - tick) / 1000.f);

  return ret;
}

int PtcClient::SendCommand(u8Array_t &byteStreamIn, uint8_t u8Cmd) {
  // std::cout << "PtcClient::SendCommand, cmd:" << u8Cmd
  //          << "data:" << std::hex << byteStreamIn;

  u8Array_t byteStreamOut;

  return QueryCommand(byteStreamIn, byteStreamOut, u8Cmd);
}

bool PtcClient::GetValFromOutput(uint8_t cmd, uint8_t retcode, const u8Array_t &payload, int start_pos, int length, u8Array_t& res) {
  return ptc_parser_->PtcStreamDecode(cmd, retcode, payload, start_pos, length, res);
}

u8Array_t PtcClient::GetCorrectionInfo() {
  u8Array_t dataIn;
  u8Array_t dataOut;

  int ret = -1;

  ret = this->QueryCommand(dataIn, dataOut,
                           kPTCGetLidarCalibration);

  // ret = this->QueryCommand(dataIn, dataOut,
  //                          PtcClient::kPTCGetInventoryInfo);

  if (ret == 0 && !dataOut.empty()) {
    std::cout << "Read correction file from lidar success" << std::endl;
  } else {
    std::cout << "Read correction file from lidar fail" << std::endl;
  }

  return dataOut;
}

int PtcClient::GetCorrectionInfo(u8Array_t &dataOut) {
  u8Array_t dataIn;
  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut,
                           kPTCGetLidarCalibration);

  if (ret == 0 && !dataOut.empty()) {
    return 0;
  } else {
    return -1;
  }
}

int PtcClient::GetFiretimesInfo(u8Array_t &dataOut) {
  u8Array_t dataIn;

  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut,
                           kPTCGetLidarFiretimes);
  if (ret == 0 && !dataOut.empty()) {
    return 0;
  } else {
    return -1;
  }
}

int PtcClient::GetChannelConfigInfo(u8Array_t &dataOut) {
  u8Array_t dataIn;

  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut,
                           kPTCGetLidarChannelConfig);
  if (ret == 0 && !dataOut.empty()) {
    return 0;
  } else {
    return -1;
  }
}

int PtcClient::SetSocketTimeout(uint32_t u32RecMillisecond,
                                           uint32_t u32SendMillisecond) {
  return client_->SetTimeout(u32RecMillisecond, u32SendMillisecond);
}


void PtcClient::CRCInit() {
  uint32_t i, j, k;

  for (i = 0; i < 256; i++) {
    k = 0;
    for (j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1)
      k = (k << 1) ^ (((k ^ j) & 0x80000000) ? 0x04c11db7 : 0);

    m_CRCTable[i] = k;
  }
}

uint32_t PtcClient::CRCCalc(uint8_t *bytes, int len) {
  uint32_t i_crc = 0xffffffff;
  int i = 0;
  for (i = 0; (size_t)i < len; i++)
    i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ bytes[i]) & 0xff];
  return i_crc;
}
