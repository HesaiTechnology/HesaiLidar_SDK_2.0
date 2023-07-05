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
 * File:   ptc_client.cc
 * Author: Felix Zou<zouke@hesaitech.com>
 */

#include "ptc_client.h"

#include <plat_utils.h>
#include <sched.h>
#include <sys/socket.h>

 
#include "udp_protocol_v1_4.h"
#include "udp_protocol_p40.h"
#include "udp_protocol_p64.h"
#include "udp_protocol_v4_3.h"
#include "udp_protocol_v6_1.h"
using namespace hesai::lidar;
const std::string PtcClient::kLidarIPAddr("192.168.1.201");

PtcClient::PtcClient(std::string ip, uint16_t u16TcpPort) {
  std::cout << "PtcClient::PtcClient()" << ip.c_str()
           << u16TcpPort << std::endl;
  TcpClient::Open(ip, u16TcpPort);
  CRCInit();
}

bool PtcClient::PTCEncode(u8Array_t &byteStreamIn,
                                     u8Array_t &byteStreamOut, uint8_t u8Cmd) {
  //设置byteStreamOut长度
  byteStreamOut.resize(sizeof(PTCHeader) + byteStreamIn.size());
  PTCHeader *pHeader = (PTCHeader *)byteStreamOut.data();
  pHeader->Init(u8Cmd);
  pHeader->SetPayloadLen(byteStreamIn.size());
  pHeader++;

  memcpy((void *)pHeader, byteStreamIn.data(), byteStreamIn.size());
  return true;
}

bool PtcClient::PTCDecode(u8Array_t &byteStreamIn,
                                     u8Array_t &byteStreamOut) {
  bool ret = false;

  if (IsValidRsp(byteStreamIn)) {
    byteStreamOut.resize(byteStreamIn.size() - sizeof(PTCHeader));
    PTCHeader *pHeader = (PTCHeader *)byteStreamIn.data();
    if (byteStreamIn.size() - sizeof(PTCHeader) >= pHeader->GetPayloadLen()) {
      if (pHeader->GetPayloadLen() > 0)
        //将收到的数据加上头部都复制给byteStreamOut
        memcpy(byteStreamOut.data(), byteStreamIn.data() + sizeof(PTCHeader),
               byteStreamIn.size() - sizeof(PTCHeader));
      ret = true;
    } else {
      printf("PTCDecode(), failed, rspcode %u, len: 0x%x/0x%lx\n",
             pHeader->m_u8RspCode, pHeader->GetPayloadLen(),
             byteStreamIn.size() - sizeof(PTCHeader));
    }
  }
  return ret;
}

bool PtcClient::IsValidRsp(u8Array_t &byteStreamIn) {
  bool ret = true;
  //用于检查输入数据的前两位是否是标记位  0x47和0x74
  while (byteStreamIn.size() >= 2 &&
         (byteStreamIn.at(0) != PTCHeader::Identifier0() ||
          byteStreamIn.at(1) != PTCHeader::Identifier1())) {
    byteStreamIn.erase(byteStreamIn.begin());
  }
  //输入数据长度小于头长度 没有数据
  if (byteStreamIn.size() < sizeof(PTCHeader)) ret = false;

  if (ret) {
    PTCHeader *pHeader = (PTCHeader *)byteStreamIn.data();
    if (!pHeader->IsValidRsp()) ret = false;
  }

  return ret;
}

//接收数据  非阻塞模式
void PtcClient::TcpFlushIn() {
  u8Array_t u8Buf(1500, 0);
  int len = 0;
  do {
    len = TcpClient::Receive(u8Buf.data(), u8Buf.size(), MSG_DONTWAIT);
    if (len > 0) std::cout << "TcpFlushIn, len" << len << std::endl;
  } while (len > 0);
}

int PtcClient::QueryCommand(u8Array_t &byteStreamIn,
                                       u8Array_t &byteStreamOut,
                                       uint8_t u8Cmd) {
  int ret = 0;
  u8Array_t encoded;

  uint32_t tick = GetMicroTickCount();

  PTCEncode(byteStreamIn, encoded, u8Cmd);
  // std::cout << "PtcClient::QueryCommand, encoded:" << encoded;

  TcpFlushIn();

  int nLen = TcpClient::Send(encoded.data(), encoded.size());
  if (nLen != encoded.size()) {
    // qDebug("%s: send failure, %d.", __func__, nLen);
    ret = -1;
  }

  //开始接收数据
  int nOnceRecvLen = 0;
  u8Array_t u8RecvHeaderBuf(sizeof(PTCHeader));
  u8Array_t::pointer pRecvHeaderBuf = u8RecvHeaderBuf.data();
  u8Array_t u8HeaderBuf(sizeof(PTCHeader));
  u8Array_t::pointer pHeaderBuf = u8HeaderBuf.data();
  //还要接收的数据长度
  int nLeft = sizeof(PTCHeader);
  //是否已经找到连续的0x47和0x74
  bool bHeaderFound = false;
  int nValidDataLen = 0;
  int nPayLoadLen = 0;

  if (ret == 0) {
    while (nLeft > 0) {
      nOnceRecvLen = TcpClient::Receive(pRecvHeaderBuf, nLeft);
      if (nOnceRecvLen <= 0) break;

      if (!bHeaderFound) {
        for (int i = 0; i < nOnceRecvLen - 1; i++) {
          if (pRecvHeaderBuf[i] == PTCHeader::Identifier0() &&
              pRecvHeaderBuf[i + 1] == PTCHeader::Identifier1()) {
            nValidDataLen = nOnceRecvLen - i;
            bHeaderFound = true;
            std::memcpy(pHeaderBuf, pRecvHeaderBuf + i, nValidDataLen);
            //剩下还需接收的长度
            nLeft -= nValidDataLen;
            break;
          }
        }
      } else {
        //已经收到PTC正确的头部 只是还没有达到8位
        std::memcpy(pHeaderBuf + nValidDataLen, pRecvHeaderBuf, nOnceRecvLen);
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
      PTCHeader *pPTCHeader = reinterpret_cast<PTCHeader *>(u8HeaderBuf.data());
      if (!pPTCHeader->IsValidRsp() || pPTCHeader->m_u8Cmd != u8Cmd) {
        // std::cout << "PtcClient::QueryCommand,RspCode invalid:"
        //          << pPTCHeader->GetRspCode() << ", u8HeaderBuf: " << std::hex
        //          << u8HeaderBuf;
        ret = -1;
      } else {
        nPayLoadLen = pPTCHeader->GetPayloadLen();
      }
    }
  }

  if (ret == 0 && nPayLoadLen > 0) {
    //对payLoad进行一个判断，避免负载过长申请过大的空间 目前指定为10K
    const int kMaxPayloadBuffSize = 10240;
    if (nPayLoadLen > kMaxPayloadBuffSize)
      std::cout << "PtcClient::QueryCommand, warning, nPayLoadLen "
                    "too large:"
                 << nPayLoadLen << std::endl;

    // tmp code to allow LiDAR bug
    const int kExtraBufLen = 4;
    u8Array_t u8BodyBuf(nPayLoadLen + kExtraBufLen);
    u8Array_t::pointer pBodyBuf = u8BodyBuf.data();

    nLeft = nPayLoadLen;

    while (nLeft > 0) {
      nOnceRecvLen = TcpClient::Receive(pBodyBuf, nLeft + kExtraBufLen);

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
      std::memcpy(byteStreamOut.data(), pBodyBuf, nPayLoadLen);
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


u8Array_t PtcClient::GetCorrectionInfo() {
  u8Array_t dataIn;
  u8Array_t dataOut;

  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut,
                           PtcClient::kPTCGetLidarCalibration);

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
                           PtcClient::kPTCGetLidarCalibration);
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
                           PtcClient::kPTCGetLidarFiretimes);
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
                           PtcClient::kPTCGetLidarChannelConfig);
  if (ret == 0 && !dataOut.empty()) {
    return 0;
  } else {
    return -1;
  }
}

int PtcClient::SetSocketTimeout(uint32_t u32RecMillisecond,
                                           uint32_t u32SendMillisecond) {
  return TcpClient::SetTimeout(u32RecMillisecond, u32SendMillisecond);
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
