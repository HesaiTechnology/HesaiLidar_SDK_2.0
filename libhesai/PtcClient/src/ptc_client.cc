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
#include <chrono>
#include <thread>
#include <iostream>
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

PtcClient::~PtcClient() {
  InitOpen = false;
  if (nullptr != open_thread_ptr_) {
    open_thread_ptr_->join();
    delete open_thread_ptr_;
    open_thread_ptr_ = nullptr;
  } 
}

PtcClient::PtcClient(std::string ip
                    , uint16_t u16TcpPort
                    , bool bAutoReceive
                    , PtcMode client_mode
                    , uint8_t ptc_version
                    , const char* cert
                    , const char* private_key
                    , const char* ca
                    , uint32_t u32RecvTimeoutMs
                    , uint32_t u32SendTimeoutMs)
  : client_mode_(client_mode)
  , ptc_version_(ptc_version) {
  lidar_ip_ = ip;
  tcp_port_ =  u16TcpPort;
  auto_receive_ = bAutoReceive;
  cert_  = cert;
  private_key_ = private_key;
  ca_ = ca;
  recv_timeout_ms_ = u32RecvTimeoutMs;
  send_timeout_ms_ = u32SendTimeoutMs;

  open_thread_ptr_ = new std::thread(std::bind(&PtcClient::TryOpen, this));
  // init ptc parser
  ptc_parser_ = std::make_shared<PtcParser>(ptc_version);

  CRCInit();
}

void PtcClient::TryOpen() {
  LogInfo("PtcClient::PtcClient() %s %u", lidar_ip_.c_str(), tcp_port_);
  if(client_mode_ == PtcMode::tcp) {
    client_ = std::make_shared<TcpClient>();
    while (InitOpen && !client_->TryOpen(lidar_ip_, tcp_port_, auto_receive_)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } else if(client_mode_ == PtcMode::tcp_ssl) {
    client_ = std::make_shared<TcpSslClient>();
    while(InitOpen && !client_->TryOpen(lidar_ip_, tcp_port_, auto_receive_, cert_, private_key_, ca_)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  if (recv_timeout_ms_ != 0 && send_timeout_ms_ != 0) {
    LogInfo("u32RecvTimeoutMs %u", recv_timeout_ms_);
    LogInfo("u32SendTimeoutMs %u", send_timeout_ms_);    
    client_->SetTimeout(recv_timeout_ms_, send_timeout_ms_);
  }
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
  if (byteStreamIn.size() < (size_t)ptc_parser_->GetPtcParserHeaderSize()) ret = false;

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
    len = client_->Receive(u8Buf.data(), static_cast<uint32_t>(u8Buf.size()), MSG_DONTWAIT);
    if (len > 0) {
      std::cout << "TcpFlushIn, len " << len << std::endl;
      for(size_t i = 0; i<u8Buf.size(); i++) {
        printf("%x ", u8Buf[i]);
      }
    }
  } while (len > 0);
}

int PtcClient::QueryCommand(u8Array_t &byteStreamIn,
                                       u8Array_t &byteStreamOut,
                                       uint8_t u8Cmd) {
  LockS lock(_mutex);
  int ret = 0;
  u8Array_t encoded;
  uint32_t tick = GetMicroTickCount();
  ptc_parser_->PtcStreamEncode(byteStreamIn, encoded, u8Cmd);
  // TcpFlushIn();

  int nLen = client_->Send(encoded.data(), static_cast<uint16_t>(encoded.size()));
  if (nLen != (int)encoded.size()) {
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
  int nPayLoadLen = 0;

  if (ret == 0) {
    //是否已经找到连续的0x47和0x74
    bool bHeaderFound = false;
    int nValidDataLen = 0;
    while (nLeft > 0) {
      nOnceRecvLen = client_->Receive(pRecvHeaderBuf, static_cast<uint32_t>(nLeft));
      if (nOnceRecvLen <= 0) break;

      if (!bHeaderFound) {
        for (int i = 0; i < nOnceRecvLen - 1; i++) {
          if (pRecvHeaderBuf[i] == ptc_parser_->GetHeaderIdentifier0() &&
              pRecvHeaderBuf[i + 1] == ptc_parser_->GetHeaderIdentifier1()) {
            nValidDataLen = nOnceRecvLen - i;
            bHeaderFound = true;
            memcpy(pHeaderBuf, pRecvHeaderBuf + i, static_cast<size_t>(nValidDataLen));
            //剩下还需接收的长度
            nLeft -= nValidDataLen;
            break;
          }
        }
      } else {
        //已经收到PTC正确的头部 只是还没有达到8位
        memcpy(pHeaderBuf + nValidDataLen, pRecvHeaderBuf, static_cast<size_t>(nOnceRecvLen));
        nValidDataLen += nOnceRecvLen;
        nLeft -= nOnceRecvLen;
      }
    }
    //因超时退出而导致数据未接收完全
    if (nLeft > 0) {
      ret = -1;
    } else {
      //开始接收body
      if(ptc_version_ == 1) {
        PTCHeader_1_0 *pPTCHeader = reinterpret_cast<PTCHeader_1_0 *>(u8HeaderBuf.data());
        if (!pPTCHeader->IsValidReturnCode() || pPTCHeader->cmd_ != u8Cmd) {
          LogWarning("PtcClient::QueryCommand, RspCode invalid:%u", pPTCHeader->return_code_);
          ret = -1;
        } else {
          nPayLoadLen = static_cast<int>(pPTCHeader->GetPayloadLen());
        }
      } else {
        PTCHeader_2_0 *pPTCHeader = reinterpret_cast<PTCHeader_2_0 *>(u8HeaderBuf.data());
        if (!pPTCHeader->IsValidReturnCode() || pPTCHeader->cmd_ != u8Cmd) {
          LogWarning("PtcClient::QueryCommand,RspCode invalid:%u", pPTCHeader->return_code_);
          ret = -1;
        } else {
          nPayLoadLen = static_cast<int>(pPTCHeader->GetPayloadLen());
        }
      }
    }
  }
  if (ret == 0 && nPayLoadLen > 0) {
    //对payLoad进行一个判断，避免负载过长申请过大的空间 目前指定为10K
    const int kMaxPayloadBuffSize = 10240;
    if (nPayLoadLen > kMaxPayloadBuffSize)
      LogWarning("PtcClient::QueryCommand, warning, nPayLoadLen too large:%d", nPayLoadLen);

    // tmp code to allow LiDAR bug
    // const int kExtraBufLen = 4;
    // u8Array_t u8BodyBuf(nPayLoadLen + kExtraBufLen);
    u8Array_t u8BodyBuf(nPayLoadLen);
    u8Array_t::pointer pBodyBuf = u8BodyBuf.data();

    nLeft = nPayLoadLen;
    while (nLeft > 0) {
      nOnceRecvLen = client_->Receive(pBodyBuf, static_cast<uint32_t>(nLeft));
      if (nOnceRecvLen <= 0) {
        break;
      }

      nLeft -= nOnceRecvLen;
      pBodyBuf += nOnceRecvLen;
    }

    if (nLeft > 0) {
      ret = -1;
    } else {
      //将收到的bodyBuf拷贝到最终要传出的buf
      byteStreamOut.resize(nPayLoadLen);
      pBodyBuf = u8BodyBuf.data();
      memcpy(byteStreamOut.data(), pBodyBuf, static_cast<size_t>(nPayLoadLen));
    }
  }

  LogDebug("exec time: %fms", (GetMicroTickCount() - tick) / 1000.f);

  return ret;
}

int PtcClient::SendCommand(u8Array_t &byteStreamIn, uint8_t u8Cmd) {

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

  if (ret == 0 && !dataOut.empty()) {
    LogInfo("Read correction file from lidar success");
  } else {
    LogWarning("Read correction file from lidar fail");
  }

  return dataOut;
}

template <typename T>
T extractField(const u8Array_t& data, size_t& offset) {
    T field = 0;
    for (size_t i = 0; i < sizeof(T); ++ i) {
        field = static_cast<T>((field << 8) | data[offset++]);
    }
    return field;
}

int PtcClient::GetPTPDiagnostics (u8Array_t &dataOut, uint8_t query_type) {
  u8Array_t dataIn;
  dataIn.push_back(query_type);
  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut,
                           kPTCGetPTPDiagnostics);

  if (ret == 0 && !dataOut.empty()) {
    return 0;
  } else {
    return -1;
  }
}

int PtcClient::GetPTPLockOffset(u8Array_t &dataOut)
{
  u8Array_t dataIn;
  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut,
                           kPTCGetPTPLockOffset);
  if (ret == 0 && !dataOut.empty()) {
    return 0;
  } else {
    return -1;
  }
}

int PtcClient::GetLidarStatus() {
  u8Array_t dataIn, dataOut;
  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut, 
                           kPTCGetLidarStatus);
  if (ret == 0 && !dataOut.empty()) {
    // according XT32M1X_TCP_API.pdf
    uint32_t systemp_uptime;
    uint16_t motor_speed;
    uint32_t temperature[8];
    uint8_t gps_pps_lock;
    uint8_t gps_gprmc_status;
    uint32_t startup_times;
    uint32_t total_operation_time;
    uint8_t ptp_status;

    size_t offset = 0;
    systemp_uptime = extractField<uint32_t>(dataOut, offset);
    motor_speed = extractField<uint16_t>(dataOut, offset);
    for (int i = 0; i < 8; i ++ ) {
      temperature[i] = extractField<uint32_t>(dataOut, offset);
    }
    gps_pps_lock = extractField<uint8_t>(dataOut, offset);
    gps_gprmc_status = extractField<uint8_t>(dataOut, offset);
    startup_times = extractField<uint32_t>(dataOut, offset);
    total_operation_time = extractField<uint32_t>(dataOut, offset);
    ptp_status = extractField<uint8_t>(dataOut, offset);
    printf("System uptime: %u second, Real-time motor speed: %u RPM\n"
           "----------Temperature(0.01 Celsius)-----------\n"
           "Bottom circuit board T1: %u\n"
           "Bottom circuit board T2: %u\n"
           "Laser emitting board RT_L1: %u\n"
           "Laser emitting board RT_L2: %u\n"
           "Laser Receiving board RT_R: %d\n"
           "Laser Receiving board RT2: %d\n"
           "Top circuit RT3: %u\n"
           "Top circuit RT4: %u\n"
           "GPS PPS status: %u, GPS NMEA status: %u\n"
           "System start-up times: %u, Total time in operation: %u\n"
           "PTP status: %u\n"
    , systemp_uptime, motor_speed, temperature[0], temperature[1],temperature[2], temperature[3],
    temperature[4], temperature[5], temperature[6], temperature[7], gps_pps_lock, gps_gprmc_status,
    startup_times, total_operation_time, ptp_status);
    printf("Lidar Status Size: %zu\n", offset);
    return 0;
  } else {
    return -1;
  }
}

int PtcClient::GetCorrectionInfo(u8Array_t &dataOut) {
  u8Array_t dataIn;
  int ret = -1;
  ret = this->QueryCommand(dataIn, dataOut,
                           kPTCGetLidarCalibration);

  if (ret == 0 && !dataOut.empty()) {
    LogInfo("Read correction file from lidar success");
    return 0;
  } else {
    LogWarning("Read correction file from lidar fail");
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

bool PtcClient::SetNet(std::string IP, std::string mask, std::string getway, uint8_t vlan_flag, uint16_t vlan_ID)
{
  u8Array_t input, output;
  std::stringstream Ip(IP);
  std::stringstream Mask(mask);
  std::stringstream GetWay(getway);
  std::string byte;
  while (std::getline(Ip, byte, '.')) {
    input.push_back(static_cast<uint8_t>(std::stoi(byte)));
  }
  while (std::getline(Mask, byte, '.')) {
    input.push_back(static_cast<uint8_t>(std::stoi(byte)));
  }
  while (std::getline(GetWay, byte, '.')) {
    input.push_back(static_cast<uint8_t>(std::stoi(byte)));
  }
  input.push_back(vlan_flag);
  input.push_back(static_cast<uint8_t>(vlan_ID >> 8));
  input.push_back(static_cast<uint8_t>(vlan_ID >> 0));
  if (this->QueryCommand(input, output, kPTCSetNet) != 0)
    return false;
  return true;
}

bool PtcClient::SetDesIpandPort(std::string des, uint16_t port, uint16_t GPS_port)
{
  u8Array_t input, output;
  std::stringstream Des(des);
  std::string byte;
  while (std::getline(Des, byte, '.')) {
    input.push_back(static_cast<uint8_t>(std::stoi(byte)));
  }
  input.push_back(static_cast<uint8_t>(port >> 8));
  input.push_back(static_cast<uint8_t>(port >> 0));
  input.push_back(static_cast<uint8_t>(GPS_port >> 8));
  input.push_back(static_cast<uint8_t>(GPS_port >> 0));
  if (this->QueryCommand(input, output, kPTCSetDestinationIPandPort) != 0)
    return false;
  return true;
}

bool PtcClient::SetReturnMode(uint8_t return_mode)
{
  u8Array_t input, output;
  input.push_back(return_mode);
  if (this->QueryCommand(input, output, kPTCSetReturnMode) != 0)
    return false;
  return true;
}

bool PtcClient::SetSyncAngle(uint8_t enable_flag, uint16_t sync_angle)
{
  u8Array_t input, output;
  input.push_back(enable_flag);
  input.push_back(static_cast<uint8_t>(sync_angle >> 8));
  input.push_back(static_cast<uint8_t>(sync_angle >> 0));
  if (this->QueryCommand(input, output, kPTCSetSyncAngle) != 0)
    return false;
  return true;
}

bool PtcClient::SetTmbFPGARegister(uint32_t address, uint32_t data)
{
  u8Array_t input, output;
  input.push_back(static_cast<uint8_t>(kPTCSetTemFpgaRegister >> 24));
  input.push_back(static_cast<uint8_t>(kPTCSetTemFpgaRegister >> 16));
  input.push_back(static_cast<uint8_t>(kPTCSetTemFpgaRegister >> 8));
  input.push_back(static_cast<uint8_t>(kPTCSetTemFpgaRegister >> 0));
  input.push_back(static_cast<uint8_t>(address >> 24));
  input.push_back(static_cast<uint8_t>(address >> 16));
  input.push_back(static_cast<uint8_t>(address >> 8));
  input.push_back(static_cast<uint8_t>(address >> 0));
  input.push_back(static_cast<uint8_t>(data >> 24));
  input.push_back(static_cast<uint8_t>(data >> 16));
  input.push_back(static_cast<uint8_t>(data >> 8));
  input.push_back(static_cast<uint8_t>(data >> 0));
  if (this->QueryCommand(input, output, 0xff) != 0)
    return false;
  return true;
}

bool PtcClient::SetFPGARegister(uint32_t address, uint32_t data)
{
  u8Array_t input, output;
  input.push_back(static_cast<uint8_t>(address >> 24));
  input.push_back(static_cast<uint8_t>(address >> 16));
  input.push_back(static_cast<uint8_t>(address >> 8));
  input.push_back(static_cast<uint8_t>(address >> 0));
  input.push_back(static_cast<uint8_t>(data >> 24));
  input.push_back(static_cast<uint8_t>(data >> 16));
  input.push_back(static_cast<uint8_t>(data >> 8));
  input.push_back(static_cast<uint8_t>(data >> 0));
  if (this->QueryCommand(input, output, kPTCSetFpgaRegister) != 0)
    return false;
  return true;
}

bool PtcClient::SetStandbyMode(uint32_t standby_mode)
{
  u8Array_t input1, output1;
  input1.push_back(static_cast<uint8_t>(standby_mode));
  if (this->QueryCommand(input1, output1, kPTCSetStandbyMode) != 0)
    return false;
  return true;
}

bool PtcClient::SetSpinSpeed(uint32_t speed)
{
  u8Array_t input2, output2;
  input2.push_back(static_cast<uint8_t>(speed >> 8));
  input2.push_back(static_cast<uint8_t>(speed >> 0));
  if (this->QueryCommand(input2, output2, kPTCSetSpinSpeed) != 0)
    return false;
  return true;
}

void PtcClient::CRCInit() {
  uint32_t i, j;

  for (i = 0; i < 256; i++) {
    uint32_t k = 0;
    for (j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1)
      k = (k << 1) ^ (((k ^ j) & 0x80000000) ? 0x04c11db7 : 0);

    m_CRCTable[i] = k;
  }
}

uint32_t PtcClient::CRCCalc(uint8_t *bytes, int len) {
  uint32_t i_crc = 0xffffffff;
  int i = 0;
  for (i = 0; i < len; i++)
    i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ bytes[i]) & 0xff];
  return i_crc;
}
