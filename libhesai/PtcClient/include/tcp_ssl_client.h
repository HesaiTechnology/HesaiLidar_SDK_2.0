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
 * File:   tcp_client_boost.h
 * Author: zhang xu<int_zhangxu@hesaitech.com>
 *
 * Created on Jul 7, 2023, 13:55 PM
 */

#ifndef TCPSSLCLIENT_H
#define TCPSSLCLIENT_H

#include "client_base.h"
// #include "util.h"
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <string.h>
#include <stdint.h>
#include <memory>
#include <iostream>
#include <thread>
#include <chrono>
#include <iostream>
#ifdef _MSC_VER
#include <winsock2.h>
#include <ws2tcpip.h> 
#else
#ifndef SOCKET
typedef unsigned int SOCKET;
#endif
#endif

const int MAX_LENGTH = 1024;

namespace hesai
{
namespace lidar
{
class TcpSslClient : public ClientBase {
 public:
  explicit TcpSslClient();
  virtual ~TcpSslClient();

  TcpSslClient(const TcpSslClient &orig) = delete;

  // 设置ssl单双向模式、密钥、证书等
  SSL_CTX* InitSslClient(const char* cert, const char* private_key, const char* ca);
  
  virtual bool Open(std::string IPAddr, uint16_t port, bool bAutoReceive = false, 
            const char* cert = nullptr, const char* private_key = nullptr, const char* ca = nullptr);
  virtual bool TryOpen(std::string IPAddr, uint16_t port, bool bAutoReceive = false, 
            const char* cert = nullptr, const char* private_key = nullptr, const char* ca = nullptr, uint32_t timeout = 1);
  bool Open();
  virtual void Close();
  virtual bool IsOpened();
  bool IsOpened(bool bExpectation);
  virtual int Send(uint8_t *u8Buf, uint16_t u16Len, int flags = 0);
  virtual int Receive(uint8_t *u8Buf, uint32_t u32Len, int flags = 0);

  /**
   * @brief 设置接收超时
   * @param u32Timeout 超时时间/ms
   * @return
   */
  virtual bool SetReceiveTimeout(uint32_t u32Timeout);

  /**
   * @brief 设置收发超时
   * @param u32RecMillisecond 接收超时/ms
   * @param u32SendMillisecond 发送超时/ms
   * @return
   */
  virtual int SetTimeout(uint32_t u32RecMillisecond, uint32_t u32SendMillisecond);

  /**
   * @brief 设置自动接收模式下Buff的大小
   * @param size
   */
  virtual void SetReceiveBufferSize(const uint32_t &size);

 private:
  // void connect(const tcp::resolver::results_type& endpoints);
  void handshake();

 protected:
  static const uint32_t kDefaultTimeout = 500;

  std::string m_sServerIP;
  uint16_t ptc_port_;
  bool m_bLidarConnected;
  uint32_t m_u32ReceiveBufferSize;
  // 收发超时/ms
  uint32_t m_u32RecTimeout = kDefaultTimeout;
  uint32_t m_u32SendTimeout = kDefaultTimeout;
  SSL_CTX* ctx_;
  SOCKET tcpsock_;
  SSL* ssl_;
  const char* cert_;
  const char* private_key_;
  const char* ca_;
};
}
}

#endif