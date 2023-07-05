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
 * File:   tcp_client.h
 * Author: Felix Zou<zouke@hesaitech.com>
 *
 * Created on Sep 5, 2019, 10:46 AM
 */

#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <stdint.h>
#include <string.h>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
namespace hesai
{
namespace lidar
{
class TcpClient{
 public:
  explicit TcpClient();
  virtual ~TcpClient();

  TcpClient(const TcpClient &orig) = delete;

  bool Open(std::string IPAddr, uint16_t port, bool bAutoReceive = false);
  bool Open();
  void Close();
  bool IsOpened();
  bool IsOpened(bool bExpectation);
  virtual int Send(uint8_t *u8Buf, uint16_t u16Len, int flags = 0);
  virtual int Receive(uint8_t *u8Buf, uint32_t u16Len, int flags = 0);

  /**
   * @brief 设置接收超时
   * @param u32Timeout 超时时间/ms
   * @return
   */
  bool SetReceiveTimeout(uint32_t u32Timeout);

  /**
   * @brief 设置收发超时
   * @param u32RecMillisecond 接收超时/ms
   * @param u32SendMillisecond 发送超时/ms
   * @return
   */
  int SetTimeout(uint32_t u32RecMillisecond, uint32_t u32SendMillisecond);

  /**
   * @brief 设置自动接收模式下Buff的大小
   * @param size
   */
  void SetReceiveBufferSize(const uint32_t &size);

 private:
  /**
   * monitor file descriptor and wait for I/O operation
   */
  int WaitFor(const int &socketfd, uint32_t timeoutSeconds = 1);

 protected:
  static const uint32_t kDefaultTimeout = 500;

  std::string m_sServerIP;
  uint16_t ptc_port_;
  int m_tcpSock;
  bool m_bLidarConnected;
  uint32_t m_u32ReceiveBufferSize;
  // 收发超时/ms
  uint32_t m_u32RecTimeout = kDefaultTimeout;
  uint32_t m_u32SendTimeout = kDefaultTimeout;
};
}
}
#endif /* TCPCLIENT_H */
