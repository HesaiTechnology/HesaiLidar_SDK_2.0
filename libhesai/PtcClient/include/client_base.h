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
 * File:   client_base.h
 * Author: zhang xu<int_zhangxu@hesaitech.com>
 *
 * Created on Jul 9, 2023, 20:03 PM
 */

#ifndef CLIENT_BASE_H
#define CLIENT_BASE_H

#include <stdint.h>
#include <string.h>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <memory>
#include <vector>

#ifdef _MSC_VER
#include <WinSock2.h>
#else
#include <arpa/inet.h>
#endif
namespace hesai
{
namespace lidar
{
class ClientBase {
 public:
  explicit ClientBase(){}
  virtual ~ClientBase(){}
  virtual bool Open(std::string IPAddr, uint16_t port, bool bAutoReceive = false, 
                    const char* cert  = nullptr, const char* private_key = nullptr, const char* ca = nullptr) = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() = 0;
  virtual int Send(uint8_t *u8Buf, uint16_t u16Len, int flags = 0) = 0;
  virtual int Receive(uint8_t *u8Buf, uint32_t u16Len, int flags = 0) = 0;
  /**
   * @brief 设置接收超时
   * @param u32Timeout 超时时间/ms
   * @return
   */
  virtual bool SetReceiveTimeout(uint32_t u32Timeout) = 0;

  /**
   * @brief 设置收发超时
   * @param u32RecMillisecond 接收超时/ms
   * @param u32SendMillisecond 发送超时/ms
   * @return
   */
  virtual int SetTimeout(uint32_t u32RecMillisecond, uint32_t u32SendMillisecond) = 0;

  /**
   * @brief 设置自动接收模式下Buff的大小
   * @param size
   */
  virtual void SetReceiveBufferSize(const uint32_t &size) = 0;
};

}
}

#endif