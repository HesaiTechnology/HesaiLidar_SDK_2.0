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
 * File:   tcp_client.cc
 * Author: Felix Zou<zouke@hesaitech.com>
 */

#include "tcp_client.h"

#include <plat_utils.h>
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
using namespace hesai::lidar;
TcpClient::TcpClient() {
  m_sServerIP.clear();
  ptc_port_ = 0;
  m_tcpSock = -1;
  m_bLidarConnected = false;
  m_u32ReceiveBufferSize = 4096;
}

TcpClient::~TcpClient() {
  Close();
}

void TcpClient::Close() {
  printf("TcpClient::Close()\n");

  m_sServerIP.clear();
  ptc_port_ = 0;
  m_bLidarConnected = false;

  if (m_tcpSock > 0) {
    close(m_tcpSock);
    m_tcpSock = -1;
  }
}

bool TcpClient::Open(std::string IPAddr, uint16_t u16Port, bool bAutoReceive) {
  if (IsOpened(true) && m_sServerIP == IPAddr && u16Port == ptc_port_) {
    return true;
  }

  std::cout << __PRETTY_FUNCTION__ << "IP" << IPAddr.c_str() << "port"
           << u16Port << std::endl;
  Close();

  m_sServerIP = IPAddr;
  ptc_port_ = u16Port;
  return Open();
}

bool TcpClient::Open() {
  struct sockaddr_in serverAddr;

  m_tcpSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (m_tcpSock == -1) return false;

  memset(&serverAddr, 0, sizeof(serverAddr));

  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(ptc_port_);
  if (inet_pton(AF_INET, m_sServerIP.c_str(), &serverAddr.sin_addr) <= 0) {
    close(m_tcpSock);
    m_tcpSock = -1;
    std::cout << __FUNCTION__ << "inet_pton error:" << m_sServerIP.c_str() << std::endl;

    return false;
  }

  // int retVal = SetTimeout(m_u32RecTimeout, m_u32SendTimeout);
  int retVal = 0;

  if (retVal == 0) {
    if (::connect(m_tcpSock, (sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
      if (EINPROGRESS != errno && EWOULDBLOCK != errno) {
        close(m_tcpSock);
        m_tcpSock = -1;
        std::cout << __FUNCTION__ << "connect failed" << errno << std::endl;

        return false;
      } else if (EINPROGRESS == errno) {
        std::cout << "connect lidar time out\n";

        return false;
      }

      std::cout << "connect lidar fail errno" << errno << std::endl;

      return false;
    }
  } else {
    std::cout << __FUNCTION__ << "setsockopt failed, errno" << errno << std::endl;

    return false;
  }

  std::cout << __FUNCTION__ << "succeed, IP" << m_sServerIP.c_str() << "port"
           << ptc_port_ << std::endl;

  m_bLidarConnected = true;

  return true;
}

bool TcpClient::IsOpened() {
  // std::cout << "is opened" << m_bLidarConnected;

  return m_bLidarConnected;
}

bool TcpClient::IsOpened(bool bExpectation) {
  return m_bLidarConnected;
}

int TcpClient::Send(uint8_t *u8Buf, uint16_t u16Len, int flags) {
  ssize_t len = -1;
  bool ret = true;

  if (!IsOpened()) ret = Open();

  if (ret) {
    len = send(m_tcpSock, u8Buf, u16Len, flags);
    if (len != u16Len && errno != EAGAIN && errno != EWOULDBLOCK &&
        errno != EINTR) {
      std::cout << __FUNCTION__ << "errno" << errno << std::endl;
      close(m_tcpSock);
      m_tcpSock = -1;
      m_bLidarConnected = false;
    }
  }
  return len;
}

int TcpClient::Receive(uint8_t *u8Buf, uint32_t u32Len, int flags) {
  ssize_t len = -1;
  bool ret = true;

  if (!IsOpened()) ret = Open();

  int tick = GetMicroTickCount();
  if (ret) {
    len = recv(m_tcpSock, u8Buf, u32Len, flags);
    if (len == 0 || (len == -1 && errno != EINTR && errno != EAGAIN &&
                     errno != EWOULDBLOCK)) {
      std::cout << __FUNCTION__ << "len: " << len << " errno: " << errno << std::endl;
      close(m_tcpSock);
      m_tcpSock = -1;
      m_bLidarConnected = false;
    }
  }

  int delta = GetMicroTickCount() - tick;

  if (delta >= 1000000) {
    std::cout << __FUNCTION__ << "execu:" << delta << "us" << std::endl;
  }

  return len;
}


bool TcpClient::SetReceiveTimeout(uint32_t u32Timeout) {
  if (m_tcpSock < 0) {
    printf("TcpClient not open\n");
    return false;
  }

  uint32_t sec = u32Timeout / 1000;
  uint32_t msec = u32Timeout % 1000;

  struct timeval timeout;
  timeout.tv_sec = sec;
  timeout.tv_usec = msec * 1000;
  int retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&timeout, sizeof(timeval));
  return retVal == 0;
}

int TcpClient::SetTimeout(uint32_t u32RecMillisecond,
                          uint32_t u32SendMillisecond) {
  if (m_tcpSock < 0) {
    printf("TcpClient not open\n");
    return -1;
  }
  m_u32RecTimeout = u32RecMillisecond;
  m_u32SendTimeout = u32SendMillisecond;
  uint32_t sec = u32RecMillisecond / 1000;
  uint32_t msec = u32RecMillisecond % 1000;

  struct timeval timeout;
  timeout.tv_sec = sec;
  timeout.tv_usec = msec * 1000;
  int retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&timeout, sizeof(timeval));
  if (retVal == 0) {
    uint32_t sec = u32SendMillisecond / 1000;
    uint32_t msec = u32SendMillisecond % 1000;

    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = msec * 1000;
    retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_SNDTIMEO,
                        (const void *)&timeout, sizeof(timeval));
  }

  return retVal;
}

void TcpClient::SetReceiveBufferSize(const uint32_t &size) {
  if (m_tcpSock < 0) {
    printf("TcpClient not open\n");
    return;
  }

  m_u32ReceiveBufferSize = size;
  int recbuffSize;
  socklen_t optlen = sizeof(recbuffSize);
  int ret = getsockopt(m_tcpSock, SOL_SOCKET, SO_RCVBUF, &recbuffSize, &optlen);
  if (ret == 0 && recbuffSize < size) {
    setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
  }
}


int TcpClient::WaitFor(const int &socketfd, uint32_t timeoutSeconds) {
  struct timeval tv;
  tv.tv_sec = timeoutSeconds;
  tv.tv_usec = 0;
  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(socketfd, &fds);
  const int selectRet = select(socketfd + 1, &fds, nullptr, nullptr, &tv);

  return selectRet;
}

