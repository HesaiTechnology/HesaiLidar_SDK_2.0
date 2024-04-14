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
 * File:   tcp_client.cc
 * Author: Felix Zou<zouke@hesaitech.com>
 */

#include "tcp_client.h"
#ifdef _MSC_VER
#include <winsock2.h>
#include <ws2tcpip.h> 
#pragma comment(lib, "ws2_32.lib")  // Winsock Library
#include <BaseTsd.h>

typedef int socklen_t;
#define MSG_DONTWAIT (0x40)
#else
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#include <plat_utils.h>
#include <string.h>
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
#ifdef _MSC_VER
          closesocket(m_tcpSock);
          WSACleanup();
#else
          close(m_tcpSock);
#endif
    m_tcpSock = -1;
  }
}

bool TcpClient::Open(std::string IPAddr, uint16_t u16Port, bool bAutoReceive,
          const char* cert, const char* private_key, const char* ca) {
  if (IsOpened(true) && m_sServerIP == IPAddr && u16Port == ptc_port_) {
    return true;
  }
#ifdef _MSC_VER
  std::cout << __FUNCTION__ << "IP" << IPAddr.c_str() << "port"
           << u16Port << std::endl;
#else
  std::cout << __PRETTY_FUNCTION__ << "IP" << IPAddr.c_str() << "port"
           << u16Port << std::endl;
#endif
  Close();

  m_sServerIP = IPAddr;
  ptc_port_ = u16Port;
  return Open();
}

bool TcpClient::Open() {
#ifdef _MSC_VER
  WSADATA wsaData;
  WORD version = MAKEWORD(2, 2);
  int res = WSAStartup(version, &wsaData);  // win sock start up
  if (res) {
      std::cerr << "Initilize winsock error !" << std::endl;
      return false;
  }
#endif  
  struct sockaddr_in serverAddr;

  m_tcpSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  if ((int)m_tcpSock == -1) return false;

  memset(&serverAddr, 0, sizeof(serverAddr));

  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(ptc_port_);
  if (inet_pton(AF_INET, m_sServerIP.c_str(), &serverAddr.sin_addr) <= 0) {
#ifdef _MSC_VER
          closesocket(m_tcpSock);
#else
          close(m_tcpSock);
#endif
    m_tcpSock = -1;
    std::cout << __FUNCTION__ << "inet_pton error:" << m_sServerIP.c_str() << std::endl;

    return false;
  }

  // int retVal = SetTimeout(m_u32RecTimeout, m_u32SendTimeout);
  int retVal = 0;

  if (retVal == 0) {
    if (::connect(m_tcpSock, (sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
      if (EINPROGRESS != errno && EWOULDBLOCK != errno) {
#ifdef _MSC_VER
          closesocket(m_tcpSock);
#else
          close(m_tcpSock);
#endif
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

  std::cout << __FUNCTION__ << " succeed, IP" << m_sServerIP.c_str() << "port"
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
  int len = -1;
  bool ret = true;

  if (!IsOpened()) ret = Open();

  if (ret) {
    len = send(m_tcpSock, (char*)u8Buf, u16Len, flags);
    if (len != u16Len && errno != EAGAIN && errno != EWOULDBLOCK &&
        errno != EINTR) {
      std::cout << __FUNCTION__ << "errno" << errno << std::endl;
#ifdef _MSC_VER
          closesocket(m_tcpSock);
#else
          close(m_tcpSock);
#endif
      m_tcpSock = -1;
      m_bLidarConnected = false;
    }
  }
  return len;
}

int TcpClient::Receive(uint8_t *u8Buf, uint32_t u32Len, int flags) {
  int len = -1;
  bool ret = true;

  if (!IsOpened()) ret = Open();

  int tick = GetMicroTickCount();
  if (ret) {
#ifdef _MSC_VER
  if (flags == MSG_DONTWAIT) {
    unsigned long nonBlockingMode = 1;
    ioctlsocket(m_tcpSock, FIONBIO, &nonBlockingMode);
  }
#endif
    len = recv(m_tcpSock, (char*)u8Buf, u32Len, flags);
    if (len == 0 || (len == -1 && errno != EINTR && errno != EAGAIN &&
                     errno != EWOULDBLOCK)) {
      std::cout << __FUNCTION__ << ", len: " << len << " errno: " << errno << std::endl;
#ifdef _MSC_VER
          closesocket(m_tcpSock);
#else
          close(m_tcpSock);
#endif
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
#ifdef _MSC_VER
  int timeout_ms = u32Timeout;
  int retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout_ms,
      sizeof(timeout_ms));
#else
  uint32_t sec = u32Timeout / 1000;
  uint32_t msec = u32Timeout % 1000;
  struct timeval timeout;
  timeout.tv_sec = sec;
  timeout.tv_usec = msec * 1000;
  int retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&timeout, sizeof(timeval));
#endif
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
#ifdef _MSC_VER
  int timeout_ms = u32RecMillisecond;
  int retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVTIMEO,
                        (char*)&timeout_ms, sizeof(timeout_ms));
#else
  uint32_t sec = u32RecMillisecond / 1000;
  uint32_t msec = u32RecMillisecond % 1000;

  struct timeval timeout;
  timeout.tv_sec = sec;
  timeout.tv_usec = msec * 1000;
  int retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&timeout, sizeof(timeval));
#endif
  if (retVal == 0) {
#ifdef _MSC_VER
    int timeout_ms = u32SendMillisecond;
    retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_SNDTIMEO,
                        (char*)&timeout_ms, sizeof(timeout_ms));    
#else    
    uint32_t sec = u32SendMillisecond / 1000;
    uint32_t msec = u32SendMillisecond % 1000;

    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = msec * 1000;
    retVal = setsockopt(m_tcpSock, SOL_SOCKET, SO_SNDTIMEO,
                        (const void *)&timeout, sizeof(timeval));
#endif
  }
  return retVal;
}

void TcpClient::SetReceiveBufferSize(const uint32_t &size) {
  if (m_tcpSock < 0) {
    printf("TcpClient not open\n");
    return;
  }

  m_u32ReceiveBufferSize = size;
  uint32_t recbuffSize;
  socklen_t optlen = sizeof(recbuffSize);
  int ret = getsockopt(m_tcpSock, SOL_SOCKET, SO_RCVBUF, (char*)&recbuffSize, &optlen);
  if (ret == 0 && recbuffSize < size) {
    setsockopt(m_tcpSock, SOL_SOCKET, SO_RCVBUF, (char*)&size, sizeof(size));
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

