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
 * File:   socket_source.cc
 * Author: Felix Zou<zouke@hesaitech.com>
 */

#include "socket_source.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/ip.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <sys/file.h>
#include <iostream>
using namespace hesai::lidar;
SocketSource::SocketSource(uint16_t port, std::string multicastIp) {
  client_ip_.clear();
  udp_port_ = port;
  udp_sock_ = -1;
  multicast_ip_ = multicastIp;
  is_select_ = false;
}

SocketSource::~SocketSource() { Close(); }

void SocketSource::Close() {
  printf("SocketSource::Close()\n");

  client_ip_.clear();
  udp_port_ = 0;

  if (udp_sock_ > 0) {
    close(udp_sock_);
    udp_sock_ = -1;
  }
}


bool SocketSource::Open() {
  int retVal = 0;
  struct sockaddr_in serverAddr;

  udp_sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (udp_sock_ == -1) return false;

  memset(&serverAddr, 0, sizeof(serverAddr));

  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddr.sin_port = htons(udp_port_);

  int reuseaddr = 1;
  retVal = setsockopt(udp_sock_, SOL_SOCKET, SO_REUSEADDR, (char*)&reuseaddr,
                      sizeof(reuseaddr));
  int nRecvBuf = 400000000;
  setsockopt(udp_sock_, SOL_SOCKET, SO_RCVBUF, (const char *)&nRecvBuf,
             sizeof(int));
  int curRcvBufSize = -1;
  socklen_t optlen = sizeof(curRcvBufSize);
  if (getsockopt(udp_sock_, SOL_SOCKET, SO_RCVBUF, &curRcvBufSize, &optlen) <
      0) {
    printf("getsockopt error=%d(%s)!!!\n", errno, strerror(errno));
  }
  printf("OS current udp socket recv buff size is: %d\n", curRcvBufSize);                 

  if (retVal == 0) {
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000;

    retVal = setsockopt(udp_sock_, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
                        sizeof(struct timeval));

    if (retVal == 0) {
      if (bind(udp_sock_, (sockaddr*)&serverAddr, sizeof(sockaddr)) == -1) {
        if (EINPROGRESS != errno && EWOULDBLOCK != errno) {
          close(udp_sock_);
          udp_sock_ = -1;
          printf("SocketSource::Open(), bind failed, errno: %d\n", errno);
          return false;
        }
      } else {
        printf("SocketSource::Open succeed, sock:%d\n", udp_sock_);
      }
    } else {
      printf("setsockopt SO_RCVTIMEO failed, errno:%d\n", errno);
    }
  } else {
    printf("setsockopt SO_REUSEADDR failed, errno:%d\n", errno);
  }
  if (fcntl(udp_sock_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
  }

  int32_t rcvBufSize = kUDPBufferSize;
  setsockopt(udp_sock_, SOL_SOCKET, SO_RCVBUF, &rcvBufSize,
      sizeof(rcvBufSize));

  if(multicast_ip_ != ""){
    struct ip_mreq mreq;                    
    mreq.imr_multiaddr.s_addr=inet_addr(multicast_ip_.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY); 
    int ret = setsockopt(udp_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq));
    if (ret < 0) {
      perror("Multicast IP error,set correct multicast ip address or keep it empty\n");
    } 
    else {
      printf("Recive data from multicast ip address %s\n", multicast_ip_.c_str());
    }
  }
  return retVal == 0;
}

bool SocketSource::IsOpened() {
  bool ret = true;

  if (udp_port_ == 0 || udp_sock_ < 0) {
    ret = false;
    printf("SocketSource::IsOpened(), port %d, sock %d\n", udp_port_,
           udp_sock_);
  }

  return ret;
}

int SocketSource::Send(uint8_t* u8Buf, uint16_t u16Len, int flags) {
  ssize_t len = -1;
  bool ret = true;
  if (!IsOpened()) ret = false;

  if (ret) {
    sockaddr addr;
    int val = inet_pton(AF_INET, client_ip_.c_str(), &addr);
    if (val == 1) {
      len = sendto(udp_sock_, u8Buf, u16Len, flags, &addr, sizeof(addr));
      if (len == -1) printf("SocketSource::Send, errno:%d\n", errno);
    } else {
      printf("SocketSource::Send(), invalid IP %s\n", client_ip_.c_str());
    }
  }
  return len;
}

bool isTimeout = false;

int SocketSource::Receive(UdpPacket& udpPacket, uint16_t u16Len, int flags,
                       int iTimeout) {  
  ssize_t len = -1;
  bool ret = true;
  if (!IsOpened()) ret = Open();

  if (ret) {
    fd_set rfd;
    timeval timeout;

    timeout.tv_sec = 0;
    timeout.tv_usec = iTimeout;
    FD_ZERO(&rfd);

    FD_SET(udp_sock_, &rfd);
    if (!is_select_) {
      sockaddr_in clientAddr;
      socklen_t addrLen = sizeof(sockaddr);
      len = recvfrom(udp_sock_, (char*)udpPacket.buffer, u16Len, flags,
                    (sockaddr*)&clientAddr, &addrLen);
      if(len == -1) {is_select_ = true;}
    } else {
      int cnt = select(udp_sock_ + 1, &rfd, NULL, NULL, &timeout);
      if (cnt > 0) {
        is_select_ = false;
        sockaddr_in clientAddr;
        socklen_t addrLen = sizeof(sockaddr);
        len = recvfrom(udp_sock_, (char*)udpPacket.buffer, u16Len, flags,
                     (sockaddr*)&clientAddr, &addrLen);        
      } else if (cnt == 0) {
        len = 0;
        udpPacket.is_timeout = true;        
      } else {
        std::cout << "Select timeout error" << std::endl;
      }
    } 
  }

  return len;
}

void SocketSource::SetSocketBufferSize(uint32_t u32BufSize) {
  setsockopt(udp_sock_, SOL_SOCKET, SO_RCVBUF, &u32BufSize,
      sizeof(u32BufSize));
}
