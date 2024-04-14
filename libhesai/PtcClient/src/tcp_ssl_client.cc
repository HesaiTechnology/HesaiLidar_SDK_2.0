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
 * File:   tcp_ssl_client_boost.cc
 * Author: zhang xu<int_zhangxu@hesaitech.com>
 */
#include "tcp_ssl_client.h"

#include <plat_utils.h>

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
using namespace hesai::lidar;
using std::placeholders::_1;
using std::placeholders::_2;
int tcp_open(const char* ipaddr, int port) {
#ifdef _MSC_VER
  WSADATA wsaData;
  WORD version = MAKEWORD(2, 2);
  int res = WSAStartup(version, &wsaData);  // win sock start up
  if (res) {
      std::cerr << "Initilize winsock error !" << std::endl;
      return false;
  }
#endif  
  int sockfd;
  struct sockaddr_in servaddr;
  printf("ip:%s port:%d\n",ipaddr,port);

  if ((sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1){
    printf("socket errno:%d, %s\n",errno,strerror(errno));
    return -1;
  }
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port);
  if (inet_pton(AF_INET, ipaddr, &servaddr.sin_addr) <= 0) {
#ifdef _MSC_VER
          closesocket(sockfd);
#else
          close(sockfd);
#endif
    sockfd = -1;
    std::cout << __FUNCTION__ << "inet_pton error:" << ipaddr << std::endl;
    return sockfd;
  }

  if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
    printf("connect errno:%d, %s\n",errno,strerror(errno));
#ifdef _MSC_VER
          closesocket(sockfd);
#else
          close(sockfd);
#endif
    sockfd = -1;
    return sockfd;
  }

  return sockfd;
}

int sys_readn_by_ssl(SSL *ssl, void *vptr, int n)
{
    int nleft, nread;
    char *ptr;

    ptr = (char *)(vptr);
    nleft = n;
    while (nleft > 0) {
      nread = SSL_read(ssl, ptr, nleft);
      if (nread < 0) {
          if (errno == EINTR)
              nread = 0;
          else
              return -1;
      }
      else if (nread == 0)
          break;

      nleft -= nread;
      ptr += nread;
    }
    return n - nleft;
}

/*
 *
 * 调用顺序：TcpSslClient() -> InitSslClient() -> Open()
 * 
 */
TcpSslClient::TcpSslClient() {
  m_sServerIP.clear();
  ptc_port_ = 0;
  tcpsock_ = -1;
  m_bLidarConnected = false;
  m_u32ReceiveBufferSize = 4096;
  ctx_ = nullptr;
  ssl_ = nullptr;
}

TcpSslClient::~TcpSslClient() {
  Close();
}

void TcpSslClient::Close() {
  printf("TcpSslClient::Close()\n");

  m_sServerIP.clear();
  ptc_port_ = 0;
  m_bLidarConnected = false;
  if(tcpsock_ > 0) {
#ifdef _MSC_VER
    closesocket(tcpsock_);
    WSACleanup();
#else
    close(tcpsock_);
#endif
  }
  if (ctx_ != nullptr) {
    SSL_CTX_free(ctx_);
  }
  if (ssl_ != nullptr) SSL_shutdown(ssl_);
}

bool TcpSslClient::IsOpened() {
  // std::cout << "is opened" << m_bLidarConnected;

  return m_bLidarConnected;
}

bool TcpSslClient::IsOpened(bool bExpectation) {
  return m_bLidarConnected;
}

bool TcpSslClient::Open(std::string IPAddr, 
                        uint16_t u16Port,
                        bool bAutoReceive, 
                        const char* cert, 
                        const char* private_key, 
                        const char* ca) {
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

  ctx_ = InitSslClient(cert, private_key, ca);
  if(ctx_ == NULL) {
    printf("%s:%d, create SSL_CTX failed\n", __func__, __LINE__);
		// ERR_print_errors_fp(stderr);
		return false;
	}
  return Open();
}

bool TcpSslClient::Open() {
  tcpsock_ = tcp_open(m_sServerIP.c_str(), ptc_port_);
  if(tcpsock_ < 0) {
		printf("Connect to Server Failed!~!~\n");
    Close();
    return false;
	}
  ssl_ = SSL_new(ctx_);
  if (ssl_ == NULL) {
		printf("%s:%d, create ssl failed\n", __func__, __LINE__);
		Close();
    return false;
	}

  SSL_set_fd(ssl_, tcpsock_);
	if(SSL_connect(ssl_) == 0) {
		printf("%s:%d, connect ssl failed\n", __func__, __LINE__);
		Close();
    return false;
	}

  if(SSL_get_verify_result(ssl_) != X509_V_OK) {
		printf("%s:%d, verify ssl failed\n", __func__, __LINE__);
		Close();
    return false;
	}

  printf("TcpSslClient::Open() success!\n");
  m_bLidarConnected = true;
  return true;
}

SSL_CTX* TcpSslClient::InitSslClient(const char* cert, const char* private_key, const char* ca) {
  SSL_library_init();
	SSL_load_error_strings();
	OpenSSL_add_all_algorithms();
	SSL_CTX *ctx = SSL_CTX_new(SSLv23_client_method());

	if(ctx == NULL) {
		printf("%s:%d, create SSL_CTX failed\n", __func__, __LINE__);
		return NULL;
	}

  if (ca) {
    printf("ca path: %s\n",ca);
		if(	SSL_CTX_load_verify_locations(ctx, ca, NULL) == 0) {
			// ERR_print_errors_fp(stderr);
			printf("%s:%d, load ca failed,please check ca file path\n", __func__, __LINE__);
			return NULL;
		}
	}

	if (cert && private_key){
    printf("cert path: %s,\nprivate_key path: %s\n",cert, private_key);
		SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, NULL);
		if(SSL_CTX_use_certificate_file(ctx, cert, SSL_FILETYPE_PEM) == 0) {
      printf("%s:%d, load cert file failed,please check cert file path\n", __func__, __LINE__);
			return NULL;
    }
    if(SSL_CTX_use_PrivateKey_file(ctx, private_key, SSL_FILETYPE_PEM) == 0) {
      printf("%s:%d, load private key file failed,please check private key file path\n", __func__, __LINE__);
			return NULL;
    }
    if(SSL_CTX_check_private_key(ctx) == 0) {
      printf("%s:%d, check private key failed\n", __func__, __LINE__);
			return NULL;
    }
	}
	return ctx;
}

int TcpSslClient::Send(uint8_t *u8Buf, uint16_t u16Len, int flags) {
  int len = -1;
  bool ret = true;

  if(!IsOpened()) ret = Open();
  if(ret) {
    len = SSL_write(ssl_, u8Buf, u16Len);
    if (len != u16Len && errno != EAGAIN && errno != EWOULDBLOCK &&
      errno != EINTR) {
      std::cout << __FUNCTION__ << "errno" << errno << std::endl;
      ERR_print_errors_fp(stderr);
      Close();
      return -1;
    }
  }

  return len;
}

int TcpSslClient::Receive(uint8_t *u8Buf, uint32_t u32Len, int flags) {
  int len = -1;
  bool ret = true;
  
  if(!IsOpened()) ret = Open();

  int tick = GetMicroTickCount();
  if(ret) {
    len = sys_readn_by_ssl(ssl_, u8Buf , (int32_t)u32Len);
    if(len != (int32_t)u32Len) {
      printf("ssl receive failed\n");
		  Close();
      return -1;
    }
  }
  
  int delta = GetMicroTickCount() - tick;

  if (delta >= 1000000) {
    std::cout << __FUNCTION__ << "execu:" << delta << "us" << std::endl;
  }

  return len;
}


bool TcpSslClient::SetReceiveTimeout(uint32_t u32Timeout) {
  if (tcpsock_ < 0) {
    printf("TcpClient not open\n");
    return false;
  }
#ifdef _MSC_VER
  int timeout_ms = u32Timeout;
  int retVal = setsockopt(tcpsock_, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout_ms,
      sizeof(timeout_ms));
#else
  uint32_t sec = u32Timeout / 1000;
  uint32_t msec = u32Timeout % 1000;

  struct timeval timeout;
  timeout.tv_sec = sec;
  timeout.tv_usec = msec * 1000;
  int retVal = setsockopt(tcpsock_, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&timeout, sizeof(timeval));
#endif
  return retVal == 0;
}


int TcpSslClient::SetTimeout(uint32_t u32RecMillisecond,
                          uint32_t u32SendMillisecond) {
  if (tcpsock_ < 0) {
    printf("TcpClient not open\n");
    return -1;
  }
  m_u32RecTimeout = u32RecMillisecond;
  m_u32SendTimeout = u32SendMillisecond;
#ifdef _MSC_VER
  int timeout_ms = u32RecMillisecond;
  int retVal = setsockopt(tcpsock_, SOL_SOCKET, SO_RCVTIMEO,
                        (char*)&timeout_ms, sizeof(timeout_ms));
#else
  uint32_t sec = u32RecMillisecond / 1000;
  uint32_t msec = u32RecMillisecond % 1000;

  struct timeval timeout;
  timeout.tv_sec = sec;
  timeout.tv_usec = msec * 1000;
  int retVal = setsockopt(tcpsock_, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&timeout, sizeof(timeval));
#endif
  if (retVal == 0) {
#ifdef _MSC_VER
    int timeout_ms = u32SendMillisecond;
    retVal = setsockopt(tcpsock_, SOL_SOCKET, SO_SNDTIMEO,
                        (char*)&timeout_ms, sizeof(timeout_ms));  
#else
    uint32_t sec = u32SendMillisecond / 1000;
    uint32_t msec = u32SendMillisecond % 1000;

    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = msec * 1000;
    retVal = setsockopt(tcpsock_, SOL_SOCKET, SO_SNDTIMEO,
                        (const void *)&timeout, sizeof(timeval));
#endif
  }
  return retVal;
}

void TcpSslClient::SetReceiveBufferSize(const uint32_t &size) {
  if (tcpsock_ < 0) {
    printf("TcpClient not open\n");
    return;
  }

  m_u32ReceiveBufferSize = size;
  uint32_t recbuffSize;
  socklen_t optlen = sizeof(recbuffSize);
  int ret = getsockopt(tcpsock_, SOL_SOCKET, SO_RCVBUF, (char*)&recbuffSize, &optlen);
  if (ret == 0 && recbuffSize < size) {
    setsockopt(tcpsock_, SOL_SOCKET, SO_RCVBUF, (char*)&size, sizeof(size));
  }
}