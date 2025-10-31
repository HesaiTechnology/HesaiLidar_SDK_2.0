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

#include "someip_client.h"
#include "logger.h"
#include <sstream>
#include <stdint.h>
using namespace hesai::lidar;

uint16_t crc16_ccitt_false(uint8_t* data, int data_len) {
    uint16_t crc = 0xFFFF; // Initial value
    const uint16_t polynomial = 0x1021; // CRC-16-CCITT polynomial
    
    for (int i = 0; i < data_len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8; // Move byte into MSB
        
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) { // Check if MSB is set
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc; // No final XOR (corout=0x0000)
}

uint32_t IPStringToUint32(std::string ip)
{
  uint32_t host_ip_uint = 0;
  std::istringstream ss(ip);
  std::string segment;  
  int segmentsCount = 0;  
  while (std::getline(ss, segment, '.')) {  
      if (segment.empty() || segmentsCount >= 4)  
          return 0;
      uint8_t ip_one = std::stoi(segment);  
      host_ip_uint = (host_ip_uint << 8) + ip_one;
      segmentsCount++;
  }
  if (segmentsCount != 4) return 0;
  return host_ip_uint;
}


// 使用 std::stoi 转换十六进制字符串
inline uint32_t HexStringToUint16Stoi(const std::string& hexStr) {
  if (hexStr.empty()) {
      return 0;
  }
  try {
      // std::stoi 支持自动识别0x前缀
      size_t pos;
      unsigned long result = std::stoul(hexStr, &pos, 16);
      
      // 检查是否所有字符都被处理
      if (pos != hexStr.length()) {
          throw std::invalid_argument("Invalid hexadecimal string: " + hexStr);
      }
      
      // 检查是否超出uint32_t范围
      if (result > UINT32_MAX) {
          throw std::out_of_range("Hex value too large for uint32_t: " + hexStr);
      }
      
      return static_cast<uint32_t>(result);
  } catch (const std::exception& e) {
      throw std::invalid_argument("Failed to convert hex string: [" + hexStr + "], error: " + e.what());
  }
  return 0;
}

SomeIPClient::SomeIPClient(const uint16_t port, const std::string& multicastIp) {
    udp_port_ = port;
    multicast_ip_ = multicastIp;
}

SomeIPClient::~SomeIPClient() {
    Close();
}

void SomeIPClient::Close() {

  if (udp_sock_ != -1) {
#ifdef _MSC_VER
    closesocket(udp_sock_);
    WSACleanup();
#else
    close(udp_sock_);
#endif
    udp_sock_ = -1;
  }
}

void SomeIPClient::Close(int &socket) {

    if (socket != -1) {
#ifdef _MSC_VER
        closesocket(socket);
        WSACleanup();
#else
        close(socket);
#endif
        socket = -1;
    }
}

bool SomeIPClient::Open() {
  return Open(udp_sock_, udp_port_);
}

bool SomeIPClient::Open(int& udp_sock, uint16_t port, uint8_t udp_buf_type) {
#ifdef _MSC_VER
    WSADATA wsaData;
    WORD version = MAKEWORD(2, 2);
    int res = WSAStartup(version, &wsaData);  // win sock start up
    if (res) {
        LogError("Initilize winsock error !");
        return false;
    }
#endif
  int retVal = 0;
  struct sockaddr_in serverAddr;

  udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (udp_sock == -1) return false;

  memset(&serverAddr, 0, sizeof(serverAddr));

  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddr.sin_port = htons(port);

  int reuseaddr = 1;
  retVal = setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, (char*)&reuseaddr,
                      sizeof(reuseaddr));
  if (udp_buf_type == 1) {
    int nRecvBuf = 400000000;
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVBUF, (const char *)&nRecvBuf,
              sizeof(int));
    int curRcvBufSize = -1;
    socklen_t optlen = sizeof(curRcvBufSize);
    if (getsockopt(udp_sock, SOL_SOCKET, SO_RCVBUF, (char*)&curRcvBufSize, &optlen) <
        0) {
      LogWarning("getsockopt error=%d(%s)!!!", errno, strerror(errno));
    }
    LogInfo("OS current udp socket recv buff size is: %d", curRcvBufSize); 
  }

  if (retVal == 0) {
#ifdef _MSC_VER
    int timeout_ms = 20;
    retVal = setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout_ms,
        sizeof(timeout_ms));
    // SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#else
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000;

    retVal = setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
                        sizeof(struct timeval));
#endif
    if (retVal == 0) {
      if (bind(udp_sock, (sockaddr*)&serverAddr, sizeof(sockaddr)) == -1) {
        if (EINPROGRESS != errno && EWOULDBLOCK != errno) {
#ifdef _MSC_VER
          closesocket(udp_sock);
#else
          close(udp_sock);
#endif
          udp_sock = -1;
          LogError("SomeIPClient::Open(), bind failed, errno: %d", errno);
          return false;
        }
      } else {
        LogInfo("SomeIPClient::Open succeed, sock:%d", udp_sock);
      }
    } else {
      LogError("setsockopt SO_RCVTIMEO failed, errno:%d", errno);
    }
  } else {
    LogError("setsockopt SO_REUSEADDR failed, errno:%d", errno);
  }
#ifdef _MSC_VER
  unsigned long nonBlockingMode = 1;
  if (ioctlsocket(udp_sock, FIONBIO, &nonBlockingMode) != 0) {
      LogError("non-block");
  }
#else
  if (fcntl(udp_sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    LogError("non-block");
  }
#endif

  int32_t rcvBufSize = kUDPBufferSize;
  setsockopt(udp_sock, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBufSize,
      sizeof(rcvBufSize));

  if(multicast_ip_ != ""){
    struct ip_mreq mreq;                    
    mreq.imr_multiaddr.s_addr=inet_addr(multicast_ip_.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY); 
    int ret = setsockopt(udp_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq));
    if (ret < 0) {
      LogError("Multicast IP error, set correct multicast ip address or keep it empty");
    } 
    else {
      LogInfo("Someip recive data from multicast ip address %s", multicast_ip_.c_str());
    }
  }

  return retVal == 0;
}

bool SomeIPClient::IsOpened() {
  bool ret = true;

  if (udp_sock_ == -1) {
    ret = false;
    LogWarning("SomeIPClient::IsOpened(), port %d, sock %d is not open", udp_port_,
           udp_sock_);
  }

  return ret;
}

bool SomeIPClient::SetReceiveTimeout(int sock, uint32_t u32Timeout) {
  if ((int)sock == -1) {
    LogWarning("Client not open");
    return false;
  }
#ifdef _MSC_VER
  int timeout_ms = u32Timeout;
  int retVal = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout_ms,
      sizeof(timeout_ms));
#else
  uint32_t sec = u32Timeout / 1000;
  uint32_t msec = u32Timeout % 1000;
  struct timeval timeout;
  timeout.tv_sec = sec;
  timeout.tv_usec = msec * 1000;
  int retVal = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&timeout, sizeof(timeval));
#endif
  return retVal == 0;
}

void SomeIPClient::InitSomeIpSDHeader(SomeIPHeader& header, uint32_t payload_len) {
  header.someip_service_id = 0xffff;
  header.someip_method_id = htons(0x8100);
  header.someip_length = htonl(payload_len + 8);
  header.someip_client_id = 0x0000;
  header.someip_session_id = htons(0x0001);
  header.someip_version = 0x01;
  header.someip_interface_version = 0x01;
  header.someip_message_type = 0x02;
  header.someip_return_code = 0x00;
}

bool SomeIPClient::InitSubscribeMessage(SomeIPMessage& message, std::string type, uint8_t ttl, uint32_t ip, uint16_t port) {
  if (matrix_config_.find(type) == matrix_config_.end()) {
    LogError("SomeIPClient::InitSubscribeMessage(), type %d is invalid", type);
    return false;
  }
  InitSomeIpSDHeader(message.header, sizeof(SomeIpSD));
  SomeIpSD *ptr = (SomeIpSD *)(&(message.payload[0]));
  ptr->flags = matrix_config_[type][COL_SD_FLAG];
  ptr->reserved[0] = 0;
  ptr->reserved[1] = 0;
  ptr->reserved[2] = 0;
  ptr->entry_length = htonl(sizeof(SomeIpSD::Entry));
  ptr->entry.eventgroup.type = matrix_config_[type][COL_ENTRY_TYPE];
  ptr->entry.eventgroup.first_option_run = 0;
  ptr->entry.eventgroup.second_option_run = 0;
  ptr->entry.eventgroup.options = 0x10;
  ptr->entry.eventgroup.service_id = htons(matrix_config_[type][COL_SERVICE_ID]);
  ptr->entry.eventgroup.instance_id = htons(matrix_config_[type][COL_INSTANCE_ID]);
  ptr->entry.eventgroup.major_version = 0x01;
  ptr->entry.eventgroup.TTL[0] = ttl;
  ptr->entry.eventgroup.TTL[1] = ttl;
  ptr->entry.eventgroup.TTL[2] = ttl;
  ptr->entry.eventgroup.reserved = 0x00;
  ptr->entry.eventgroup.flag_counter = 0x00;
  ptr->entry.eventgroup.eventgroup_id = htons(matrix_config_[type][COL_EVENTGROUP_ID]);
  ptr->option_length = htonl(sizeof(SomeIpSD::Option::Option_0x4));
  ptr->option.type4.len = htons(0x0009);
  ptr->option.type4.type = matrix_config_[type][COL_OPTION_TYPE];
  ptr->option.type4.discardable_flag_reserved = 0x00;
  ptr->option.type4.ip = htonl(ip);
  ptr->option.type4.reserved = 0;
  ptr->option.type4.proto = matrix_config_[type][COL_PROTOCOL];
  ptr->option.type4.port = htons(port);
  
  return true;
}

bool SomeIPClient::SubscribePointCloud(uint8_t ttl) {
  std::string lidar_ip = matrix_config_string_[SD_POINT_CLOUD][COL_LIDAR_IP];
  // uint32_t lidar_port = matrix_config_[SD_POINT_CLOUD][COL_LIDAR_PORT];
  std::string host_ip = matrix_config_string_[SD_POINT_CLOUD][COL_HOST_IP];
  uint32_t host_port = matrix_config_[SD_POINT_CLOUD][COL_HOST_PORT];
  uint32_t sd_port = matrix_config_[SD_POINT_CLOUD][COL_SD_PORT];
  int udp_fd = -1;
  Open(udp_fd, sd_port);
  sockaddr_in addr2;
  memset(&addr2, 0, sizeof(sockaddr_in));
  // Prepare the sockaddr_in structure
  addr2.sin_family = AF_INET;
  addr2.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
  addr2.sin_port = htons(sd_port);

  uint32_t host_ip_uint = matrix_config_[SD_POINT_CLOUD][COL_HOST_IP];
  if (host_ip_uint == 0) {
    Close(udp_fd);
    return false;
  }
  SomeIPMessage msg;
  bool ret = InitSubscribeMessage(msg, SD_POINT_CLOUD, ttl, host_ip_uint, host_port);
  if (ret == false) {
    Close(udp_fd);
    return false;
  }
  int bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg), 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip point cloud subscribe message");
      Close(udp_fd);
      return false;
  }
  LogInfo("someip subscribe");
  Close(udp_fd);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));//sleep 100毫秒
  return true;
}

bool SomeIPClient::SubscribeFaultMessage(uint8_t ttl) {
  if (!isHavConfig(SD_FAULT_MSG)) return false;
  std::string lidar_ip = matrix_config_string_[SD_FAULT_MSG][COL_LIDAR_IP];
  std::string host_ip = matrix_config_string_[SD_FAULT_MSG][COL_HOST_IP];
  uint32_t host_port = matrix_config_[SD_FAULT_MSG][COL_HOST_PORT];
  uint32_t sd_port = matrix_config_[SD_FAULT_MSG][COL_SD_PORT];
  int udp_fd = -1;
  Open(udp_fd, sd_port);
  sockaddr_in addr2;
  memset(&addr2, 0, sizeof(sockaddr_in));
  // Prepare the sockaddr_in structure
  addr2.sin_family = AF_INET;
  addr2.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
  addr2.sin_port = htons(sd_port);

  uint32_t host_ip_uint = matrix_config_[SD_FAULT_MSG][COL_HOST_IP];
  if (host_ip_uint == 0) {
    Close(udp_fd);
    return false;
  }
  SomeIPMessage msg;
  bool ret = InitSubscribeMessage(msg, SD_FAULT_MSG, ttl, host_ip_uint, host_port);
  if (ret == false) {
    Close(udp_fd);
    return false;
  }
  int bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg), 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip fault message subscribe message");
      Close(udp_fd);
      return false;
  }
  LogInfo("someip fault message subscribe");
  
  Close(udp_fd);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));//sleep 100毫秒
  return true;

}
bool SomeIPClient::SomeipSubscribePointCloudAndFaultMessageIpv4() {
  bool ret_pc = SubscribePointCloud(0xFF);
  bool ret_fm = SubscribeFaultMessage(0xFF);
  if (ret_pc == false) return false;
  if (ret_fm == false) {
    LogError("someip fault message subscribe failed");
  }
  return true;
}

void SomeIPClient::InitSomeIpHeader(SomeIPHeader& header, uint32_t payload_len, std::string type, uint16_t methodId) {
  header.someip_service_id = htons(matrix_config_[type][COL_SERVICE_ID]);
  header.someip_method_id = htons(methodId);
  header.someip_length = htonl(payload_len + 8);
  header.someip_client_id = 0x0000;
  header.someip_session_id = 0x0000;
  header.someip_version = 0x01;
  header.someip_interface_version = 0x01;
  header.someip_message_type = 0x02;
  header.someip_return_code = 0x00;
}

bool SomeIPClient::InitRequestData(SomeIPMessage& message, int payload_len, std::string type, uint8_t counter) {
  InitSomeIpHeader(message.header, sizeof(E2EHeaderV6) + payload_len, type, matrix_config_[type][COL_METHOD_ID]);
  E2EHeaderV6* e2e_header = (E2EHeaderV6*)message.payload;
  e2e_header->counter = counter;
  e2e_header->len = htons(payload_len + sizeof(E2EHeaderV6) + 8);
  uint8_t data[40];
  int offset = 0;
  memcpy(data, (uint8_t *)&(message.header.someip_client_id), 8);
  memcpy(data + 8, (uint8_t *)(message.payload + 2), sizeof(E2EHeaderV6) - 2 + payload_len);
  offset = 8 + sizeof(E2EHeaderV6) - 2 + payload_len;
  uint16_t data_id = matrix_config_[type][COL_DATA_ID] & 0xFFFF;
  data[offset] = data_id / 0x100;
  data[offset + 1] = data_id % 0x100;
  offset += 2;
  uint16_t crc = crc16_ccitt_false(data, offset);
  e2e_header->crc = htons(crc);
  return true;
}


bool SomeIPClient::SomeipSubscribeServiceIpv4()
{
  if (!isHavConfig(SD_SERVICE)) return false;
  std::string lidar_ip = matrix_config_string_[SD_SERVICE][COL_LIDAR_IP];
  uint32_t lidar_port = matrix_config_[SD_SERVICE][COL_LIDAR_PORT];
  std::string host_ip = matrix_config_string_[SD_SERVICE][COL_HOST_IP];
  uint32_t host_port = matrix_config_[SD_SERVICE][COL_HOST_PORT];
  int udp_fd = -1;
  Open(udp_fd, host_port);
  sockaddr_in addr2;
  memset(&addr2, 0, sizeof(sockaddr_in));
  // Prepare the sockaddr_in structure
  addr2.sin_family = AF_INET;
  addr2.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
  addr2.sin_port = htons(host_port);

  uint32_t host_ip_uint = IPStringToUint32(host_ip);
  if (host_ip_uint == 0) {
    Close(udp_fd);
    return false;
  }
  SomeIPMessage msg;
  bool ret = InitSubscribeMessage(msg, SD_SERVICE, 0xFF, host_ip_uint, lidar_port);
  if (ret == false) return false;
  int bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg), 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip service subscribe message");
      Close(udp_fd);
      return false;
  }
  LogInfo("someip service subscribe");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));//sleep

  host_ip_uint = matrix_config_[SD_SERVICE][COL_MULTI_IP];//IPStringToUint32("239.23.0.3");
  if (host_ip_uint == 0) {
    Close(udp_fd);
    return false;
  }
  
  ret = InitSubscribeMessage(msg, SD_SERVICE_ACK, 0xFF, host_ip_uint, lidar_port);
  if (ret == false) {
    Close(udp_fd);
    return false;
  }
  bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg), 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip service subscribe message");
      Close(udp_fd);
      return false;
  }
  LogInfo("someip service ack subscribe");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));//sleep 
  is_service_sub_ = true;
  Close(udp_fd);
  return true;
}
#define ACL_ID_CORRECTION 1
#define ACL_ID_DCF        2
#define ACL_ID_FIRETIME   3

bool SomeIPClient::RecvACLData(std::vector<uint8_t> &acl_data, uint32_t &acl_id) {
  int udp_sock_tmp = -1;
  if (Open(udp_sock_tmp, matrix_config_[SD_ACL_SUBPACKET][COL_HOST_PORT]) == false) {
      LogError("Error opening socket");
      return false;
  }
#ifdef _MSC_VER  
    u_long mode = 0; // 0为阻塞模式  
    ioctlsocket(udp_sock_tmp, FIONBIO, &mode);
#else  
    int flags = fcntl(udp_sock_tmp, F_GETFL, 0); 
    fcntl(udp_sock_tmp, F_SETFL, flags & ~O_NONBLOCK); 
#endif 
  SetReceiveTimeout(udp_sock_tmp, 1000);

  int total_number = -1;
  int recv_number = 0;
  auto start_time = std::chrono::system_clock::now();
  
  while (1) {
    auto end_time = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                              start_time)
            .count() > 2000) {
      LogError("%s timeout", __func__);
      break;
    }
    sockaddr_in clientAddr;
    socklen_t addrLen = sizeof(sockaddr);
    uint8_t data[1500];
    int len = recvfrom(udp_sock_tmp, (char*)data, 1500, 0,
                        (sockaddr*)&clientAddr, &addrLen);
    int offset = 0;
    while (offset < len) {
      SomeIPHeader *ptr = (SomeIPHeader *)&data[offset];
      if (ptr->someip_service_id == htons(matrix_config_[SD_ACL_SUBPACKET][COL_SERVICE_ID])) {
        if (ptr->someip_method_id == htons(matrix_config_[SD_ACL_SUBPACKET][COL_METHOD_ID])) {
          uint8_t *data_ptr = (uint8_t *)ptr + sizeof(SomeIPHeader) + sizeof(E2EHeaderV6);
          uint8_t index = data_ptr[0];
          acl_id = data_ptr[4];
          if (total_number == -1) {
            if (index != 1) {
              LogWarning("not first number %u, total %d, acl_id %d", index, data_ptr[1], acl_id);
              break;
            }
            total_number = data_ptr[1];
            acl_data.resize(total_number * 1024);
          }
          uint16_t valid_data_len = data_ptr[2] + data_ptr[3] * 0x100;
          
           // in ncd, it is a uint32 type, and is little endian
          if (acl_id == ACL_ID_CORRECTION && (valid_data_len != 1024 || ntohl(ptr->someip_length) != 1045)) {
            if (!(index == total_number && valid_data_len < 1024)) {
              LogWarning("recv valid_data_len error %u  %u", valid_data_len, ntohl(ptr->someip_length));
              break;
            }
          }
          LogInfo("someip recv acl %u/%u, %u, %d", index, total_number, valid_data_len, acl_id);
          memcpy(acl_data.data() + (index - 1) * 1024, data_ptr + 8, valid_data_len);
          acl_data.resize(total_number * 1024 - 1024 + valid_data_len);
          recv_number++;
          if (index == total_number) {
            LogInfo("someip recv acl success %d", acl_id);
            break;
          }
        }
        // else if (ptr->someip_method_id == htons(matrix_config_[SD_ACL_TRANSMIT][COL_METHOD_ID])) {
        //   uint8_t *data_ptr = (uint8_t *)ptr + sizeof(SomeIPHeader) + sizeof(E2EHeaderV6);
        //   if (data_ptr[0] == 0x03 || data_ptr[0] == 0x04) {
        //     LogInfo("someip recv acl status %u, exit", data_ptr[0]);
        //     len = -1;
        //     break;
        //   }
        // }
        offset += 8 + ntohl(ptr->someip_length);
      }
      else break;
    }
    if (len <= 0) break;
    if (total_number != -1 && total_number == recv_number) {
      switch(acl_id) {
        case ACL_ID_CORRECTION : LogInfo("someip recv correction data success, size %lu", acl_data.size()); break;
        case ACL_ID_FIRETIME   : LogInfo("someip recv firetime data success size %lu", acl_data.size());   break;
        case ACL_ID_DCF        : LogInfo("someip recv dcf data success size %lu", acl_data.size());        break;
        default                : LogError("someip acl error %d", acl_id);        break;
      }
      break;
    }
    
  }
  if (total_number == -1 || total_number != recv_number) {
    LogError("someip recv acl data error, %d/%d", recv_number, total_number);
  }
  Close(udp_sock_tmp);
  return true;
} 

bool SomeIPClient::GetCorrectionData() {
  if (!isHavConfig(SD_ACL_SET)) return false;
  std::string lidar_ip = matrix_config_string_[SD_ACL_SET][COL_LIDAR_IP];
  std::string host_ip = matrix_config_string_[SD_ACL_SET][COL_HOST_IP];
  uint32_t host_port = matrix_config_[SD_ACL_SET][COL_HOST_PORT];
  int udp_fd = -1;
  Open(udp_fd, host_port);
  sockaddr_in addr2;
  memset(&addr2, 0, sizeof(sockaddr_in));
  // Prepare the sockaddr_in structure
  addr2.sin_family = AF_INET;
  addr2.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
  addr2.sin_port = htons(matrix_config_[SD_ACL_SET][COL_SD_PORT]);

  if (is_service_sub_ == false) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));//sleep 
    if (SomeipSubscribeServiceIpv4() == false) {
      LogError("Error subscribe someip service");
      Close(udp_fd);
      return false;
    }
  }
  std::vector<uint8_t> acl_data;
  uint32_t acl_id = 0;
  RecvACLData(acl_data, acl_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));//sleep 
  SomeIPMessage msg;
  msg.payload[sizeof(E2EHeaderV6)] = 0x01;
  InitRequestData(msg, 1, SD_ACL_SET, 1);
  int bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg.header) + sizeof(E2EHeaderV6) + 1, 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip 0x8008 0x01");
      Close(udp_fd);
      return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));//sleep 
  msg.payload[sizeof(E2EHeaderV6)] = 0x02;
  InitRequestData(msg, 1, SD_ACL_SET, 2);
  bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg.header) + sizeof(E2EHeaderV6) + 1, 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip 0x8008 0x02");
      Close(udp_fd);
      return false;
  }
  
  correction_data_ = acl_data;
  
  Close(udp_fd);
  return true;
}

/*
ttl = 0, means stop subscribe
ttl = 0xff, means subscribe
*/
bool SomeIPClient::SubscribeACLData(uint8_t ttl) {
  if (!isHavConfig(SD_ACL_SET)) return false;
  std::string lidar_ip = matrix_config_string_[SD_ACL_SET][COL_LIDAR_IP];
  uint16_t lidar_port = matrix_config_[SD_ACL_SET][COL_LIDAR_PORT];
  std::string host_ip = matrix_config_string_[SD_ACL_SET][COL_HOST_IP];
  uint32_t host_port = matrix_config_[SD_ACL_SET][COL_HOST_PORT];
  int udp_fd = -1;
  Open(udp_fd, host_port);
  sockaddr_in addr2;
  memset(&addr2, 0, sizeof(sockaddr_in));
  // Prepare the sockaddr_in structure
  addr2.sin_family = AF_INET;
  addr2.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
  addr2.sin_port = htons(matrix_config_[SD_ACL_SET][COL_SD_PORT]);

  uint32_t host_ip_uint = matrix_config_[SD_ACL_SET][COL_HOST_IP];
  if (host_ip_uint == 0) {
    Close(udp_fd);
    return false;
  }
  SomeIPMessage msg;
  bool ret = InitSubscribeMessage(msg, SD_ACL_TRANSMIT, ttl, host_ip_uint, lidar_port);
  if (ret == false) {
    Close(udp_fd);
    return false;
  }
  int bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg), 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip point cloud subscribe message");
      Close(udp_fd);
      return false;
  }
  Close(udp_fd);
  LogInfo("someip acl subscribe");
  return true;
}

bool SomeIPClient::GetACLData()
{
  if (SubscribeACLData(0xFF) == false) return false;
  
  uint32_t acl_id = 0;
  for (int i = 0; i < 3; i++) {
    std::vector<uint8_t> acl_data;
    acl_id = 0;
    RecvACLData(acl_data, acl_id);
    if (acl_id == ACL_ID_CORRECTION) {
      correction_data_ = acl_data;
    } else if(acl_id == ACL_ID_DCF) {
      dcf_data_ = acl_data;
    } else if (acl_id == ACL_ID_FIRETIME) {
      firetime_data_ = acl_data;
    }
  }
  return true;
}

bool SomeIPClient::SetLidarStatus(uint8_t status) {
  if (!isHavConfig(SD_RUN_MODE_SET)) return false;
  std::string lidar_ip = matrix_config_string_[SD_RUN_MODE_SET][COL_LIDAR_IP];
  std::string host_ip = matrix_config_string_[SD_RUN_MODE_SET][COL_HOST_IP];
  uint32_t host_port = matrix_config_[SD_RUN_MODE_SET][COL_HOST_PORT];
  int udp_fd = -1;
  Open(udp_fd, host_port);
  sockaddr_in addr2;
  memset(&addr2, 0, sizeof(sockaddr_in));

  // Prepare the sockaddr_in structure
  addr2.sin_family = AF_INET;
  addr2.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
  addr2.sin_port = htons(host_port);

  if (is_service_sub_ == false) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));//sleep 
    if (SomeipSubscribeServiceIpv4() == false) {
      LogError("Error subscribe someip service");
      Close(udp_fd);
      return false;
    }
  }

  static uint8_t counter = 1; 
  std::this_thread::sleep_for(std::chrono::milliseconds(200));//sleep 
  SomeIPMessage msg;
  msg.payload[sizeof(E2EHeaderV6)] = status;
  InitRequestData(msg, 1, SD_RUN_MODE_SET, counter);

  int bytes_sent = sendto(udp_fd, reinterpret_cast<char*>(&msg), sizeof(msg.header) + sizeof(E2EHeaderV6) + 1, 0, reinterpret_cast<const sockaddr*>(&addr2), sizeof(sockaddr_in));
  if (bytes_sent == SOCKET_ERROR) {
      LogError("Error sending someip %x %d", matrix_config_[SD_RUN_MODE_SET][COL_METHOD_ID], counter);
      Close(udp_fd);
      return false;
  }
  LogInfo("set lidar status %u successfully", status);
  counter++;
  Close(udp_fd);
  return true;
}


bool SomeIPClient::GetSomeipMatrixConfig(std::string config_file_path)
{
  matrix_config_string_ = ReadCSVMapContent(config_file_path);
  if (!isHavConfig(SD_POINT_CLOUD)) {
    return false;
  }
  for (auto &item : matrix_config_string_) {
    if (item.first == "name") {
      // std::cout << "item.first " << item.first << std::endl;
      continue;
    }
    for (auto kv : item.second) {
      if (kv.second.find(".") != std::string::npos) {
        matrix_config_[item.first][kv.first] = IPStringToUint32(kv.second);
      } else if (kv.second.find("0x") != std::string::npos || kv.second.find("0X") != std::string::npos) {
        matrix_config_[item.first][kv.first] = HexStringToUint16Stoi(kv.second);
      } else {
        matrix_config_[item.first][kv.first] = kv.second.empty() ? 0 : std::stoi(kv.second);
      }
      LogDebug("%-16s %-16s: %x\n", item.first.c_str(), kv.first.c_str(), matrix_config_[item.first][kv.first]);
    }
  }
  return true;
}

uint32_t SomeIPClient::GetSomeipCusType() {
  uint32_t spec = matrix_config_[SD_FOR_CONFIG][COL_SPEC];
  return (spec & SPEC_CUS_TYPE_MASK);
}

bool SomeIPClient::IsMulticastSpec() {
  uint32_t spec = matrix_config_[SD_FOR_CONFIG][COL_SPEC];
  return ((spec & SPEC_IP_USE_CONFIG_MASK) >> 8) == SPEC_USE_MULTICAST;
}

bool SomeIPClient::GetIPPort(std::string name, std::string &lidar_ip, uint32_t& lidar_port, std::string& host_ip, uint32_t& host_port, std::string &multi_ip, uint32_t& multi_port) { 
  if (!isHavConfig(name)) return false;
  lidar_ip = matrix_config_string_[name][COL_LIDAR_IP];
  lidar_port = matrix_config_[name][COL_LIDAR_PORT];
  host_ip = matrix_config_string_[name][COL_HOST_IP];
  host_port = matrix_config_[name][COL_HOST_PORT];
  multi_ip = matrix_config_string_[name][COL_MULTI_IP];
  multi_port = matrix_config_[name][COL_MULTI_PORT];
  return true;
}

bool SomeIPClient::isHavConfig(std::string name) {
  if (matrix_config_string_.find(name) == matrix_config_string_.end()) {
    LogError("Error: %s not found in someip config", name.c_str());
    return false;
  }
  return true;
}