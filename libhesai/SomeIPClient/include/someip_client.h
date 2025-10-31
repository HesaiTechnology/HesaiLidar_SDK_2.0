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
#include "inner_com.h"
#include "iostream"
#include <cstdint>
#include <cstring>
#ifdef _MSC_VER
#include <winsock2.h>
#include <ws2tcpip.h> 
#pragma comment(lib, "ws2_32.lib")  // Winsock Library
#include <BaseTsd.h>
typedef int socklen_t;
#else
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
typedef unsigned int SOCKET;
#define SOCKET_ERROR -1
#endif
#include <csv_reader.hpp>
namespace hesai
{
namespace lidar
{
#define SOMEIP_PORT 30490
#define SOMEIP_SERVICE_PORT 40000

#pragma pack(push, 1)
struct SomeIPHeader {
    uint16_t someip_service_id;
    uint16_t someip_method_id;
    uint32_t someip_length;
    uint16_t someip_client_id;
    uint16_t someip_session_id;
    uint8_t someip_version;
    uint8_t someip_interface_version;
    uint8_t someip_message_type;
    uint8_t someip_return_code;
};
static_assert(sizeof(SomeIPHeader) == 16);
struct SomeIPMessage {
    SomeIPHeader header;
    char payload[40];
};

struct SomeIpSD {
  uint8_t flags;
  uint8_t reserved[3];
  uint32_t entry_length;
  union Entry {
    struct Service {
      uint8_t type;
      uint8_t first_option_run;
      uint8_t second_option_run;
      uint8_t options;
      uint16_t service_id;
      uint16_t instance_id;
      uint8_t major_version;
      uint8_t TTL[3];
      uint32_t minor_version;
    } service;
    struct EventGroup {
      uint8_t type;
      uint8_t first_option_run;
      uint8_t second_option_run;
      uint8_t options;
      uint16_t service_id;
      uint16_t instance_id;
      uint8_t major_version;
      uint8_t TTL[3];
      uint8_t reserved;
      uint8_t flag_counter; // 1(inital data requested flag) + 3(reserved) + 4(counter)
      uint16_t eventgroup_id;
    } eventgroup;
  } entry;  
  uint32_t option_length;
  union Option {
    struct Option_0x1 {
      uint16_t len;
      uint8_t type = 0x01;
      uint8_t discardable_flag_reserved;
      // payload
    } type1;
    struct Option_0x2 {
      uint16_t len = htons(0x0005);
      uint8_t type = 0x02;
      uint8_t discardable_flag_reserved;
      uint16_t priority;
      uint16_t weight;
    } type2;
    struct Option_0x4 {
      uint16_t len = htons(0x0009);
      uint8_t type = 0x04;
      uint8_t discardable_flag_reserved;
      uint32_t ip;
      uint8_t reserved;
      uint8_t proto;
      uint16_t port;
    } type4;
    struct Option_0x14 {
      uint16_t len = htons(0x0009);
      uint8_t type = 0x14;
      uint8_t discardable_flag_reserved;
      uint32_t ip;
      uint8_t reserved;
      uint8_t proto;
      uint16_t port;
    } type14;
  } option;
};
static_assert(sizeof(SomeIpSD) == 40);

struct E2EHeaderV6 {
  uint16_t crc;
  uint16_t len;
  uint8_t counter;
};

#pragma pack(pop)

enum {
  POINT_CLOUD = 0,
  FAULT_MESSAGE = 1,
  SERVICE = 2,
  SERVICE_ACK = 3,
  PARAM_MATRIX_SIZE
};
struct ParamMatrix {
  std::string name;
  uint16_t service_id;
  uint16_t instance_id;
  uint8_t entry_type;
  uint16_t eventgroup_id;
  uint8_t option_type;
  uint8_t protocol;
  uint8_t sd_flag;   
}
static const ParamMatrix[PARAM_MATRIX_SIZE] = {
//     name      service_id  instance_id entry_type eventgroup_id option_type protocol sd_flag
  {"pointCloud",   0xa2a0,      0x000a,     0x06,     0x0001,       0x04,        0x11,   0x40},
  {"faultCloud",   0xa29e,      0x000a,     0x06,     0x0001,       0x04,        0x11,   0x40},
  {"service",      0xa2a3,      0x000a,     0x01,     0x0000,       0x04,        0x11,   0x40},
  {"service_ack",  0xa2a3,      0x000a,     0x07,     0x0001,       0x04,        0x11,   0x40},
};

#define SD_SERVICE        "service"
#define SD_SERVICE_ACK    "service_ack"
#define SD_POINT_CLOUD    "pointcloud"
#define SD_ACL_SUBPACKET  "acl_subpacket"
#define SD_ACL_TRANSMIT   "acl_transmit"
#define SD_FAULT_MSG      "faultmsg"
#define SD_ACL_SET        "acl_set"
#define SD_RUN_MODE_SET   "run_mode_set"
#define SD_RUN_MODE_GET   "run_mode_get"
#define SD_WIN_HEAT_SET   "windows_heat_set"
#define SD_WIN_HEAT_GET   "windows_heat_get"
#define SD_FOR_CONFIG     SD_POINT_CLOUD


#define COL_NAME          "name"
#define COL_METHOD_ID     "method_id"
#define COL_HOST_IP       "host_ip"
#define COL_HOST_PORT     "host_port"
#define COL_LIDAR_IP      "lidar_ip"
#define COL_LIDAR_PORT    "lidar_port"
#define COL_MULTI_IP      "multi_ip"
#define COL_MULTI_PORT    "multi_port"
#define COL_SERVICE_ID    "service_id"
#define COL_INSTANCE_ID   "instance_id"
#define COL_ENTRY_TYPE    "entry_type"
#define COL_EVENTGROUP_ID "eventgroup_id"
#define COL_OPTION_TYPE   "option_type"
#define COL_PROTOCOL      "protocol"
#define COL_SD_FLAG       "sd_flag"
#define COL_DATA_ID       "data_id"
#define COL_SPEC          "spec"
#define COL_SD_PORT       "someip_port"

#define MODE_STANDBY     1
#define MODE_ACTIVE      2
#define MODE_LOW_POWER   3

#define SPEC_CUS_TYPE_MASK 0xFF
#define SPEC_IP_USE_CONFIG_MASK 0xFF00

#define SPEC_USE_MULTICAST 0x01

class SomeIPClient {
public:
  SomeIPClient(const uint16_t port, const std::string& multicastIp);
  ~SomeIPClient();
  bool Open();
  bool Open(int&, uint16_t, uint8_t udp_buf_type = 0);
  void Close();
  void Close(int& socket);
  bool IsOpened();
  bool SomeipSubscribePointCloudAndFaultMessageIpv4();
  bool SomeipSubscribeServiceIpv4();
  bool GetCorrectionData();
  bool GetACLData();
  std::vector<uint8_t> correction_data_;
  std::vector<uint8_t> firetime_data_;
  std::vector<uint8_t> dcf_data_;
  bool SetLidarStatus(uint8_t status);
  bool GetSomeipMatrixConfig(std::string config_file_path);
  bool RecvACLData(std::vector<uint8_t> &acl_data, uint32_t &acl_id);
  uint32_t GetSomeipCusType();
  bool IsMulticastSpec();
  bool GetIPPort(std::string name, std::string &lidar_ip, uint32_t& lidar_port, std::string& host_ip, uint32_t& host_port, std::string &multi_ip, uint32_t& multi_port);
  inline bool isHavConfig(std::string name);
protected:
  bool SetReceiveTimeout(int sock, uint32_t u32Timeout);
  void InitSomeIpSDHeader(SomeIPHeader& header, uint32_t payload_len);
  bool InitSubscribeMessage(SomeIPMessage& message, std::string type, uint8_t ttl, uint32_t ip, uint16_t port);
  bool SubscribeACLData(uint8_t ttl); 
  bool SubscribePointCloud(uint8_t ttl);
  bool SubscribeFaultMessage(uint8_t ttl);
  void InitSomeIpHeader(SomeIPHeader& header, uint32_t payload_len, std::string type, uint16_t methodId);
  bool InitRequestData(SomeIPMessage& message, int payload_len, std::string type, uint8_t counter);
  uint16_t udp_port_;
  int udp_sock_;
  static const int32_t kUDPBufferSize = 26214400;  // udp buffer size
  std::string multicast_ip_;
  std::string lidar_ip_;
  uint16_t udp_port_send_;
  sockaddr_in send_addr_;
  bool is_service_sub_ = false;
  std::map<std::string, std::map<std::string, std::string>> matrix_config_string_;
  std::map<std::string, std::map<std::string, uint32_t>> matrix_config_;
};

} // lidar
} // hesai