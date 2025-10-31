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
 * File:   ptc_client.h
 * Author: Felix Zou<zouke@hesaitech.com>
 *
 * Created on Jun 20, 2019, 10:46 AM
 */

#ifndef PtcClient_H
#define PtcClient_H

#include <vector>
#include "tcp_client.h"
#include "driver_param.h"
#include "ptc_parser.h"
#include "byte_printer.h"
#ifdef WITH_PTCS_USE
#include "tcp_ssl_client.h"
#endif

namespace hesai
{
namespace lidar
{
const uint8_t  kPTCGetLidarCalibration = 0x05;
const uint8_t  kPTCGetPTPDiagnostics= 0x06;
const uint8_t  kPTCGetInventoryInfo = 0x07;
const uint8_t  kPTCGetLidarStatus = 0x09;
const uint8_t  kPTCGetFpgaRegister = 0x0C;
const uint8_t  kPTCSetFpgaRegister = 0x0D;
const uint8_t  kPTCRebootLidar = 0x10;
const uint8_t  kPTCSetSpinSpeed = 0x17;
const uint8_t  kPTCSetSyncAngle = 0x18;
const uint8_t  kPTCSetReturnMode = 0x1E;
const uint8_t  kPTCSetStandbyMode = 0x1C;
const uint8_t  kPTCSetDestinationIPandPort = 0x20;
const uint8_t  kPTCSetNet = 0x21;
const uint8_t  KPTCSetFov = 0x22;
const uint8_t  KPTCGetFov = 0x23;
const uint8_t  kPTCResetLidar = 0x25;
const uint8_t  kPTCGetFreezeFrames = 0x30;
const uint8_t  kPTCGetLidarOperationLog = 0x38;
const uint8_t  kPTCGetPTPLockOffset = 0x3a;
const uint8_t  kPTCUpgradeLidar = 0x83;
const uint8_t  kPTCGetUpgradeLidarLog = 0x87;
const uint8_t  kPTCGetLidarChannelConfig = 0xA8;
const uint8_t  kPTCGetLidarFiretimes = 0xA9;
const uint32_t kPTCSetTemFpgaRegister = 0x00010031;
const uint32_t kPTCUpgradeLidarSubCmd = 0x0000000D;
const uint32_t kPTCGetRetroSubCmd = 0x0000011E;

typedef struct UpgradeProgress {
    int total_packets;
    int current_packet;
    int status;
    int error_code;
} UpgradeProgress;
typedef void *(*UpgradeProgressFunc_t)(void *);

class PtcClient {
 public:
  using Mutex = std::mutex;
  using LockS = std::lock_guard<Mutex>;
  using CallbackType = std::function<void(const std::string&)>; // 函数指针 std::function
 public:
  PtcClient(std::string IP = kLidarIPAddr
            , uint16_t u16TcpPort = kTcpPort
            , bool bAutoReceive = false
            , PtcMode client_mode = PtcMode::tcp
            , uint8_t ptc_version = 1
            , std::string cert = ""
            , std::string private_key = ""
            , std::string ca = ""
            , uint32_t u32RecvTimeoutMs = 500
            , uint32_t u32SendTimeoutMs = 500
            , float ptc_connect_timeout = -1
            , uint16_t u16HostPort = 0);
  ~PtcClient();

  PtcClient(const PtcClient &orig) = delete;
  PtcClient& operator=(const PtcClient&) = delete;

  bool IsValidRsp(u8Array_t &byteStreamIn);
  bool IsOpen();

  void TcpFlushIn();
  void TryOpen();
  int QueryCommand(u8Array_t &byteStreamIn, u8Array_t &byteStreamOut, uint8_t u8Cmd, bool isHaveHeader = false);
  int SendCommand(u8Array_t &byteStreamIn, uint8_t u8Cmd);
  bool GetValFromOutput(uint8_t cmd, uint8_t retcode, const u8Array_t &payload, int start_pos, int length, u8Array_t &res);

  u8Array_t GetCorrectionInfo();
  int GetLidarStatus();
  int GetPTPDiagnostics (u8Array_t &dataOut, uint8_t query_type);
  int GetPTPLockOffset(u8Array_t &dataOut);
  int GetCorrectionInfo(u8Array_t &dataOut);
  int GetFiretimesInfo(u8Array_t &dataOut);
  int GetChannelConfigInfo(u8Array_t &dataOut);
  int SetSocketTimeout(uint32_t u32RecMillisecond, uint32_t u32SendMillisecond);

  int UpgradeLidar(u8Array_t &dataIn);
  int UpgradeLidar(u8Array_t &dataIn, std::string Cmd_id, int &upgradeProgress);
  int UpgradeLidar(u8Array_t &dataIn, uint32_t cmd_id, int is_extern, int &upgrade_progress);
  void RegisterUpgradeProcessFunc(UpgradeProgressFunc_t func);
  bool RebootLidar();
  int GetOperationLog(int module, int type, u8Array_t &dataOut);
  int GetFreezeFrames(u8Array_t &dataOut);
  int GetUpgradeLidarLog(u8Array_t &dataOut);
  int SetAllChannelFov(float fov[], int fov_num, int fov_model);
  int GetAllChannelFov(float fov[], int& fov_num, int& fov_model);
  /**
   * @brief upgrade lidar patch
   * 
   * @param file_path                   lidar patch file path
   * @param cmd_id                      ptc command id, like 0x83, 0x0000000D
   * @param is_extern                   Is extern command, like 0x0000000D is extern command, 0x83 is not extern command
   * @return int                        0: success, -1: fail
   * @param upgrade_progress            Upgrade progress
   * @return int                        0: success, -1: fail
   */  
  int UpgradeLidarPatch(const std::string &file_path, uint32_t cmd_id, int is_extern);  

  /**
   * @brief Set the lidar net
   * 
   * @param IP                          Expected lidar IP
   * @param mask                        Expected lidar mask
   * @param getway                      Expected lidar getwag
   * @param vlan_flag                   Expected lidar valan_flag [1 0] If you want set it vlan_flag should be set 1
   * @param vlan_ID                     Expected lidar valan_ID
   * @return true                       Set successful
   * @return false                      Set faild
   */
  bool SetNet(std::string IP, std::string mask, std::string getway, uint8_t vlan_flag, uint16_t vlan_ID);

  /**
   * @brief Set the destination Ip and Port and gps_port
   * 
   * @param des                         Expected destination_IP  
   * @param port                        Expected destination_PORT
   * @param GPS_port                    Expected GPS_port
   * @return true                       Set successful 
   * @return false                      Set faild
   */
  bool SetDesIpandPort(std::string des, uint16_t port, uint16_t GPS_port);

  /**
   * @brief Set the Return_Mode of lidar
   * 
   * @param return_mode                Expected return_mode of lidar              
   * @return true                      Set successful
   * @return false                     Set faild
   */
  bool SetReturnMode(uint8_t return_mode);

  /**
   * @brief Set the syncangle of lidar
   * 
   * @param enable_flag                Expected lidar enable_flag [1 0] If you want set it enable_flag should be set 1        
   * @param sync_angle                 Expected syncangle
   * @return true                      Set successful
   * @return false                     Set faild
   */
  bool SetSyncAngle(uint8_t enable_flag, uint16_t sync_angle);

  /**
   * @brief Set the TmbFPGARegister, it is worth noting that this setting is volatile
   * 
   * @param address                    Address of register
   * @param data                       The data you want to wirte
   * @return true                      Set successful
   * @return false                     Set faild
   */
  bool SetTmbFPGARegister(uint32_t address, uint32_t data);

   /**
   * @brief Get the FPGARegister, it is worth noting that this setting is volatile
   * 
   * @param address                    Address of register
   * @param data                       The data you want to wirte
   * @return true                      Get successful
   * @return false                     Get faild
   */
  bool GetFPGARegister(uint32_t address, uint32_t &data);
   /**
   * @brief Set the FPGARegister, it is worth noting that this setting is volatile
   * 
   * @param address                    Address of register
   * @param data                       The data you want to wirte
   * @return true                      Set successful
   * @return false                     Set faild
   */
  bool SetFPGARegister(uint32_t address, uint32_t data);

  /**
   * @brief Set the standby_mode of lidar
   * 
   * @param standby_mode              Expected standby_mode [0:in operation  1:standby_mode]
   * @return true                     Set successful
   * @return false                    Set faild
   */
  bool SetStandbyMode(uint32_t standby_mode);

  /**
   * @brief Set the spin_speed of lidar
   * 
   * @param speed                     Expected spinspeed, it is important to note that you must fill in the RPM supported by the radar you are using
   * @return true 
   * @return false 
   */
  bool SetSpinSpeed(uint32_t speed);

  void CRCInit();
  uint32_t CRCCalc(uint8_t *bytes, int len); 
  void SetCallback(CallbackType callback) { this->log_message_handler_callback_ = callback; }
  void ProduceLogMessage(const std::string& message); 

 public:
  void SetLidarIP(std::string);
  void SetLidarIP(uint32_t);
  uint32_t m_CRCTable[256];       
  bool InitOpen;                       
  int ret_code_;                

 private:
  Mutex _mutex;
  static const std::string kLidarIPAddr;
  static const uint16_t kTcpPort = 9347;
  uint16_t m_u16PtcPort;
  PtcMode client_mode_;
  uint8_t ptc_version_;
  std::shared_ptr<ClientBase> client_;
  std::shared_ptr<PtcParser> ptc_parser_;
  std::thread *open_thread_ptr_;
  std::string lidar_ip_;
  uint16_t tcp_port_;
  uint16_t host_port_;
  bool auto_receive_;
  std::string cert_;
  std::string private_key_;
  std::string ca_;
  uint32_t recv_timeout_ms_;
  uint32_t send_timeout_ms_;
  float ptc_connect_timeout_;
  UpgradeProgressFunc_t upgradeProcessFunc;
  CallbackType log_message_handler_callback_ = nullptr;
};
}
}

#endif /* PtcClient_H */