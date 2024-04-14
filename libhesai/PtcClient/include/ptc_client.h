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
 * File:   ptc_client.h
 * Author: Felix Zou<zouke@hesaitech.com>
 *
 * Created on Jun 20, 2019, 10:46 AM
 */

#ifndef PtcClient_H
#define PtcClient_H

#include <vector>
#include "tcp_client.h"
#include "tcp_ssl_client.h"
#include "lidar_types.h"
#include "driver_param.h"
#include "ptc_parser.h"

#define PKT_SIZE_40P (1262)
#define PKT_SIZE_AC  (1256)
#define PKT_SIZE_64  (1194)
#define PKT_SIZE_20  (1270)

namespace hesai
{
namespace lidar
{

const uint8_t  kPTCGetPTPLockOffset = 0x3a;
const uint8_t  kPTCGetLidarStatus = 0x09;
const uint8_t  kPTCGetPTPDiagnostics= 0x06;
const uint8_t  kPTCGetLidarCalibration = 0x05;
const uint8_t  kPTCGetInventoryInfo = 0x07;
const uint8_t  kPTCGetLidarFiretimes = 0xA9;
const uint8_t  kPTCGetLidarChannelConfig = 0xA8;
const uint8_t  kPTCSetNet = 0x21;
const uint8_t  kPTCSetDestinationIPandPort = 0x20;
const uint8_t  kPTCSetReturnMode = 0x1E;
const uint8_t  kPTCSetSyncAngle = 0x18;
const uint8_t  kPTCSetStandbyMode = 0x1C;
const uint8_t  kPTCSetSpinSpeed = 0x17;
const uint32_t kPTCSetTemFpgaRegister = 0x00010031;
const uint8_t  kPTCSetFpgaRegister = 0x0D;

class PtcClient {
 public:
  PtcClient(std::string IP = kLidarIPAddr
            , uint16_t u16TcpPort = kTcpPort
            , bool bAutoReceive = false
            , PtcMode client_mode = PtcMode::tcp
            , uint8_t ptc_version = 1
            , const char* cert = nullptr
            , const char* private_key = nullptr
            , const char* ca = nullptr
            , uint32_t u32RecvTimeoutMs = 500
            , uint32_t u32SendTimeoutMs = 500);
  ~PtcClient() {}

  PtcClient(const PtcClient &orig) = delete;

  bool IsValidRsp(u8Array_t &byteStreamIn);

  void TcpFlushIn();
  int QueryCommand(u8Array_t &byteStreamIn, u8Array_t &byteStreamOut, uint8_t u8Cmd );
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

 public:
  uint32_t m_CRCTable[256];                                              

 private:
  static const std::string kLidarIPAddr;
  static const uint16_t kTcpPort = 9347;
  uint16_t m_u16PtcPort;
  bool running_;
  PtcMode client_mode_;
  uint8_t ptc_version_;
  std::shared_ptr<ClientBase> client_;
  std::shared_ptr<PtcParser> ptc_parser_;
};
}
}

#endif /* PtcClient_H */