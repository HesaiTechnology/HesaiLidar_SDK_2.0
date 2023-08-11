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
#ifndef FAULT_MESSAGE_H
#define FAULT_MESSAGE_H
#include "lidar_types.h"

#pragma pack(1)
struct FaultMessageVersion3 {
 public:
  uint16_t sob;
  uint8_t version_info;
  uint8_t utc_time[6];
  uint32_t time_stamp;
  uint8_t operate_state;
  uint8_t fault_state;
  uint8_t fault_code_type;
  uint8_t rolling_counter;
  uint8_t total_fault_code_num;
  uint8_t fault_code_id;
  uint32_t fault_code;
  uint8_t time_division_multiplexing[27];
  uint8_t software_version[8];
  uint8_t heating_state;
  uint8_t lidar_high_temp_state;
  uint8_t reversed[3];
  uint32_t crc;
  uint8_t cycber_security[32];
  DTCState ParserDTCState() {
    switch (fault_code & 0x01) {
      case 1: {
        return kFault;
        break;
      }
      case 0: {
        return kNoFault;
        break;
      }

      default:
        break;
    }
    return kNoFault;
  }
  LidarOperateState ParserOperateState() {
    switch (operate_state) {
      case 0:
        return kBoot;
        break;
      case 1:
        return kInit;
        break;
      case 2:
        return kFullPerformance;
        break;
      case 3:
        return kHalfPower;
        break;
      case 4:
        return kSleepMode;
        break;
      case 5:
        return kHighTempertureShutdown;
        break;
      case 6:
        return kFaultShutdown;
        break;

      default:
        return kUndefineOperateState;
        break;
    }
  }
  LidarFaultState ParserFaultState() {
    switch (fault_state) {
      case 0:
        return kNormal;
        break;
      case 1:
        return kWarning;
        break;
      case 2:
        return kPrePerformanceDegradation;
        break;
      case 3:
        return kPerformanceDegradation;
        break;
      case 4:
        return kPreShutDown;
        break;
      case 5:
        return kShutDown;
        break;
      case 6:
        return kPreReset;
        break;
      case 7:
        return kReset;
        break;

      default:
        return kUndefineFaultState;
        break;
    }
  }
  FaultCodeType ParserFaultCodeType() {
    switch (fault_code_type) {
      case 1:
        return kCurrentFaultCode;
        break;
      case 2:
        return kHistoryFaultCode;
        break;

      default:
        return kUndefineFaultCode;
        break;
    }
  }
  TDMDataIndicate ParserTDMDataIndicate() {
    switch (time_division_multiplexing[0]) {
      case 0:
        return kInvaild;
        break;
      case 1:
        return kLensDirtyInfo;
        break;

      default:
        return kUndefineIndicate;
        break;
    }
  }
  void ParserLensDirtyState(
      LensDirtyState lens_dirty_state[LENS_AZIMUTH_AREA_NUM]
                                   [LENS_ELEVATION_AREA_NUM]) {
    for (int i = 0; i < LENS_AZIMUTH_AREA_NUM; i++) {
      uint16_t rawdata =
          (*((uint16_t *)(&time_division_multiplexing[3 + i * 2])));
      for (int j = 0; j < LENS_ELEVATION_AREA_NUM; j++) {
        uint16_t lens_dirty_state_temp =
            (rawdata << ((LENS_ELEVATION_AREA_NUM - j - 1) * 2));
        uint16_t lens_dirty_state_temp1 =
            (lens_dirty_state_temp >> ((LENS_ELEVATION_AREA_NUM - 1) * 2));
        if (time_division_multiplexing[0] == 1) {
          switch (lens_dirty_state_temp1) {
            case 0: {
              lens_dirty_state[i][j] = kLensNormal;
              break;
            }
            case 1: {
              lens_dirty_state[i][j] = kPassable;
              break;
            }
            case 3: {
              lens_dirty_state[i][j] = kUnPassable;
              break;
            }
            default:
              lens_dirty_state[i][j] = kUndefineData;
              break;
          }

        } else
          lens_dirty_state[i][j] = kUndefineData;
      }
    }
  }
  HeatingState ParserHeatingState() {
    switch (heating_state) {
      case 0:
        return kOff;
        break;
      case 1:
        return kHeating;
        break;
      case 2:
        return kHeatingProhibit;
        break;

      default:
        break;
    }
    return kUndefineHeatingState;
  }
  HighTempertureShutdownState ParserHighTempertureShutdownState() {
    switch (lidar_high_temp_state) {
      case 1:
        return kPreShutdown;
        break;
      case 2:
        return kShutdownMode1;
        break;
      case 6:
        return kShutdownMode2;
        break;
      case 10:
        return kShutdownMode2Fail;
        break;
      default:
        break;
    }
    return kUndefineShutdownData;
  }
  void ParserFaultMessage(FaultMessageInfo &fault_message_info) {
    fault_message_info.version = version_info;
    memcpy(fault_message_info.utc_time, utc_time, sizeof(utc_time));
    double unix_second = 0;
    if (utc_time[0] != 0) {
      struct tm t = {0};
      t.tm_year = utc_time[0];
      if (t.tm_year >= 200) {
        t.tm_year -= 100;
      }
      t.tm_mon = utc_time[1] - 1;
      t.tm_mday = utc_time[2];
      t.tm_hour = utc_time[3];
      t.tm_min = utc_time[4];
      t.tm_sec = utc_time[5];
      t.tm_isdst = 0;

      unix_second = static_cast<double>(mktime(&t));
    } else {
      uint32_t utc_time_big = *(uint32_t *)(&utc_time[0] + 2);
      unix_second = ((utc_time_big >> 24) & 0xff) |
                    ((utc_time_big >> 8) & 0xff00) |
                    ((utc_time_big << 8) & 0xff0000) | ((utc_time_big << 24));
    }
    fault_message_info.timestamp = time_stamp;
    fault_message_info.total_time =
        unix_second + (static_cast<double>(time_stamp)) / 1000000.0;
    fault_message_info.operate_state = ParserOperateState();
    fault_message_info.fault_state = ParserFaultState();
    fault_message_info.faultcode_type = ParserFaultCodeType();
    fault_message_info.rolling_counter = rolling_counter;
    fault_message_info.total_faultcode_num = total_fault_code_num;
    fault_message_info.faultcode_id = fault_code_id;
    fault_message_info.faultcode = fault_code;
    fault_message_info.dtc_num = (fault_code & 0x0000ffc0) >> 6;
    fault_message_info.dtc_state = ParserDTCState();
    fault_message_info.tdm_data_indicate = ParserTDMDataIndicate();
    fault_message_info.temperature = ParserTemperature();
    ParserLensDirtyState(fault_message_info.lens_dirty_state);
    fault_message_info.software_id = *((uint16_t *)(&software_version[0]));
    fault_message_info.software_version =
        *((uint16_t *)(&software_version[2]));
    fault_message_info.hardware_version =
        *((uint16_t *)(&software_version[4]));
    fault_message_info.bt_version = *((uint16_t *)(&software_version[6]));
    fault_message_info.heating_state = ParserHeatingState();
    fault_message_info.high_temperture_shutdown_state =
        ParserHighTempertureShutdownState();
    memcpy(fault_message_info.reversed, reversed, sizeof(reversed));
    fault_message_info.crc = crc;
    memcpy(fault_message_info.cycber_security, cycber_security,
          sizeof(cycber_security));
  }
  double ParserTemperature() {
    double temp =
        ((double)(*((uint16_t *)(&time_division_multiplexing[1])))) * 0.1f;
    return temp;
}
};
#pragma pack()
#endif