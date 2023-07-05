/******************************************************************************
 * Copyright 2021 The Hesai Technology. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef AUTOTIMER_H_
#define AUTOTIMER_H_

#include <map>
#include <numeric>
#include <string>
#include <vector>
#include <iostream>
namespace hesai
{
namespace lidar
{
class AutoTickCount;

class TickCount {
 public:
  TickCount();
  ~TickCount();

  int Start();
  int Pause();
  std::string GetTimeCost(uint64_t &u64TimeCost);
  std::string GetTimeCost();
  int ShowTimeSlice(std::string sLogFile = "", bool bSaveSysTime = false);
  std::map<std::string, std::vector<uint64_t>> GetTimeSlice() {
    return m_timeSlice;
  }
  uint64_t GetTimeSlice(std::string sKey, int nTime = -1);
  // 多次使用相同关键字调用，只有第一次的生效
  int Begin(std::string sKey);
  int End(std::string sKey, bool bShow = true);
  int SetName(std::string sName) {
    m_sName = sName;

    return 0;
  }

 private:
  static const uint32_t kMicroToSec = 1000000;
  static const uint8_t kClockUnit = 60;
  uint64_t m_u64StartTime;
  uint64_t m_u64EndTime;
  std::map<std::string, std::vector<uint64_t>> m_timeSlice;
  std::map<std::string, uint64_t> m_startTime;
  std::string m_sName;

  friend class AutoTickCount;

  int AppentTimeSlice(std::string sKey, uint64_t u64Time);
};

class AutoTickCount {
 public:
  AutoTickCount(TickCount &t, std::string sKey = "", bool bShow = true);
  ~AutoTickCount();

 private:
  TickCount *m_pT;
  uint64_t m_u64StartTime;
  uint64_t m_u64EndTime;
  bool m_bShow;
  std::string m_sKey;
};
}  // namespace lidar
}  // namespace hesai
#endif  // AUTOTIMER_H_
