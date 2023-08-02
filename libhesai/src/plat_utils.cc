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

#include <plat_utils.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#define gettid() syscall(SYS_gettid)
static const int kTimeStrLen = 1000;

void ShowThreadPriorityMaxMin(int policy) {
  int priority = sched_get_priority_max(policy);
  printf("policy %d max_priority = %d\n", policy, priority);
  priority = sched_get_priority_min(policy);
  printf("policy %d, min_priority = %d\n", policy, priority);
}

void SetThreadPriority(int policy, int priority) {
  printf("set thread %lu, tid %ld, policy %d and priority %d\n", pthread_self(),
         gettid(), policy, priority);
  sched_param param;
  param.sched_priority = priority;
  pthread_setschedparam(pthread_self(), policy, &param);

  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  printf("get thead %lu, tid %ld, policy %d and priority %d\n", pthread_self(),
         gettid(), ret_policy, param.sched_priority);
}

unsigned int GetTickCount() {
  unsigned int ret = 0;
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000000 + time.tv_sec * 1000;
  }
  return ret;
}

unsigned int GetMicroTickCount() {
  unsigned int ret = 0;
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  }
  return ret;
}

uint64_t GetMicroTickCountU64() {
  uint64_t ret = 0;
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  }
  return ret;
}

int GetAvailableCPUNum() { return sysconf(_SC_NPROCESSORS_ONLN); }

int GetAnglesFromFile(
    const std::string& sFile,
    std::map<int, std::pair<float, float>>& mapAngleMetaData) {
  FILE* pFile = fopen(sFile.c_str(), "r");

  if (NULL == pFile) {
    printf("cannot open the angle file, please check: %s\n", sFile.c_str());
    return 1;
  }

  char sContent[255] = {0};
  fgets(sContent, 255, pFile);  // skip first line

  while (!feof(pFile)) {
    memset(sContent, 0, 255);
    fgets(sContent, 255, pFile);

    if (strlen(sContent) < strlen(" , , ")) {
      break;
    }

    int iChannelId;
    float fPitch;
    float fAzimuth;

    sscanf(sContent, "%d,%f,%f", &iChannelId, &fPitch, &fAzimuth);

    std::pair<float, float> pairAngleData = std::make_pair(fPitch, fAzimuth);
    mapAngleMetaData.insert(std::map<int, std::pair<float, float>>::value_type(
        iChannelId, pairAngleData));
  }

  fclose(pFile);
  pFile = NULL;
  return 0;
}

// 2004-05-03T17:30:08+08:00
int GetCurrentTime(std::string &sTime, int nFormat) {
  time_t currentTime = time(NULL);
  struct tm *pLocalTime = localtime(&currentTime);
  char sFormattedTime[kTimeStrLen];

  if (ISO_8601_FORMAT == nFormat) {
    strftime(sFormattedTime, kTimeStrLen, "%FT%T%z", pLocalTime);

    sTime = std::string(sFormattedTime);
    // either ISO 8601 or C language is stupid, so change 0800 to 08:00
    sTime = sTime.insert(sTime.length() - 2, ":");

    return 0;
  } else {
    return -1;
  }
}
