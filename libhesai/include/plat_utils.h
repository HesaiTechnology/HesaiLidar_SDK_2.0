#ifndef _PLAT_UTILS_H_
#define _PLAT_UTILS_H_

#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <map>
#include <string>
#include <utility>

#define SHED_FIFO_PRIORITY_HIGH 99
#define SHED_FIFO_PRIORITY_MEDIUM 70
#define SHED_FIFO_PRIORITY_LOW 1
#define ISO_8601_FORMAT 1

extern void ShowThreadPriorityMaxMin(int policy);
extern void SetThreadPriority(int policy, int priority);

extern unsigned int GetTickCount();

extern unsigned int GetMicroTickCount();

extern uint64_t GetMicroTickCountU64();

extern int GetAvailableCPUNum();

// extern int GetAnglesFromFile(
//     const std::string& sFile,
//     std::map<int, std::pair<float, float>>& mapAngleMetaData);

extern int GetCurrentTime(std::string &sTime, int nFormat = ISO_8601_FORMAT);
#endif  //_PLAT_UTILS_H_
