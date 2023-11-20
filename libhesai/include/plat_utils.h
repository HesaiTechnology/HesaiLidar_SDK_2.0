#ifndef _PLAT_UTILS_H_
#define _PLAT_UTILS_H_

#ifdef _MSC_VER
#else
#include <pthread.h>
#include <sched.h>
#endif
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <map>
#include <string>
#include <utility>
#include <thread>
#include <sstream>
#define SHED_FIFO_PRIORITY_HIGH 99
#define SHED_FIFO_PRIORITY_MEDIUM 70
#define SHED_FIFO_PRIORITY_LOW 1
#define ISO_8601_FORMAT 1

#ifdef _MSC_VER
extern void SetThreadPriorityWin(int priority);
#else
extern void SetThreadPriority(int policy, int priority);
#endif
#ifndef _MSC_VER
extern unsigned int GetTickCount();
#endif

extern unsigned int GetMicroTickCount();

extern uint64_t GetMicroTickCountU64();

extern int GetAvailableCPUNum();

template <typename T_Point>
void split_string(T_Point&, const std::string&, char);

// extern int GetAnglesFromFile(
//     const std::string& sFile,
//     std::map<int, std::pair<float, float>>& mapAngleMetaData);

extern int GetCurrentTimeStamp(std::string &sTime, int nFormat = ISO_8601_FORMAT);
#endif  //_PLAT_UTILS_H_
