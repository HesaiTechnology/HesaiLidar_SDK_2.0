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
