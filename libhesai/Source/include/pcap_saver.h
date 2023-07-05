#ifndef PCPASAVER_H_
#define PCPASAVER_H_

#include <functional>
#include <array>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <string>
#include <string_view>
#include <fstream>
#include <memory>
#include <queue>
#include <cstring>
#include "pcap_source.h"
#include "blocking_ring.h"
#include <chrono>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include "pcap_source.h"
#include "lidar_types.h"
namespace hesai
{
namespace lidar
{
using Clock = std::chrono::system_clock;
using Time = Clock::time_point;
using Period = Clock::period;
using Duration = Clock::duration;

struct PandarPacket {
  uint32_t size;
  uint16_t port;
  uint8_t buffer[1500];
  PandarPacket(const uint8_t *data = nullptr, uint32_t sz = 0, uint16_t prt = 0)
      : size(sz), port(prt) {
    memcpy(buffer, data, size);
  }
};

class PcapSaver {
public:
    bool tcp_dumped_;
private:
    std::ofstream ofs_;
    std::string pcap_path_;

    using Container = BlockingRing<PandarPacket, 32*1024>;
    std::shared_ptr<Container> packets_cache_;
    bool dumping_;
    bool dumping_blocked_;
    // std::mutex _mutex;
    std::thread dumping_thread_;
public:
    PcapSaver();
    PcapSaver(const PcapSaver&) = delete;
    PcapSaver& operator=(const PcapSaver&) = delete;
    ~PcapSaver();

    std::string pcap_path() const;
    void SetPcapPath(std::string pcap_path);
    int Save();
    int Save(const std::string &recordPath, const UdpFrame_t &packets,
            int port = 2368);
    int Save(const std::string &recordPath, const UdpFrameArray_t &packets,
            int port = 2368);
    void Dump(const uint8_t*, uint32_t, uint16_t port = 2368);
    void TcpDump(const uint8_t*, uint32_t, uint32_t max_pkt_len = 1500 - sizeof(PcapTCPHeader), uint16_t port = 2368);
    void close();
};
}  // namespace lidar
}  // namespace hesai


#endif // PCPASTREAMER_H_
