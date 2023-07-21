#ifndef UDP_PARSER_H_
#define UDP_PARSER_H_

#include <vector>
#include "general_parser.h"
#include "pcap_saver.h"
#include "lidar_types.h"
#include "udp_p40_parser.h"
#include "udp_p64_parser.h"
#include "udp1_4_parser.h"
#include "udp3_1_parser.h"
#include "udp3_2_parser.h"
#include "udp4_3_parser.h"
#include "udp6_1_parser.h"
#include "udp7_2_parser.h"
#include "udp_p40_parser.h"
#include "udp_p64_parser.h"
#define PKT_SIZE_40P (1262)
#define PKT_SIZE_AC (1256)
#define PKT_SIZE_64 (1194)
#define PKT_SIZE_20 (1270)
namespace hesai
{
namespace lidar
{
// class UdpParser
// the UdpParser class is an interface layer.it instantiates a specific udp parser class,
// which is determined by lidar type.
// UdpParser mainly parsers udp or pcap packets and computes xyzi of points 
// you can parser the upd or pcap packets using the DocodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template <typename T_Point>
class UdpParser {
 public:
  UdpParser(uint8_t major, uint8_t minor);
  UdpParser(std::string lidar_type);
  UdpParser(UdpPacket &packet);
  UdpParser();
  virtual ~UdpParser();
  void CreatGeneralParser(uint8_t major, uint8_t minor);
  void CreatGeneralParser(std::string lidar_type);
  void CreatGeneralParser(const UdpPacket& packet);
  GeneralParser<T_Point> *GetGeneralParser();
  void SetGeneralParser(GeneralParser<T_Point> *Parser);
  PcapSaver *GetPcapSaver();

  // get lidar correction file from local file,and pass to udp parser
  void LoadCorrectionFile(std::string correction_path);  //从本地文件获取
  int LoadCorrectionString(char *correction_string);  //从角度文件char*数据获取

  // get lidar firetime correction file from local file,and pass to udp parser
  void LoadFiretimesFile(std::string firetimes_path);
  void EnableUpdateMonitorInfo();
  void DisableUpdateMonitorInfo();
  uint16_t *GetMonitorInfo1();
  uint16_t *GetMonitorInfo2();
  uint16_t *GetMonitorInfo3();

  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // udp_packet is the origin udp packet, output is the decoded packet
  int DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket); 

  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);  

  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame    
  int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet);

  int GetGeneralParser(GeneralParser<T_Point> **parser);
  int SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw);
  GeneralParser<T_Point>* GetParser() {return parser_;}
  std::string GetLidarType() {return lidar_type_decoded_;}
  void SetPcapPlay(bool pcap_time_synchronization, int source_type);
  void SetFrameAzimuth(float frame_start_azimuth);
 private:
  GeneralParser<T_Point> *parser_;
  PcapSaver *pcap_saver_;
  std::string lidar_type_decoded_;
  bool fisrt_packet_;
  uint64_t last_host_timestamp_;
  uint64_t last_sensor_timestamp_;
  uint8_t packet_count_;
  bool pcap_time_synchronization_;
  int source_type_;
};
}  // namespace lidar
}  // namespace hesai

#include "udp_parser.cc"

#endif  // UDP_PARSER_H_
