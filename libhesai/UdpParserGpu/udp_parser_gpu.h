#ifndef UDP_PARSER_GPU_H_
#define UDP_PARSER_GPU_H_
#include <stdint.h>
#include <iostream>
#include "lidar_types.h"
#include "general_parser_gpu.h"

namespace hesai
{
namespace lidar
{
// class UdpParserGpu
// the UdpParserGpu class is an interface layer.it instantiates a specific udp parser class,
// which is determined by lidar type.
// UdpParserGpu mainly computes xyzi of points with gpu
// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class UdpParserGpu {
 public:
  UdpParserGpu() {
    m_generalParserGpu = nullptr;
  };
  ~UdpParserGpu(){
    if (m_generalParserGpu != nullptr) {
      delete m_generalParserGpu;
      m_generalParserGpu = nullptr;
    }
  };
  int LoadCorrectionFile(std::string correction_path);
  int LoadCorrectionString(char *correction_string);
  void LoadFiretimesFile(std::string firetimes_path);
  int LoadFiretimesString(char *firetimes_string);
  void CreatGeneralParser(uint8_t major, uint8_t minor);
  int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  void SetLidarType(std::string lidar_type);
  int SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw);
 private:
  GeneralParserGpu<T_Point> * m_generalParserGpu;

};
}
}
#include "udp_parser_gpu.cu"

#endif  // UDP_PARSER_GPU_H_

