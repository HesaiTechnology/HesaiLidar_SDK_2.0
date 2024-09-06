/*
  This file encapsulates the data types for ET radar correction files and defines a class for an ET parser.
*/

#ifndef UDP2_6_PARSER_H
#define UDP2_6_PARSER_H

#include "general_parser.h"
#include "lidar_types.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
namespace hesai
{
namespace lidar
{
  // class Udp2_6Parser
  // parsers packets and computes points for ET25
  // you can parser the upd or pcap packets using the DocodePacket fuction
  // you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
  template<typename T_Point>
  class Udp2_6Parser : public GeneralParser<T_Point> {
    public:
      Udp2_6Parser();
      virtual ~Udp2_6Parser();
      
      // 
      int16_t GetVecticalAngle(int channel); 

      // determine whether frame splitting is needed
      bool IsNeedFrameSplit(uint16_t frame_id);


      // get lidar correction file from local file,and pass to udp parser 
      virtual void LoadCorrectionFile(std::string correction_path);
      virtual int LoadCorrectionString(char *correction_string);
      int LoadCorrectionDatData(char *correction_string);
      int LoadCorrectionCsvData(char *correction_string);
      
      // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
      virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);

      // compute xyzi of points from decoded packet
      // param packet is the decoded packet; xyzi of points after computed is puted in frame    
      virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index);
      ETCorrections corrections_;
      

    // 类维护的成员变量 
    protected:
      // int view_mode_;           
      // std::string pcap_file_;
      // std::string lidar_correction_file_;
      bool get_correction_file_;
      int last_frameid_ = -1;
  };



}  // namespace lidar
}  // namespace hesai

#include "udp2_6_parser.cc"


#endif // UDP2_6_PARSER_H
