#include "udp2_4_parser.h"
#include "udp_protocol_v2_4.h"
#include "udp_protocol_header.h"
#include "udp_parser.h"

using namespace hesai::lidar;

template<typename T_Point>
Udp2_4Parser<T_Point>::Udp2_4Parser() {
  this->get_correction_file_ = false;
  for (int i = 0; i < CIRCLE; ++i) {
    this->sin_all_angle_[i] = std::sin(i * 2 * M_PI / CIRCLE);
    this->cos_all_angle_[i] = std::cos(i * 2 * M_PI / CIRCLE);
  }
}

template<typename T_Point>
Udp2_4Parser<T_Point>::~Udp2_4Parser() { printf("release Udp2_4Parser\n"); }

// The main purpose of this function is to open, read, and parse a lidar calibration file, and print appropriate messages based on the parsing results.
// This function can read both .bin and .csv files.
template<typename T_Point>
void Udp2_4Parser<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
  int type = 0;
  int length = lidar_correction_file.length();
  if (length >= 4) {
        std::string extension = lidar_correction_file.substr(length - 4);
        if (extension == ".bin") {
            type = 1; //  .bin
            printf(".bin\n");
        } else if (extension == ".csv") {
            type = 2; //  .csv
            printf(".csv\n");
        } else {
            type = 0; //  wrong
            return;
        }   
  }
  if (type == 1) {
    // print information
    int ret = 0;
    printf("load correction file from local correction.bin now!\n");
    std::ifstream fin(lidar_correction_file);
    if (fin.is_open()) {
      printf("Open correction file success!\n");
      int length = 0;
      std::string str_lidar_calibration;
      fin.seekg(0, std::ios::end);
      length = fin.tellg();
      // return the begin of file
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      // file --> buffer
      fin.read(buffer, length);
      fin.close();
      ret = LoadCorrectionString(buffer);
      if (ret != 0) {
        printf("Parse local Correction file Error!\n");
      } else {
        printf("Parse local Correction file Success!!!\n");
        this->get_correction_file_ = true;
      }
    } else { // open failed
      printf("Open correction file failed\n");
      return;
    }
  } 

  if (type == 2) {
    // print information
    int ret = 0;
    printf("load correction file from local correction.csv now!\n");
    ret = LoadCorrectionString_csv(lidar_correction_file);
    if (1 == ret) {
      printf("Parse local Correction file Success!\n");
      this->get_correction_file_ = true;
    } else {
      printf("Parse local Correction file faild!\n");
    }
  }
}

// csv ----> correction
template<typename T_Point>
int  Udp2_4Parser<T_Point>::LoadCorrectionString_csv(std::string lidar_correction_file)
{
    std::ifstream file(lidar_correction_file);
    if (!file.is_open()) {
        printf("open the .csv faild\n");
        return 0;
    }
    std::vector<float> column2;
    std::vector<float> column3;
    std::string line;
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<std::string> row;
        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }
        if (row.size() >= 3) {
            column2.push_back(std::stof(row[1]));
            column3.push_back(std::stof(row[2]));
        }
    }
    file.close();
    m_ET_corrections.delimiter[0] = 0xee;
    m_ET_corrections.delimiter[1] = 0xff;
    m_ET_corrections.major_version = 0x03;
    m_ET_corrections.min_version = 0x01;
    m_ET_corrections.channel_number = 0x40;
    m_ET_corrections.angle_division = 0x01;
  
    for (size_t i = 0; i < column3.size(); i++) {
      m_ET_corrections.elevations[i] = column2[i];
      m_ET_corrections.azimuths[i] = column3[i];
    }
    return 1;
}

// buffer(.bin) ---> correction
template<typename T_Point>
int Udp2_4Parser<T_Point>::LoadCorrectionString(char *data) {
  try {
    char *p = data;
    struct ETCorrections_v4_Header ETheader = *((struct ETCorrections_v4_Header* )p);
    if (0xee == ETheader.delimiter[0] && 0xff == ETheader.delimiter[1]) {
      switch (ETheader.min_version) {
        case 1: {
          memcpy((void *)&m_ET_corrections, p, sizeof(struct ETCorrections_v4_Header));
          p += sizeof(ETCorrections_v4_Header);
          auto channel_num = m_ET_corrections.channel_number;
          uint16_t division = m_ET_corrections.angle_division;
          memcpy((void *)&m_ET_corrections.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ET_corrections.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(uint32_t) * channel_num;
          m_ET_corrections.elevations[0] = ((float)(m_ET_corrections.apha)) / division;
          m_ET_corrections.elevations[1] = ((float)(m_ET_corrections.beta)) / division;
          m_ET_corrections.elevations[2] = ((float)(m_ET_corrections.gamma)) / division;
          printf("apha:%f, beta:%f, gamma:%f\n", m_ET_corrections.elevations[0], m_ET_corrections.elevations[1], m_ET_corrections.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            m_ET_corrections.azimuths[i + 3] = ((float)(m_ET_corrections.raw_azimuths[i])) / division;
            m_ET_corrections.elevations[i + 3] = ((float)(m_ET_corrections.raw_elevations[i])) / division;
            printf("%d %f %f \n",i, m_ET_corrections.azimuths[i + 3], m_ET_corrections.elevations[i + 3]);
          }
          memcpy((void*)&m_ET_corrections.SHA_value, p, 32);
          // successed
          this->get_correction_file_ = true;
          return 0;
        } break;
        default:
          printf("min_version is wrong!\n");
          break;
      }
    } else {
        return -1;
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return -1;
  }
  return -1;
}

// decode, udpPacket---->DecodePacket
template<typename T_Point>
int Udp2_4Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  // verify  buffer[0] and buffer[1]
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }

  // point to header skip pre_header
  const HS_LIDAR_HEADER_ET_V4* pHeader =
    reinterpret_cast<const HS_LIDAR_HEADER_ET_V4 *>(
      &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  // point to tails
  const HS_LIDAR_TAIL_ET_V4 *pTail =
    reinterpret_cast<const HS_LIDAR_TAIL_ET_V4 *>(
      &(udpPacket.buffer[0]) +  pHeader->GetPacketSize() - 
      sizeof(HS_LIDAR_TAIL_CRC_ET_V4) - 
      sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V4) - 
      sizeof(HS_LIDAR_TAIL_ET_V4)
      );
  // if (pHeader->HasSeqNum()){
  //   const HS_LIDAR_TAIL_SEQ_NUM_ET_V4 *pTailSeqNum =
  //     reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ET_V4 *>(
  //     &(udpPacket.buffer[0]) + pHeader->GetPacketSize() - 
  //     sizeof(HS_LIDAR_TAIL_CRC_ET_V4) - 
  //     sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V4)
  //     );
  // }
  const HS_LIDAR_BODY_SEQ2_ET_V4* pSeq2 = 
    reinterpret_cast<const HS_LIDAR_BODY_SEQ2_ET_V4 *>(
      (const unsigned char *)pHeader + 
      sizeof(HS_LIDAR_HEADER_ET_V4) + 2
    );
  const HS_LIDAR_BODY_LASTER_UNIT_ET_V4* pUnit = 
    reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V4 *>(
      (const unsigned char *)pSeq2 + 
      sizeof(HS_LIDAR_BODY_SEQ2_ET_V4)
    );
  // write the value to output
  output.host_timestamp = GetMicroTickCountU64();
  output.distance_unit = pHeader->GetDistUnit();
  if (output.use_timestamp_type == 0) {
    output.sensor_timestamp = pTail->GetMicroLidarTimeU64();
  } else {
    output.sensor_timestamp = udpPacket.recv_timestamp;
  }
  output.host_timestamp = GetMicroTickCountU64();
  output.points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  output.scan_complete = false;
  output.block_num = pHeader->GetBlockNum();
  output.laser_num = pHeader->GetLaserNum();
  
  int index_unit = 0;
  // int index_seq = 0;
  for (int blockId = 0; blockId < pHeader->GetBlockNum(); blockId++) {
    for (int seqId = 0; seqId < pHeader->GetSeqNum(); seqId++) {
      int16_t horizontalAngle = pSeq2->GetHorizontalAngle();
      int16_t verticalAngle = pSeq2->GetVerticalAngle();
      for (int unitId = 0; unitId < (pHeader->GetLaserNum()/pHeader->GetSeqNum()); unitId++){
        int16_t distance = pUnit->GetDistance();
        int8_t reflectivity = pUnit->GetReflectivity();
        // int8_t confidence = pUnit->GetConfidence();
        output.reflectivities[index_unit] = reflectivity;
        output.distances[index_unit] = distance;
        // get the degrees
        output.azimuth[index_unit] = horizontalAngle/512.0f;
        output.elevation[index_unit] = verticalAngle/512.0f;
        index_unit++;
        pUnit += 1;
      }
      // pSeq2 next
      pSeq2 = reinterpret_cast<const HS_LIDAR_BODY_SEQ2_ET_V4 *>(
        (const unsigned char *)pSeq2 + 
        sizeof(HS_LIDAR_BODY_SEQ2_ET_V4) + 
        sizeof(HS_LIDAR_BODY_LASTER_UNIT_ET_V4)*(pHeader->GetLaserNum()/pHeader->GetSeqNum())  
       );
       
      // pUnit next
      pUnit = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V4 *>(
        (const unsigned char *)pSeq2 + 
        sizeof(HS_LIDAR_BODY_SEQ2_ET_V4)
      ); 
    } 
    if (blockId < (pHeader->GetBlockNum())) {
      // skip point id
      pSeq2 = reinterpret_cast<const HS_LIDAR_BODY_SEQ2_ET_V4 *>(
        (const unsigned char *)pSeq2 + 2  
        );
      pUnit = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V4 *>(
        (const unsigned char *)pUnit + 2
        );
    } // end if (blockId < (pHeader->GetBlockNum()))
  }
  //printf("%d\n",index_unit);
  uint16_t nowid = pTail->GetFrameID();
  //printf("%d\n", nowid);
  if (this->use_angle_ && IsNeedFrameSplit(nowid)) {
    output.scan_complete = true;
  }
  this->last_frameid_  = pTail->GetFrameID();
  return 0;
}
  
//  Framing
template<typename T_Point>
bool Udp2_4Parser<T_Point>::IsNeedFrameSplit(uint16_t nowid) {
  if ( nowid != this->last_frameid_ ) {
      return true;
  }
  return false;
}

// get elevations[i]
template<typename T_Point>
int16_t Udp2_4Parser<T_Point>::GetVecticalAngle(int channel) {
  if (this->get_correction_file_ == false) {
    printf ("GetVecticalAngle: no correction file get, Error\n");
    return -1;
  }
  return m_ET_corrections.elevations[channel];
}


template<typename T_Point>
int Udp2_4Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  // get configer information from m_ET_corrections, 1 block 1 m_ET_corrections 
  // float division = (float)m_ET_corrections.angle_division;
  float apha =  m_ET_corrections.elevations[0];
  float beta =  m_ET_corrections.elevations[1];
  float gamma =  m_ET_corrections.elevations[2];
  // get the laser_num
  uint16_t lasernum = packet.laser_num;
  for (int blockId = 0; blockId < packet.block_num; blockId++) {
    // T_Point point;
    // float delte_apha = 0;
    // float delte_theta = 0;
    for (int i = 0; i < lasernum; i++) {
      int point_index = packet.packet_index * packet.points_num + blockId * packet.laser_num + i; 
      // get phi and psi and distance
      float raw_azimuth = packet.azimuth[i + blockId*lasernum];
      float raw_elevation = packet.elevation[i + blockId*lasernum];
      float distance = packet.distances[i + blockId*lasernum] * packet.distance_unit;
      float phi = m_ET_corrections.azimuths[i + 3];
      float theta = m_ET_corrections.elevations[i + 3];
      float an = apha + phi;
      float theta_n = (raw_elevation + theta / std::cos(an * M_PI / 180));
      float elv_v = raw_elevation * M_PI / 180 + theta * M_PI / 180 - std::tan(raw_elevation * M_PI / 180) * (1 - std::cos(an * M_PI / 180)) ;
      float delt_azi_v = std::sin(an * M_PI / 180) * std::cos(an * M_PI / 180) * theta_n * theta_n / 2  * 1.016 * M_PI / 180 * M_PI / 180;
      float eta = phi + delt_azi_v * 180 / M_PI + beta + raw_azimuth / 2;
      float delt_azi_h = std::sin(eta * M_PI / 180) * std::tan(2 * gamma * M_PI / 180) * std::tan(elv_v ) + std::sin(2 * eta * M_PI / 180) * gamma * gamma * M_PI / 180 * M_PI / 180;
      float elv_h = elv_v * 180 / M_PI + std::cos(eta * M_PI / 180) * 2 * gamma ;
      float azi_h = 90 +  raw_azimuth + delt_azi_h * 180 / M_PI + delt_azi_v * 180 / M_PI + phi;


      int azimuth = (int)(azi_h * 100 + CIRCLE) % CIRCLE;
      if (packet.config.fov_start != -1 && packet.config.fov_end != -1)
      {
        int fov_transfer = azimuth / 256 / 100;
        if (fov_transfer < packet.config.fov_start || fov_transfer > packet.config.fov_end){//不在fov范围continue
          continue;
        }
      } 
      int elevation = (int)(elv_h * 100 + CIRCLE) % CIRCLE;
      float xyDistance = distance * this->cos_all_angle_[elevation];
      float x = xyDistance * this->sin_all_angle_[azimuth];
      float y = xyDistance * this->cos_all_angle_[azimuth];
      float z = distance * this->sin_all_angle_[elevation];
      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], packet.reflectivities[blockId * packet.laser_num + i]);
      setTimestamp(frame.points[point_index], double(packet.sensor_timestamp) / kMicrosecondToSecond);
      setRing(frame.points[point_index], i);
    }
  }
  frame.points_num += packet.points_num;
  frame.packet_num = packet.packet_index;
  return 0;
}

template<typename T_Point>
int Udp2_4Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
{
  // TO DO
  return 0;
}