
#include "udp2_5_parser.h"
#include "udp_protocol_v2_5.h"
#include "udp_protocol_header.h"
#include "udp_parser.h"

using namespace hesai::lidar;

template<typename T_Point>
Udp2_5Parser<T_Point>::Udp2_5Parser() {
  this->get_correction_file_ = false;
  for (int i = 0; i < CIRCLE; ++i) {
    this->sin_all_angle_[i] = std::sin(i * 2 * M_PI / CIRCLE);
    this->cos_all_angle_[i] = std::cos(i * 2 * M_PI / CIRCLE);
  }
}

template<typename T_Point>
Udp2_5Parser<T_Point>::~Udp2_5Parser() { printf("release Udp2_5Parser\n"); }


// The main purpose of this function is to open, read, and parse a lidar calibration file, and print appropriate messages based on the parsing results.
// This function can read both .bin and .csv files.
template<typename T_Point>
void Udp2_5Parser<T_Point>::LoadCorrectionFile(std::string correction_path) {
  int ret = 0;
  printf("load correction file from local correction file now!\n");
  std::ifstream fin(correction_path);
  if (fin.is_open()) {
    printf("Open correction file success\n");
    int length = 0;
    std::string str_lidar_calibration;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    str_lidar_calibration = buffer;
    ret = LoadCorrectionString(buffer);
    if (ret != 0) {
      printf("Parse local Correction file Error\n");
    } else {
      
      printf("Parse local Correction file Success!!!\n");
    }
  } else {
    printf("Open correction file failed\n");
    return;
  }
}

template<typename T_Point>
int Udp2_5Parser<T_Point>::LoadCorrectionString(char *data) {
  if (LoadCorrectionDatData(data) == 0) {
    return 0;
  }
  return LoadCorrectionCsvData(data);
}

// csv ----> correction
template<typename T_Point>
int  Udp2_5Parser<T_Point>::LoadCorrectionCsvData(char *correction_string)
{
  std::string correction_content_str = correction_string;
  std::istringstream ifs(correction_content_str);
  std::string line;

  // skip first line "Laser id,Elevation,Azimuth" or "eeff"
  std::getline(ifs, line);  
  float elevation_list[MAX_LASER_NUM], azimuth_list[MAX_LASER_NUM];
  std::vector<std::string> vfirstLine;
  split_string(vfirstLine, line, ',');
  if (vfirstLine[0] == "EEFF" || vfirstLine[0] == "eeff") {
    // skip second line
    std::getline(ifs, line);  
  }

  int lineCount = 0;
  while (std::getline(ifs, line)) {
    std::vector<std::string> vLineSplit;
    split_string(vLineSplit, line, ',');
    // skip error line or hash value line
    if (vLineSplit.size() < 3) {  
      continue;
    } else {
      lineCount++;
    }
    float elevation, azimuth;
    int laserId = 0;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> laserId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elevation;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (laserId != lineCount || laserId >= MAX_LASER_NUM) {
      std::cout << "laser id is wrong in correction file. laser Id:"
                  << laserId << ", line" << lineCount << std::endl;
      // return -1;
    }
    elevation_list[laserId - 1] = elevation;
    azimuth_list[laserId - 1] = azimuth;
  }

  for (int i = 0; i < lineCount; ++i) {
    corrections_.azimuths[i] = azimuth_list[i];
    corrections_.elevations[i] = elevation_list[i];
    printf("%d %f %f \n",i, corrections_.azimuths[i], corrections_.elevations[i]);
  }
  this->get_correction_file_ = true;
  return 0;
}


// buffer(.bin) ---> correction
template<typename T_Point>
int Udp2_5Parser<T_Point>::LoadCorrectionDatData(char *data) {
  try {
    char *p = data;
    struct ETCorrectionsHeader ETheader = *((struct ETCorrectionsHeader* )p);
    if (0xee == ETheader.delimiter[0] && 0xff == ETheader.delimiter[1]) {
      switch (ETheader.min_version) {
        case 1: {
          memcpy((void *)&corrections_.header, p, sizeof(struct ETCorrectionsHeader));
          p += sizeof(ETCorrectionsHeader);
          auto channel_num = corrections_.header.channel_number;
          uint16_t division = corrections_.header.angle_division;
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(uint32_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.header.gamma)) / division;
          printf("apha:%f, beta:%f, gamma:%f\n", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            printf("%d %f %f \n",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          
          memcpy((void*)&corrections_.SHA_value, p, 32);
          // successed
          this->get_correction_file_ = true;
          return 0;
        } break;
        case 2: {
          memcpy((void *)&corrections_.header, p, sizeof(struct ETCorrectionsHeader));
          p += sizeof(ETCorrectionsHeader);
          auto channel_num = corrections_.header.channel_number;
          uint16_t division = corrections_.header.angle_division;
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(uint32_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.header.gamma)) / division;
          printf("apha:%f, beta:%f, gamma:%f\n", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            printf("%d %f %f \n",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          corrections_.azimuth_adjust_interval = *((char*)p);
          p = p + 1;
          corrections_.elevation_adjust_interval = *((char*)p);
          p = p + 1;
          int angle_offset_len = (120 / (corrections_.azimuth_adjust_interval * 0.5) + 1) * (25 / (corrections_.elevation_adjust_interval * 0.5) + 1);
          memcpy((void*)corrections_.azimuth_adjust, p, sizeof(int16_t) * angle_offset_len);
          p = p + sizeof(int16_t) * angle_offset_len;
          memcpy((void*)corrections_.elevation_adjust, p, sizeof(int16_t) * angle_offset_len); 
          p = p + sizeof(int16_t) * angle_offset_len;
          // int adjustNum = channel_num;
          memcpy((void*)&corrections_.SHA_value, p, 32);
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
int Udp2_5Parser<T_Point>::DecodePacket(LidarDecodedPacket<T_Point> &output, const UdpPacket& udpPacket) {
  // verify  buffer[0] and buffer[1]
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }

  // point to header skip pre_header
  const HS_LIDAR_HEADER_ET_V5* pHeader =
    reinterpret_cast<const HS_LIDAR_HEADER_ET_V5 *>(
      &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));


  // point to tails
  const HS_LIDAR_TAIL_ET_V5 *pTail =
    reinterpret_cast<const HS_LIDAR_TAIL_ET_V5 *>(
      &(udpPacket.buffer[0]) +  pHeader->GetPacketSize() - 
      sizeof(HS_LIDAR_CYBER_SECURITY_ET_V5) - 
      sizeof(HS_LIDAR_TAIL_CRC_ET_V5) - 
      sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V5) - 
      sizeof(HS_LIDAR_TAIL_ET_V5)
      );
  // if (pHeader->HasSeqNum()){
  //   const HS_LIDAR_TAIL_SEQ_NUM_ET_V5 *pTailSeqNum =
  //     reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ET_V5 *>(
  //     &(udpPacket.buffer[0]) + pHeader->GetPacketSize() - 
  //     sizeof(HS_LIDAR_CYBER_SECURITY_ET_V5) - 
  //     sizeof(HS_LIDAR_TAIL_CRC_ET_V5) - 
  //     sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V5)
  //     );
  // }
  const HS_LIDAR_BODY_SEQ3_ET_V5* pSeq3 = 
    reinterpret_cast<const HS_LIDAR_BODY_SEQ3_ET_V5 *>(
      (const unsigned char *)pHeader + 
      sizeof(HS_LIDAR_HEADER_ET_V5) + 2
    );
  const HS_LIDAR_BODY_LASTER_UNIT_ET_V5* pUnit = 
    reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V5 *>(
      (const unsigned char *)pSeq3 + 
      sizeof(HS_LIDAR_BODY_SEQ3_ET_V5)
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
      int16_t horizontalAngle = pSeq3->GetHorizontalAngle();
      int16_t verticalAngle = pSeq3->GetVerticalAngle();
      // uint8_t Confidence = pSeq3->GetConfidence();
      for (int unitId = 0; unitId < (pHeader->GetLaserNum()/pHeader->GetSeqNum()); unitId++){
        int16_t distance = pUnit->GetDistance();
        int8_t reflectivity = pUnit->GetReflectivity();
        output.reflectivities[index_unit] = reflectivity;
        output.distances[index_unit] = distance;
        output.azimuth[index_unit] = horizontalAngle/512.0f;
        output.elevation[index_unit] = verticalAngle/512.0f;
        index_unit++;
        pUnit += 1;
      }
      // pSeq3 next
      pSeq3 = reinterpret_cast<const HS_LIDAR_BODY_SEQ3_ET_V5 *>(
        (const unsigned char *)pSeq3 + 
        sizeof(HS_LIDAR_BODY_SEQ3_ET_V5) + 
        sizeof(HS_LIDAR_BODY_LASTER_UNIT_ET_V5)*(pHeader->GetLaserNum()/pHeader->GetSeqNum())  
       );
       
      // pUnit next
      pUnit = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V5 *>(
        (const unsigned char *)pSeq3 + 
        sizeof(HS_LIDAR_BODY_SEQ3_ET_V5)
      ); 
    } 
    if (blockId < (pHeader->GetBlockNum())) {
      // skip point id
      pSeq3 = reinterpret_cast<const HS_LIDAR_BODY_SEQ3_ET_V5 *>(
        (const unsigned char *)pSeq3 + 2  
        );
      pUnit = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V5 *>(
        (const unsigned char *)pUnit + 2
        );
    } // end if (blockId < (pHeader->GetBlockNum()))
  }
  uint16_t frame_id = pTail->GetFrameID();
  if (this->use_angle_ && IsNeedFrameSplit(frame_id)) {
    output.scan_complete = true;
  }
  this->last_frameid_  = pTail->GetFrameID();
  return 0;
}
  
//  Framing
template<typename T_Point>
bool Udp2_5Parser<T_Point>::IsNeedFrameSplit(uint16_t frame_id) {
  if ( frame_id != this->last_frameid_ ) {
      return true;
  }
  return false;
}

// get elevations[i]
template<typename T_Point>
int16_t Udp2_5Parser<T_Point>::GetVecticalAngle(int channel) {
  if (this->get_correction_file_ == false) {
    printf ("GetVecticalAngle: no correction file get, Error\n");
    return -1;
  }
  return corrections_.elevations[channel];
}


template<typename T_Point>
int Udp2_5Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, LidarDecodedPacket<T_Point> &packet) {
  // get configer information from corrections_, 1 block 1 corrections_ 
  // float division = (float)corrections_.header.angle_division;
  float apha =  corrections_.elevations[0];
  float beta =  corrections_.elevations[1];
  float gamma =  corrections_.elevations[2];
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
      float phi = corrections_.azimuths[i + 3];
      float theta = corrections_.elevations[i + 3];
      float an = apha + phi;
      float theta_n = (raw_elevation + theta / std::cos(an * M_PI / 180));
      float elv_v = raw_elevation * M_PI / 180 + theta * M_PI / 180 - std::tan(raw_elevation * M_PI / 180) * (1 - std::cos(an * M_PI / 180)) ;
      float delt_azi_v = std::sin(an * M_PI / 180) * std::cos(an * M_PI / 180) * theta_n * theta_n / 2  * 1.016 * M_PI / 180 * M_PI / 180;
      float eta = phi + delt_azi_v * 180 / M_PI + beta + raw_azimuth / 2;
      float delt_azi_h = std::sin(eta * M_PI / 180) * std::tan(2 * gamma * M_PI / 180) * std::tan(elv_v ) + std::sin(2 * eta * M_PI / 180) * gamma * gamma * M_PI / 180 * M_PI / 180;
      float elv_h = elv_v * 180 / M_PI + std::cos(eta * M_PI / 180) * 2 * gamma ;
      float azi_h = 90 +  raw_azimuth + delt_azi_h * 180 / M_PI + delt_azi_v * 180 / M_PI + phi;
      if (corrections_.header.min_version == 2) {
        azi_h = azi_h + corrections_.getAziAdjustV2(azi_h - 90, elv_h);
        elv_h = elv_h + corrections_.getEleAdjustV2(azi_h - 90, elv_h);
      }
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
int Udp2_5Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
{
  // TO DO
  return 0;
}