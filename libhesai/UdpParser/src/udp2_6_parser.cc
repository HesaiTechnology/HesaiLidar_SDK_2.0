#include "udp2_6_parser.h"
#include "udp_protocol_v2_6.h"
#include "udp_protocol_header.h"
#include "udp_parser.h"

using namespace hesai::lidar;

template<typename T_Point>
Udp2_6Parser<T_Point>::Udp2_6Parser() {
  this->get_correction_file_ = false;
}

template<typename T_Point>
Udp2_6Parser<T_Point>::~Udp2_6Parser() { LogInfo("release Udp2_6Parser"); }

// The main purpose of this function is to open, read, and parse a lidar calibration file, and print appropriate messages based on the parsing results.
// This function can read both .bin and .csv files.
template<typename T_Point>
void Udp2_6Parser<T_Point>::LoadCorrectionFile(std::string correction_path) {
  LogInfo("load correction file from local correction file now!");
  std::ifstream fin(correction_path);
  if (fin.is_open()) {
    LogDebug("Open correction file success");
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = static_cast<int>(fin.tellg());
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    int ret = LoadCorrectionString(buffer);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local Correction file Error");
    } else {  
      LogInfo("Parse local Correction file Success!!!");
    }
  } else {
    LogError("Open correction file failed");
    return;
  }
}

template<typename T_Point>
int Udp2_6Parser<T_Point>::LoadCorrectionString(char *data) {
  if (LoadCorrectionDatData(data) == 0) {
    return 0;
  }
  return LoadCorrectionCsvData(data);
}

// csv ----> correction
template<typename T_Point>
int  Udp2_6Parser<T_Point>::LoadCorrectionCsvData(char *correction_string)
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

    if (laserId > MAX_LASER_NUM || laserId <= 0) {
      LogFatal("laser id is wrong in correction file. laser Id: %d, line: %d", laserId, lineCount);
      continue;
    }
    if (laserId != lineCount) {
      LogWarning("laser id is wrong in correction file. laser Id: %d, line: %d.  continue", laserId, lineCount);
      continue;
    }
    elevation_list[laserId - 1] = elevation;
    azimuth_list[laserId - 1] = azimuth;
  }

  for (int i = 0; i < lineCount; ++i) {
    corrections_.azimuths[i] = azimuth_list[i];
    corrections_.elevations[i] = elevation_list[i];
    LogDebug("%d %f %f ", i, corrections_.azimuths[i], corrections_.elevations[i]);
  }
  this->get_correction_file_ = true;
  return 0;
}


// buffer(.bin) ---> correction
template<typename T_Point>
int Udp2_6Parser<T_Point>::LoadCorrectionDatData(char *data) {
  try {
    char *p = data;
    struct ETCorrectionsHeader ETheader = *((struct ETCorrectionsHeader* )p);
    if (0xee == ETheader.delimiter[0] && 0xff == ETheader.delimiter[1]) {
      switch (ETheader.min_version) {
        case 1: {
          ETCorrectionsHeader_V1V2 correction_v1;
          memcpy((void *)&correction_v1, p, sizeof(struct ETCorrectionsHeader_V1V2));
          corrections_.header.getDataFromV1V2(correction_v1);
          p += sizeof(ETCorrectionsHeader_V1V2);
          auto channel_num = corrections_.header.channel_number;
          uint16_t division = corrections_.header.angle_division;
          if ((channel_num > ET_MAX_CHANNEL_NUM - 3) || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.header.gamma)) / division;
          LogDebug("apha:%f, beta:%f, gamma:%f", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            LogDebug("%d %f %f",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          
          memcpy((void*)&corrections_.SHA_value, p, 32);
          // successed
          this->get_correction_file_ = true;
          return 0;
        } break;
        case 2: {
          ETCorrectionsHeader_V1V2 correction_v1;
          memcpy((void *)&correction_v1, p, sizeof(struct ETCorrectionsHeader_V1V2));
          corrections_.header.getDataFromV1V2(correction_v1);
          p += sizeof(ETCorrectionsHeader_V1V2);
          auto channel_num = corrections_.header.channel_number;
          uint16_t division = corrections_.header.angle_division;
          if ((channel_num > ET_MAX_CHANNEL_NUM - 3) || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&corrections_.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          corrections_.elevations[2] = ((float)(corrections_.header.gamma)) / division;
          LogDebug("apha:%f, beta:%f, gamma:%f", corrections_.elevations[0], corrections_.elevations[1], corrections_.elevations[2]);
          for (int i = 0; i < channel_num; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
            LogDebug("%d %f %f ",i, corrections_.azimuths[i + 3], corrections_.elevations[i + 3]);
          }
          corrections_.azimuth_adjust_interval = *((char*)p);
          p = p + 1;
          corrections_.elevation_adjust_interval = *((char*)p);
          p = p + 1;
          int angle_offset_len = int((120 / (corrections_.azimuth_adjust_interval * 0.5) + 1) * (25 / (corrections_.elevation_adjust_interval * 0.5) + 1));
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
          LogWarning("min_version is wrong!");
          break;
      }
    } else {
        return -1;
    }
  } catch (const std::exception &e) {
    LogFatal("load correction error: %s", e.what());
    return -1;
  }
  return -1;
}

//  Framing
template<typename T_Point>
bool Udp2_6Parser<T_Point>::IsNeedFrameSplit(uint16_t nowid) {
  if ( nowid != this->last_frameid_  && this->last_frameid_ >= 0) {
      return true;
  }
  return false;
}

// get elevations[i]
template<typename T_Point>
int16_t Udp2_6Parser<T_Point>::GetVecticalAngle(int channel) {
  if (this->get_correction_file_ == false) {
    LogError("GetVecticalAngle: no correction file get, Error");
    return -1;
  }
  return corrections_.elevations[channel];
}


template<typename T_Point>
int Udp2_6Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  // get configer information from corrections_, 1 block 1 corrections_ 
  // float division = (float)corrections_.header.angle_division;
  float apha =  corrections_.elevations[0];
  float beta =  corrections_.elevations[1];
  float gamma =  corrections_.elevations[2];

  for (size_t i = 0; i < frame.per_points_num; i++) {
    int point_index = packet_index * frame.per_points_num + i; 
    // get phi and psi and distance
    float raw_azimuth = frame.pointData[point_index].azimuth;
    float raw_elevation = frame.pointData[point_index].elevation;
    float distance = static_cast<float>(frame.pointData[point_index].distances * frame.distance_unit);
    float phi = corrections_.azimuths[frame.pointData[point_index].chn_index + 3];
    float theta = corrections_.elevations[frame.pointData[point_index].chn_index + 3];
    float an = apha + phi;
    float theta_n = (raw_elevation + theta / std::cos(an * M_PI / 180));
    float elv_v = raw_elevation * M_PI / 180 + theta * M_PI / 180 - std::tan(raw_elevation * M_PI / 180) * (1 - std::cos(an * M_PI / 180)) ;
    float delt_azi_v = std::sin(an * M_PI / 180) * std::cos(an * M_PI / 180) * theta_n * theta_n / 2  * 1.016 * M_PI / 180 * M_PI / 180;
    float eta = phi + delt_azi_v * 180 / M_PI + beta + raw_azimuth / 2;
    float delt_azi_h = std::sin(eta * M_PI / 180) * std::tan(2 * gamma * M_PI / 180) * std::tan(elv_v ) + std::sin(2 * eta * M_PI / 180) * gamma * gamma * M_PI / 180 * M_PI / 180;
    float elv_h = elv_v * 180 / M_PI + std::cos(eta * M_PI / 180) * 2 * gamma ;
    float azi_h = 90 +  raw_azimuth + delt_azi_h * 180 / M_PI + delt_azi_v * 180 / M_PI + phi;
    if (corrections_.header.min_version == 2 || corrections_.header.min_version == 3) {
      azi_h = azi_h + corrections_.getAziAdjustV2(azi_h - 90, elv_h);
      elv_h = elv_h + corrections_.getEleAdjustV2(azi_h - 90, elv_h);
    }
    int azimuth = (int)(azi_h * kAllFineResolutionFloat + CIRCLE) % CIRCLE;
    if (frame.config.fov_start != -1 && frame.config.fov_end != -1)
    {
      int fov_transfer = azimuth / 256 / 100;
      if (fov_transfer < frame.config.fov_start || fov_transfer > frame.config.fov_end){//不在fov范围continue
          memset(&frame.points[point_index], 0, sizeof(T_Point));
        continue;
      }
    }       
    int elevation = (int)(elv_h * kAllFineResolutionFloat + CIRCLE) % CIRCLE;
    float xyDistance = distance * this->cos_all_angle_[elevation];
    float x = xyDistance * this->sin_all_angle_[azimuth];
    float y = xyDistance * this->cos_all_angle_[azimuth];
    float z = distance * this->sin_all_angle_[elevation];
    setX(frame.points[point_index], x);
    setY(frame.points[point_index], y);
    setZ(frame.points[point_index], z);
    setIntensity(frame.points[point_index], frame.pointData[point_index].reflectivities);
    setTimestamp(frame.points[point_index], double(frame.sensor_timestamp[packet_index]) / kMicrosecondToSecond);
    setRing(frame.points[point_index], frame.pointData[point_index].chn_index);
  }

  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
int Udp2_6Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
{
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }

  const HS_LIDAR_HEADER_ET_V6* pHeader =
    reinterpret_cast<const HS_LIDAR_HEADER_ET_V6 *>(
      &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  const HS_LIDAR_TAIL_ET_V6 *pTail =
    reinterpret_cast<const HS_LIDAR_TAIL_ET_V6 *>(
      &(udpPacket.buffer[0]) +  pHeader->GetPacketSize() - 
      sizeof(HS_LIDAR_CYBER_SECURITY_ET_V6) - 
      sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V6) - 
      sizeof(HS_LIDAR_TAIL_ET_V6)
      );
  if (pHeader->HasSeqNum()){
    const HS_LIDAR_TAIL_SEQ_NUM_ET_V6 *pTailSeqNum =
      reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ET_V6 *>(
      &(udpPacket.buffer[0]) + pHeader->GetPacketSize() - 
      sizeof(HS_LIDAR_CYBER_SECURITY_ET_V6) - 
      sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V6)
      );
    this->CalPktLoss(pTailSeqNum->GetSeqNum());
  }
  this->CalPktTimeLoss(pTail->GetMicroLidarTimeU64());
  const HS_LIDAR_BODY_SEQ2_ET_V6* pSeq2 = 
    reinterpret_cast<const HS_LIDAR_BODY_SEQ2_ET_V6 *>(
      (const unsigned char *)pHeader + 
      sizeof(HS_LIDAR_HEADER_ET_V6) + sizeof(HS_LIDAR_BODY_POINT_ID_ET_V6)
    );
  const HS_LIDAR_BODY_LASTER_UNIT_1_ET_V6* pUnit1 = 
    reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_1_ET_V6 *>(
      (const unsigned char *)pSeq2 + 
      sizeof(HS_LIDAR_BODY_SEQ2_ET_V6)
    );
  const HS_LIDAR_BODY_LASTER_UNIT_2_ET_V6* pUnit2 = 
    reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_2_ET_V6 *>(
      (const unsigned char *)pUnit1 + 
      sizeof(HS_LIDAR_BODY_LASTER_UNIT_1_ET_V6) * (pHeader->GetLaserNum() / pHeader->GetSeqNum())
    );
  
  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }
  frame.host_timestamp = GetMicroTickCountU64();
  frame.work_mode = pTail->m_u8ShutDown;
  frame.distance_unit = pHeader->GetDistUnit();
  frame.per_points_num = (pHeader->GetLaserNum() + pHeader->GetSecRetNum() * pHeader->GetSeqNum()) * pHeader->GetBlockNum();
  frame.scan_complete = false;
  frame.block_num = pHeader->GetBlockNum();
  frame.laser_num = pHeader->GetLaserNum();
  
  int index = frame.packet_num * (pHeader->GetLaserNum() + pHeader->GetSecRetNum() * pHeader->GetSeqNum()) * pHeader->GetBlockNum();
  int index_unit = index;
  int index_unit2 = index + pHeader->GetLaserNum();
  for (int seqId = 0; seqId < pHeader->GetSeqNum(); seqId++) {
    int16_t horizontalAngle = pSeq2->GetHorizontalAngle();
    int16_t verticalAngle = pSeq2->GetVerticalAngle();
    for (int unitId = 0; unitId < (pHeader->GetLaserNum()/pHeader->GetSeqNum()); unitId++) {
      frame.pointData[index_unit].reflectivities = pUnit1->GetReflectivity();
      frame.pointData[index_unit].distances = pUnit1->GetDistance();
      frame.pointData[index_unit].azimuth = horizontalAngle/512.0f;
      frame.pointData[index_unit].elevation = verticalAngle/512.0f;
      frame.pointData[index_unit].chn_index = static_cast<uint8_t>(unitId);
      index_unit++;
      pUnit1 += 1;
    }
    for (int unitId = 0; unitId < pHeader->GetSecRetNum(); unitId++) {
      frame.pointData[index_unit2].reflectivities = pUnit2->GetReflectivity();
      frame.pointData[index_unit2].distances = pUnit2->GetDistance();
      frame.pointData[index_unit2].azimuth = horizontalAngle/512.0f;
      frame.pointData[index_unit2].elevation = verticalAngle/512.0f;
      frame.pointData[index_unit2].chn_index = pUnit2->GetChnIndex();
      index_unit2++;
      pUnit2 += 1;
    }
    // pSeq2 next
    pSeq2 = reinterpret_cast<const HS_LIDAR_BODY_SEQ2_ET_V6 *>(
      (const unsigned char *)pSeq2 + 
      sizeof(HS_LIDAR_BODY_SEQ2_ET_V6) + 
      sizeof(HS_LIDAR_BODY_LASTER_UNIT_1_ET_V6)*(pHeader->GetLaserNum()/pHeader->GetSeqNum()) +
      sizeof(HS_LIDAR_BODY_LASTER_UNIT_2_ET_V6)*(pHeader->GetSecRetNum())
      );
      
    // pUnit next
    pUnit1 = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_1_ET_V6 *>(
      (const unsigned char *)pSeq2 + 
      sizeof(HS_LIDAR_BODY_SEQ2_ET_V6)
    ); 
    pUnit2 = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_2_ET_V6 *>(
    (const unsigned char *)pUnit1 + 
    sizeof(HS_LIDAR_BODY_LASTER_UNIT_1_ET_V6) * (pHeader->GetLaserNum() / pHeader->GetSeqNum())
    );
  } 

  uint16_t nowid = pTail->GetFrameID();
  if (this->use_angle_ && IsNeedFrameSplit(nowid)) {
    frame.scan_complete = true;
  }
  this->last_frameid_  = pTail->GetFrameID();
  frame.packet_num++;
  return 0;
}