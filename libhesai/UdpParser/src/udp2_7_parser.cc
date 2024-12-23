
#include "udp2_7_parser.h"
#include "udp_protocol_v2_7.h"
#include "udp_protocol_header.h"
#include "udp_parser.h"

using namespace hesai::lidar;

template<typename T_Point>
Udp2_7Parser<T_Point>::Udp2_7Parser() {
  this->get_correction_file_ = false;
}

template<typename T_Point>
Udp2_7Parser<T_Point>::~Udp2_7Parser() { LogInfo("release Udp2_7Parser"); }


// The main purpose of this function is to open, read, and parse a lidar calibration file, and print appropriate messages based on the parsing results.
// This function can read both .bin and .csv files.
template<typename T_Point>
void Udp2_7Parser<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
  int type = 0;
  size_t length = lidar_correction_file.length();
  if (length >= 4) {
    std::string extension = lidar_correction_file.substr(length - 4);
    if (extension == ".bin" || extension == ".dat") {
        type = 1; //  .bin
    } else if (extension == ".csv") {
        type = 2; //  .csv
    } else {
        // type = 0; //  wrong
        return;
    }   
  }

  LogInfo("load correction file from local correction now!");
  std::ifstream fin(lidar_correction_file);
  if (fin.is_open()) {
    LogDebug("Open correction file success!");
    fin.seekg(0, std::ios::end);
    int len = static_cast<int>(fin.tellg());
    // return the begin of file
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[len];
    // file --> buffer
    fin.read(buffer, len);
    fin.close();
    int ret = 0;
    if (type == 1) {
      ret = LoadCorrectionString(buffer);
    } else if(type == 2) {
      ret = LoadCorrectionCsvData(buffer);
    } else {
      LogWarning("Invalid suffix name");
      ret = -1;
    }
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local Correction file Error!");
    } else {
      LogInfo("Parse local Correction file Success!!!");
    }
  } else { // open failed
    LogError("Open correction file failed");
    return;
  }
}

// csv ----> correction
template<typename T_Point>
int  Udp2_7Parser<T_Point>::LoadCorrectionCsvData(char *correction_string)
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
  int zeroCount = 0;
  corrections_.min_version = 1;
  while (std::getline(ifs, line)) {
    if (lineCount == 6 && zeroCount == 6) {
      corrections_.min_version = 4;
    }
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
    if (azimuth == 0) zeroCount++;
  }

  if (corrections_.min_version == 1) {
    for (int i = 0; i < lineCount; ++i) {
      corrections_.azimuths[i] = azimuth_list[i];
      corrections_.elevations[i] = elevation_list[i];
      LogDebug("%d %f %f", i, corrections_.azimuths[i], corrections_.elevations[i]);
    }
  } else if (corrections_.min_version == 4) {
    corrections_.elevations[0] = elevation_list[0];
    corrections_.elevations[1] = elevation_list[1];
    corrections_.gamma_f[0] = elevation_list[2];
    corrections_.gamma_f[1] = elevation_list[3];
    corrections_.gamma_f[2] = elevation_list[4];
    corrections_.gamma_f[3] = elevation_list[5];
    for (int i = 6; i < lineCount - 4; i++) {
      corrections_.azimuths[i - 3] = azimuth_list[i];
      corrections_.elevations[i - 3] = elevation_list[i];
      LogDebug("%d %f %f", i - 3, corrections_.azimuths[i - 3], corrections_.elevations[i - 3]);
    }
    for (int i = 0; i < 4; i++) {
      corrections_.azimuth_offset_delta_f[i] = azimuth_list[lineCount - 4 + i];
      corrections_.elevation_offset_delta_f[i] = elevation_list[lineCount - 4 + i];
    }
  } else {
    return -1;
  }
  
  this->get_correction_file_ = true;
  return 0;
}


// buffer(.bin) ---> correction
template<typename T_Point>
int Udp2_7Parser<T_Point>::LoadCorrectionString(char *data) {
  try {
    char *p = data;
    struct ETCorrectionsHeader ETheader = *((struct ETCorrectionsHeader* )p);
    if (0xee == ETheader.delimiter[0] && 0xff == ETheader.delimiter[1]) {
      switch (ETheader.min_version) {
        case 1: {
          ETCorrectionsHeader_V1V2 correction_v1;
          memcpy((void *)&correction_v1, p, sizeof(struct ETCorrectionsHeader_V1V2));
          corrections_.header.getDataFromV1V2(correction_v1);
          corrections_.min_version = corrections_.header.min_version;
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
          ETCorrectionsHeader_V1V2 correction_v2;
          memcpy((void *)&correction_v2, p, sizeof(struct ETCorrectionsHeader_V1V2));
          p += sizeof(ETCorrectionsHeader_V1V2);
          corrections_.header.getDataFromV1V2(correction_v2);
          corrections_.min_version = corrections_.header.min_version;
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
        case 3: {
          memcpy((void *)&corrections_.header, p, sizeof(struct ETCorrectionsHeader));
          p += sizeof(ETCorrectionsHeader);
          corrections_.min_version = corrections_.header.min_version;
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
        case 4: {
          ETCorrectionsHeader_V3_4 correction_v3_4;
          memcpy((void *)&correction_v3_4, p, sizeof(struct ETCorrectionsHeader_V3_4));
          p += sizeof(ETCorrectionsHeader_V3_4);
          corrections_.header.getDataFromV3_4(correction_v3_4);
          corrections_.turn_number_per_frame = correction_v3_4.turn_number_per_frame;
          corrections_.min_version = corrections_.header.min_version;
          auto N = corrections_.header.channel_number;
          auto M = corrections_.header.mirror_number_reserved3;
          auto T = corrections_.turn_number_per_frame;
          auto division = corrections_.header.angle_division;
          if ((N > ET_MAX_CHANNEL_NUM - 3) || division == 0 || M > 8) {
            LogError("data error: channel_num is %u, division is %u, mirror_number is %u", N, division, M);
            return -1;
          }
          memcpy((void *)&corrections_.gamma, p,
                 sizeof(int16_t) * M);
          p += sizeof(int16_t) * M;
          memcpy((void *)&corrections_.raw_azimuths, p,
                 sizeof(int16_t) * N);
          p += sizeof(int16_t) * N;
          memcpy((void *)&corrections_.raw_elevations, p,
               sizeof(int16_t) * N);
          p += sizeof(int16_t) * N;
          memcpy((void *)&corrections_.azimuth_offset_delta, p,
               sizeof(int16_t) * M);
          p += sizeof(int16_t) * M;
          memcpy((void *)&corrections_.elevation_offset_delta, p,
               sizeof(int16_t) * M);
          p += sizeof(int16_t) * M;
          corrections_.elevations[0] = ((float)(corrections_.header.apha)) / division;
          corrections_.elevations[1] = ((float)(corrections_.header.beta)) / division;
          for (int i = 0; i < M; i++) {
            corrections_.gamma_f[i] = ((float)(corrections_.gamma[i])) / division;
            corrections_.azimuth_offset_delta_f[i] = ((float)(corrections_.azimuth_offset_delta[i])) / division;
            corrections_.elevation_offset_delta_f[i] = ((float)(corrections_.elevation_offset_delta[i])) / division;
          }
          for (int i = 0; i < N; i++) {
            corrections_.azimuths[i + 3] = ((float)(corrections_.raw_azimuths[i])) / division;
            corrections_.elevations[i + 3] = ((float)(corrections_.raw_elevations[i])) / division;
          }
          corrections_.azimuth_adjust_interval = *((char*)p);
          p = p + 1;
          corrections_.elevation_adjust_interval = *((char*)p);
          p = p + 1;
          int angle_offset_len = int(T * (120 / (corrections_.azimuth_adjust_interval * 0.1) + 1) * (3.2 / (corrections_.elevation_adjust_interval * 0.1) + 1));
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
bool Udp2_7Parser<T_Point>::IsNeedFrameSplit(uint16_t frame_id) {
  if ( frame_id != this->last_frameid_  && this->last_frameid_ >= 0) {
      return true;
  }
  return false;
}

// get elevations[i]
template<typename T_Point>
int16_t Udp2_7Parser<T_Point>::GetVecticalAngle(int channel) {
  if (this->get_correction_file_ == false) {
    LogError("GetVecticalAngle: no correction file get, Error");
    return -1;
  }
  return corrections_.elevations[channel];
}


template<typename T_Point>
int Udp2_7Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  // get configer information from corrections_, 1 block 1 corrections_ 
  // float division = (float)corrections_.header.angle_division;
  float apha =  corrections_.elevations[0];
  float beta =  corrections_.elevations[1];
  float gamma =  corrections_.elevations[2];
  uint8_t mirror_index = (frame.pointData[packet_index * frame.per_points_num].mirror_index >> 4) & 0x0F;
  uint8_t turn_index = frame.pointData[packet_index * frame.per_points_num].mirror_index & 0x0F;
  if (corrections_.min_version == 4) {
    gamma = corrections_.gamma_f[mirror_index];
  }
  // get the laser_num
  uint16_t lasernum = frame.laser_num;
  for (int blockId = 0; blockId < frame.block_num; blockId++) {
    // T_Point point;
    // float delte_apha = 0;
    // float delte_theta = 0;
    for (int i = 0; i < lasernum; i++) {
      int point_index = packet_index * frame.per_points_num + blockId * frame.laser_num + i; 
      // get phi and psi and distance
      float raw_azimuth = frame.pointData[point_index].azimuth;
      float raw_elevation = frame.pointData[point_index].elevation;
      float distance = static_cast<float>(frame.pointData[point_index].distances * frame.distance_unit);
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
      if (corrections_.header.min_version == 2 || corrections_.header.min_version == 3) {
        azi_h = azi_h + corrections_.getAziAdjustV2(azi_h - 90, elv_h);
        elv_h = elv_h + corrections_.getEleAdjustV2(azi_h - 90, elv_h);
      } else if (corrections_.min_version == 4) {
        azi_h += corrections_.getAziAdjustV4(azi_h - 90, elv_h, turn_index);
        elv_h += corrections_.getEleAdjustV4(azi_h - 90, elv_h, turn_index);
        azi_h += corrections_.azimuth_offset_delta_f[mirror_index];
        elv_h += corrections_.elevation_offset_delta_f[mirror_index];
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
      setRing(frame.points[point_index], static_cast<uint16_t>(i));
    }
  }
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
int Udp2_7Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
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

  const HS_LIDAR_HEADER_ET_V7* pHeader =
    reinterpret_cast<const HS_LIDAR_HEADER_ET_V7 *>(
      &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  const HS_LIDAR_TAIL_ET_V7 *pTail =
    reinterpret_cast<const HS_LIDAR_TAIL_ET_V7 *>(
      &(udpPacket.buffer[0]) +  pHeader->GetPacketSize() - 
      sizeof(HS_LIDAR_CYBER_SECURITY_ET_V7) - 
      sizeof(HS_LIDAR_TAIL_CRC_ET_V7) - 
      sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V7) - 
      sizeof(HS_LIDAR_TAIL_ET_V7)
      );
  if (pHeader->HasSeqNum()){
    const HS_LIDAR_TAIL_SEQ_NUM_ET_V7 *pTailSeqNum =
      reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ET_V7 *>(
      &(udpPacket.buffer[0]) + pHeader->GetPacketSize() - 
      sizeof(HS_LIDAR_CYBER_SECURITY_ET_V7) - 
      sizeof(HS_LIDAR_TAIL_CRC_ET_V7) - 
      sizeof(HS_LIDAR_TAIL_SEQ_NUM_ET_V7)
      );
    this->CalPktLoss(pTailSeqNum->GetSeqNum());
  }
  this->CalPktTimeLoss(pTail->GetMicroLidarTimeU64());
  const HS_LIDAR_BODY_SEQ3_ET_V7* pSeq3 = 
    reinterpret_cast<const HS_LIDAR_BODY_SEQ3_ET_V7 *>(
      (const unsigned char *)pHeader + 
      sizeof(HS_LIDAR_HEADER_ET_V7) + 2
    );
  const HS_LIDAR_BODY_LASTER_UNIT_ET_V7* pUnit = 
    reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V7 *>(
      (const unsigned char *)pSeq3 + 
      sizeof(HS_LIDAR_BODY_SEQ3_ET_V7)
    );

  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }
  frame.host_timestamp = GetMicroTickCountU64();
  frame.work_mode = pTail->m_u8ShutDown;
  frame.return_mode = pTail->m_u8EchoMode;
  frame.distance_unit = pHeader->GetDistUnit();
  frame.per_points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.scan_complete = false;
  frame.block_num = pHeader->GetBlockNum();
  frame.laser_num = pHeader->GetLaserNum();
  uint8_t mirror_index = pTail->GetMirrorIndex();
  
  int index = frame.packet_num * pHeader->GetBlockNum() * pHeader->GetLaserNum();
  for (int blockId = 0; blockId < pHeader->GetBlockNum(); blockId++) {
    for (int seqId = 0; seqId < pHeader->GetSeqNum(); seqId++) {
      int16_t horizontalAngle = pSeq3->GetHorizontalAngle();
      int16_t verticalAngle = pSeq3->GetVerticalAngle();
      for (int unitId = 0; unitId < (pHeader->GetLaserNum()/pHeader->GetSeqNum()); unitId++){
        frame.pointData[index].reflectivities = pUnit->GetReflectivity();
        frame.pointData[index].distances = pUnit->GetDistance();
        frame.pointData[index].azimuth = horizontalAngle/512.0f;
        frame.pointData[index].elevation = verticalAngle/512.0f;
        frame.pointData[index].mirror_index = mirror_index;
        if (this->get_firetime_file_) {
          int lidar_i = (pHeader->GetLaserNum()/pHeader->GetSeqNum()) * seqId + unitId;
          frame.pointData[index].azimuth += this->GetFiretimesCorrection(lidar_i, 2.4);  // 1200 / 1000 * 2 固定为1200转，码盘角与实际角度倍数为2
        }
        index++;
        pUnit += 1;
      }
      // pSeq3 next
      pSeq3 = reinterpret_cast<const HS_LIDAR_BODY_SEQ3_ET_V7 *>(
        (const unsigned char *)pSeq3 + 
        sizeof(HS_LIDAR_BODY_SEQ3_ET_V7) + 
        sizeof(HS_LIDAR_BODY_LASTER_UNIT_ET_V7)*(pHeader->GetLaserNum()/pHeader->GetSeqNum())  
       );
       
      // pUnit next
      pUnit = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V7 *>(
        (const unsigned char *)pSeq3 + 
        sizeof(HS_LIDAR_BODY_SEQ3_ET_V7)
      ); 
    } 
    if (blockId < (pHeader->GetBlockNum())) {
      // skip point id
      pSeq3 = reinterpret_cast<const HS_LIDAR_BODY_SEQ3_ET_V7 *>(
        (const unsigned char *)pSeq3 + 2  
        );
      pUnit = reinterpret_cast<const HS_LIDAR_BODY_LASTER_UNIT_ET_V7 *>(
        (const unsigned char *)pUnit + 2
        );
    } // end if (blockId < (pHeader->GetBlockNum()))
  }
  uint16_t frame_id = pTail->GetFrameID();
  if (this->use_angle_ && IsNeedFrameSplit(frame_id)) {
    frame.scan_complete = true;
  }
  this->last_frameid_  = pTail->GetFrameID();
  frame.packet_num++;
  return 0;
}