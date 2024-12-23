/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/

#include "lidar_types.h"
#include "lidar.h"
#include <inttypes.h>
#include <stdio.h>
#include "Version.h"
#include <chrono>
using namespace hesai::lidar;

template <typename T_Point>
Lidar<T_Point>::Lidar() {
  udp_parser_ = new UdpParser<T_Point>();
  is_record_pcap_ = false;
  running_ = true;
  udp_thread_running_ = true;
  parser_thread_running_ = true;
  handle_thread_count_ = 0;
  recieve_packet_thread_ptr_ = nullptr;
  mutex_list_ = new std::mutex[GetAvailableCPUNum()];
  handle_buffer_size_ = kPacketBufferSize;
  source_ = nullptr;
  ptc_client_ = nullptr;
  std::fill(init_finish_, init_finish_ + 4, false);
  ptc_port_ = 0;
  udp_port_ = 0;
  parser_thread_ptr_ = nullptr;
  init_set_ptc_ptr_ = nullptr;
  serial_client_ = nullptr;
  source_rs232_ = nullptr;
}

template <typename T_Point>
Lidar<T_Point>::~Lidar() {
  running_ = false;
  udp_thread_running_ = false;
  parser_thread_running_ = false;
  if (recieve_packet_thread_ptr_) {
    recieve_packet_thread_ptr_->join();
    delete recieve_packet_thread_ptr_;
    recieve_packet_thread_ptr_ = nullptr;
  }

  if (parser_thread_ptr_) {
    parser_thread_ptr_->join();
    delete parser_thread_ptr_;
    parser_thread_ptr_ = nullptr;
  }

  if (init_set_ptc_ptr_) {
    init_set_ptc_ptr_->join();
    delete init_set_ptc_ptr_;
    init_set_ptc_ptr_ = nullptr;
  }
  if (handle_thread_count_ > 1) {
    for (int i = 0; i < handle_thread_count_; i++) {
      if (handle_thread_vec_[i]) {
        handle_thread_vec_[i]->join();
        delete handle_thread_vec_[i];
        handle_thread_vec_[i] = nullptr;
      }
    }
  }

  if (udp_parser_ != nullptr) {
    delete udp_parser_;
    udp_parser_ = nullptr;
  }

  if (ptc_client_ != nullptr) {
    delete ptc_client_;
    ptc_client_ = nullptr;
  }

  if (serial_client_ != nullptr) {
    delete serial_client_;
    serial_client_ = nullptr;
  }

  if (source_rs232_ != nullptr) {
    delete source_rs232_;
    source_rs232_ = nullptr;
  }

  if (source_ != nullptr) {
    delete source_;
    source_ = nullptr;
  }

  if (mutex_list_ != nullptr) {
    delete[] mutex_list_;
    mutex_list_ = nullptr;
  }
  Logger::GetInstance().Stop();
}

template <typename T_Point>
std::string Lidar<T_Point>::GetLidarType() {
  return udp_parser_->GetLidarType();
}

template <typename T_Point>
int Lidar<T_Point>::Init(const DriverParam& param) {
  int res = -1;
    /*******************************Init log*********************************************/
    Logger::GetInstance().SetFileName(param.log_path.c_str());
    Logger::GetInstance().setLogTargetRule(param.log_Target);
    Logger::GetInstance().setLogLevelRule(param.log_level);
    // Logger::GetInstance().bindLogCallback(logCallback);
    Logger::GetInstance().Start(); 
    /**********************************************************************************/

    /***************************Init source****************************************/
    udp_port_ = param.input_param.udp_port;
    if (param.input_param.source_type == 2) {
      int packet_interval = 10;
      source_ = new PcapSource(param.input_param.pcap_path, packet_interval);
      source_->Open();
    }
    else if(param.input_param.source_type == 1){
      source_ = new SocketSource(param.input_param.udp_port, param.input_param.multicast_ip_address);
      source_->Open();
    }
    else if(param.input_param.source_type == 4){
      source_ = new SerialSource(param.input_param.rs485_com, param.input_param.rs485_baudrate, param.input_param.point_cloud_baudrate);
      source_rs232_ = new SerialSource(param.input_param.rs232_com, param.input_param.rs232_baudrate);
      source_->SetReceiveStype(SERIAL_POINT_CLOUD_RECV);
      source_->Open();
      if (!source_->IsOpened()) LogError("recv point cloud serial open error");
      source_rs232_->SetReceiveStype(SERIAL_COMMAND_RECV);
      source_rs232_->Open();
      if (!source_rs232_->IsOpened()) LogError("send cmd serial open error");
      serial_client_ = new SerialClient();
      serial_client_->SetSerial(source_rs232_, source_);
    }
    parser_thread_running_ = param.decoder_param.enable_parser_thread;
    udp_thread_running_ = param.decoder_param.enable_udp_thread;
    if (param.decoder_param.socket_buffer_size > 0) {
      source_->SetSocketBufferSize(param.decoder_param.socket_buffer_size);
    }
    
    use_timestamp_type_ = param.decoder_param.use_timestamp_type;
    fov_start_ = param.decoder_param.fov_start;
    fov_end_ = param.decoder_param.fov_end;
    SetThreadNum(param.decoder_param.thread_num);
    /********************************************************************************/
#ifndef JT128_256
    if (param.input_param.source_type == 1) {
      ptc_client_ = new (std::nothrow) PtcClient(param.input_param.device_ip_address
                                                  , param.input_param.ptc_port
                                                  , false
                                                  , param.input_param.ptc_mode
                                                  , 1
                                                  , param.input_param.certFile
                                                  , param.input_param.privateKeyFile
                                                  , param.input_param.caFile
                                                  , 2000
                                                  , 2000);
      init_set_ptc_ptr_ = new std::thread(std::bind(&Lidar<T_Point>::InitSetPtc, this, param));
    }
#endif
    // clock_t start_time, end_time;
    // double time_interval = 0;
    UdpPacket udp_packet;
    // start_time = clock();
    while (udp_parser_->GetParser() == nullptr && running_) {
      int ret = this->GetOnePacket(udp_packet);
      if (ret == -1) continue;
      this->DecodePacket(frame_, udp_packet);
      // end_time = clock();
      // time_interval = double(end_time-start_time) / CLOCKS_PER_SEC;
    }
    frame_.packet_num = 0;
    if (udp_parser_->GetParser() == nullptr) {
      return res;
    }
    udp_parser_->GetParser()->SetLidarType(param.lidar_type);
    udp_parser_->GetParser()->SetOpticalCenterFlag(param.decoder_param.distance_correction_lidar_flag);
    udp_parser_->GetParser()->SetXtSpotCorrection(param.decoder_param.xt_spot_correction);
    init_finish_[FaultMessParse] = true;
    LogDebug("finish 0: The basic initialisation is complete");
    
    /***************************Init decoder****************************************/   
    udp_parser_->SetTransformPara(param.decoder_param.transform_param.x, \
                                  param.decoder_param.transform_param.y, \
                                  param.decoder_param.transform_param.z, \
                                  param.decoder_param.transform_param.roll, \
                                  param.decoder_param.transform_param.pitch, \
                                  param.decoder_param.transform_param.yaw);
    udp_parser_->SetPcapPlay(param.decoder_param.pcap_play_synchronization, param.input_param.source_type);
    udp_parser_->SetFrameAzimuth(param.decoder_param.frame_start_azimuth);
    udp_parser_->GetParser()->EnablePacketLossTool(param.decoder_param.enable_packet_loss_tool);
    udp_parser_->GetParser()->EnablePacketTimeLossTool(param.decoder_param.enable_packet_timeloss_tool);
    udp_parser_->GetParser()->PacketTimeLossToolContinue(param.decoder_param.packet_timeloss_tool_continue);
    switch (param.input_param.source_type)
    {
    case 1: 
#ifdef JT128_256
      LoadCorrectionFile(param.input_param.correction_file_path);
#else
      {
        while ((!ptc_client_->IsOpen()) && running_) 
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (LoadCorrectionForUdpParser() == -1) {
          LogWarning("---Failed to obtain correction file from lidar!---");
          LoadCorrectionFile(param.input_param.correction_file_path);
        }
      }
#endif
      break;
    case 2:
      LoadCorrectionFile(param.input_param.correction_file_path);
      break;
    case 3:
      LoadCorrectionFile(param.input_param.correction_file_path);
      break;
    case 4:
      if (LoadCorrectionForSerialParser(param.input_param.correction_save_path) != 0) {
        LogWarning("---Failed to obtain correction file from lidar!---");
        LoadCorrectionFile(param.input_param.correction_file_path);
      }
      break;
    default:
      break;
    }
    LoadFiretimesFile(param.input_param.firetimes_path);
    init_finish_[PointCloudParse] = true;
    LogDebug("finish 2: The angle calibration file is finished loading");
    /********************************************************************************/
    res = 0;
    return res;
}

template <typename T_Point>
void Lidar<T_Point>::InitSetPtc(const DriverParam param) {
  while(running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!ptc_client_->IsOpen()) continue;
    init_finish_[PtcInitFinish] = true;
    LogDebug("finish 1: ptc connection successfully");
    if (param.input_param.standby_mode != -1) {
      if(!SetStandbyMode(ptc_client_, param.input_param.standby_mode)) {
        LogInfo("set standby mode successed!");
      } else {
        LogWarning("set standby mode failed!");
      }
    }
    if (param.input_param.speed != -1) {
      if(!SetSpinSpeed(ptc_client_, param.input_param.speed)) {
        LogInfo("set speed successed!");
      } else {
        LogWarning("set speed failed!");
      }
    }
    break;
  }
}

template <typename T_Point>
int Lidar<T_Point>::LoadCorrectionFromROSbag() {
  if (udp_parser_) {
    return udp_parser_->LoadCorrectionString(
        (char *)correction_string_.data());
  } else {
    LogError("udp_parser_ nullptr");
    return -1;
  }
  return 0;
}

template <typename T_Point>
int Lidar<T_Point>::LoadCorrectionForUdpParser() {
  u8Array_t sData;
  if (ptc_client_->GetCorrectionInfo(sData) != 0) {
    LogError("LoadCorrectionForUdpParser get correction info fail");
    return -1;
  }
  correction_string_ = sData;
  if (udp_parser_) {
    return udp_parser_->LoadCorrectionString(
        (char *)sData.data());
  } else {
    LogError("udp_parser_ nullptr");
    return -1;
  }
  return 0;
}

template <typename T_Point>
int Lidar<T_Point>::SaveCorrectionFile(std::string correction_save_path) {
  int ret = -1;
  u8Array_t raw_data;
  if (ptc_client_->GetCorrectionInfo(raw_data) != 0) {
    LogError("SaveCorrectionFile get correction info fail");
    return ret;
  }
  correction_string_ = raw_data;
  std::string correction_content_str = (char*) raw_data.data();
  std::ofstream out_file(correction_save_path, std::ios::out);
  if(out_file.is_open()) {
    out_file << correction_content_str;
    ret = 0;
    out_file.close();
    return ret;
  } else {
    LogError("create correction file fail");
    return ret;
  }
}

template <typename T_Point>
int Lidar<T_Point>::LoadCorrectionForSerialParser(std::string correction_save_path) {
  u8Array_t sData;
  int ret = serial_client_->GetCorrectionInfo(sData);
  if (ret != 0) {
    LogError("get correction error, ret:%d", ret);
    return ret;
  }

  // save
  if (correction_save_path != "") {
    std::string correction_content_str = (char*) sData.data();
    std::ofstream out_file(correction_save_path, std::ios::out);
    if(out_file.is_open()) {
      out_file << correction_content_str;
      out_file.close();
    } else {
      LogError("create correction file fail, save correction file error");
    }
  }

  if (udp_parser_) {
    return udp_parser_->LoadCorrectionString(
        (char *)sData.data());
  } else {
    LogError("udp_parser_ nullptr");
    return -1;
  }
  return 0;
}

template <typename T_Point>
int Lidar<T_Point>::SetLidarType(std::string lidar_type) {
  int ret = -1;
  if (udp_parser_) {
    udp_parser_->CreatGeneralParser(lidar_type);
    ret = 0;
  } else
    LogError("udp_parser_ nullptr");

  return ret;
}


template <typename T_Point>
int Lidar<T_Point>::GetOnePacket(UdpPacket &packet) {
  if (origin_packets_buffer_.try_pop_front(packet))  return 0;
  else  return -1;
}

template <typename T_Point>
int Lidar<T_Point>::StartRecordPcap(std::string record_path) {
  int ret = -1;
  if (udp_parser_) {
    udp_parser_->GetPcapSaver()->SetPcapPath(record_path);
    EnableRecordPcap(true);
    udp_parser_->GetPcapSaver()->Save();
  } else
    LogError("udp_parser_ nullptr");

  return ret;
}

template <typename T_Point>
int Lidar<T_Point>::SaveUdpPacket(const std::string &record_path,
                              const UdpFrameArray_t &packets, int port) {
  int ret = -1;
  if (udp_parser_) {
    ret = udp_parser_->GetPcapSaver()->Save(record_path, packets,
                                                         port);
  } else
    LogError("udp_parser_ nullptr");

  return ret;
}

template <typename T_Point>
int Lidar<T_Point>::ComputeXYZI(int packet_index) {

  decoded_packets_buffer_.push_back(std::move(packet_index));
  return 0;

}

template <typename T_Point>
int Lidar<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udp_packet) {
  if (udp_parser_) {
    return udp_parser_->DecodePacket(frame, udp_packet);
  } else
    LogError("udp_parser_ nullptr");

  return -1;
} 


template <typename T_Point>
bool Lidar<T_Point>::ComputeXYZIComplete(uint32_t index) {
  if (udp_parser_ == nullptr) return false;
  if (udp_parser_->GetComputePacketNum() == index) {
    udp_parser_->SetComputePacketNumToZero();
    return true;
  }
  return false;
}

template <typename T_Point>
int Lidar<T_Point>::ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info) {
  if (udp_parser_) {
    return udp_parser_->ParserFaultMessage(udp_packet, fault_message_info);
  } else
    LogError("udp_parser_ nullptr");

  return -1;
}

template <typename T_Point>
void Lidar<T_Point>::LoadCorrectionFile(std::string correction_path) {
  if (udp_parser_) {
    udp_parser_->LoadCorrectionFile(correction_path);
    return ;
  } else
    LogError("udp_parser_ nullptr");

  return ;
}

template <typename T_Point>
int Lidar<T_Point>::LoadCorrectionString(char *correction_string) {
  int ret = -1;
  if (udp_parser_) {
    return udp_parser_->LoadCorrectionString(correction_string);

  } else
    LogError("udp_parser_ nullptr");

  return ret;
}

template <typename T_Point>
void Lidar<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  if (udp_parser_) {
    udp_parser_->LoadFiretimesFile(firetimes_path);
    return ;
  } else
    LogError("udp_parser_ nullptr");

  return ;
}

template <typename T_Point>
int Lidar<T_Point>::SaveUdpPacket(const std::string &record_path,
                              const UdpFrame_t &packets, int port) {
  int ret = -1;
  if (udp_parser_) {
    ret = udp_parser_->GetPcapSaver()->Save(record_path, packets,
                                                         port);
  } else
    LogError("udp_parser_ nullptr");

  return ret;
}

template <typename T_Point>
int Lidar<T_Point>::StopRecordPcap() {
  int ret = -1;
  if (udp_parser_) {
    EnableRecordPcap(false);
    udp_parser_->GetPcapSaver()->close();
  } else
    LogError("udp_parser_ nullptr");

  return ret;
}

template <typename T_Point>
int Lidar<T_Point>::GetGeneralParser(GeneralParser<T_Point> **parser) {
  int ret = 0;
  if (udp_parser_)
    ret = udp_parser_->GetGeneralParser(parser);
  else
    LogError("udp_parser_ nullptr");

  return ret;
}

template <typename T_Point>
void Lidar<T_Point>::RecieveUdpThread() {
  if(!udp_thread_running_) return;
  // uint32_t u32StartTime = GetMicroTickCount();
  LogInfo("Lidar::Recieve Udp Thread start to run");
#ifdef _MSC_VER
  SetThreadPriorityWin(THREAD_PRIORITY_TIME_CRITICAL);
#else
  SetThreadPriority(SCHED_FIFO, SHED_FIFO_PRIORITY_MEDIUM);
#endif
  while (running_) {
    if (source_ == nullptr) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      continue;
    }
    UdpPacket udp_packet;
    int len = source_->Receive(udp_packet, kBufSize);
    if (len == -1) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      continue;
    }
    while(origin_packets_buffer_.full() && running_) std::this_thread::sleep_for(std::chrono::microseconds(1000));
    if(running_ == false) break;
    udp_packet.recv_timestamp = GetMicroTimeU64();
    switch (len) {
      case 0:
        if (is_timeout_ == false) {
          udp_packet.packet_len = AT128E2X_PACKET_LEN; 
          udp_packet.buffer[0] = 0;
          udp_packet.buffer[1] = 0;
          origin_packets_buffer_.emplace_back(udp_packet);
          is_timeout_ = true;
        } 
        break;
      case kFaultMessageLength:
        udp_packet.packet_len = static_cast<uint16_t>(len);
        origin_packets_buffer_.emplace_back(udp_packet);
        break;
      case GPS_PACKET_LEN:
        break;
      default :
        if (len > 0) {
          udp_packet.packet_len = static_cast<uint16_t>(len);
          origin_packets_buffer_.emplace_back(udp_packet);
          is_timeout_ = false;
        }
        break;
  }
    if (udp_packet.packet_len > 0 && is_record_pcap_) {
        udp_parser_->GetPcapSaver()->Dump(udp_packet.buffer, udp_packet.packet_len, udp_port_);
    }
  }
  return;
}

template <typename T_Point>
void Lidar<T_Point>::ParserThread() {
  if(!parser_thread_running_) return;
  int nUDPCount = 0;
  LogInfo("Lidar::ParserThread start to run");
#ifdef _MSC_VER
  SetThreadPriorityWin(THREAD_PRIORITY_TIME_CRITICAL);
#else
  SetThreadPriority(SCHED_FIFO, SHED_FIFO_PRIORITY_MEDIUM);
#endif
  while (running_) {
    int decoded_packet_index;
    bool decoded_result = decoded_packets_buffer_.try_pop_front(decoded_packet_index);
    // decoded_packet.use_timestamp_type = use_timestamp_type_;
    if (!decoded_result) {
      continue;
    }
    if (handle_thread_count_ < 2) {
        udp_parser_->ComputeXYZI(frame_, decoded_packet_index);
      continue;
    } else {
      nUDPCount = nUDPCount % handle_thread_count_;
      mutex_list_[nUDPCount].lock();
      handle_thread_packet_buffer_[nUDPCount].push_back(decoded_packet_index);

      if (handle_thread_packet_buffer_[nUDPCount].size() > handle_buffer_size_) {
        handle_thread_packet_buffer_[nUDPCount].pop_front();
      }
      mutex_list_[nUDPCount].unlock();
      nUDPCount++;
    }
  }
  return;
}

template <typename T_Point>
void Lidar<T_Point>::HandleThread(int nThreadNum) {
  // struct timespec timeout;
#ifdef _MSC_VER
  SetThreadPriorityWin(THREAD_PRIORITY_TIME_CRITICAL);
#else
  SetThreadPriority(SCHED_FIFO, SHED_FIFO_PRIORITY_MEDIUM);
#endif
  if(!parser_thread_running_) return;
  while (running_) {
    mutex_list_[nThreadNum].lock();
    if (handle_thread_packet_buffer_[nThreadNum].size() > 0) {
      int decoded_packet_index = handle_thread_packet_buffer_[nThreadNum].front();
      handle_thread_packet_buffer_[nThreadNum].pop_front();
      udp_parser_->ComputeXYZI(frame_, decoded_packet_index);
    }
    mutex_list_[nThreadNum].unlock();
  }
}

template <typename T_Point>
void Lidar<T_Point>::SetThreadNum(int nThreadNum) {
  if (nThreadNum > GetAvailableCPUNum() - 2) {
    nThreadNum = GetAvailableCPUNum() - 2;
    if (nThreadNum <= 0) {
      nThreadNum = 1;
    }
  }

  if (handle_thread_count_ == nThreadNum) {
    return;
  }

  running_ = false;

  if (recieve_packet_thread_ptr_ != nullptr) {
    recieve_packet_thread_ptr_->join();
    delete recieve_packet_thread_ptr_;
    recieve_packet_thread_ptr_ = nullptr;
  }

  for (int i = 0; i < handle_thread_count_; i++) {
    if (handle_thread_vec_[i] != nullptr) {
      handle_thread_vec_[i]->join();
      delete handle_thread_vec_[i];
      handle_thread_vec_[i] = nullptr;
    }

    handle_thread_packet_buffer_[i].clear();
  }

  running_ = true;
  if (nThreadNum > 1) {
    handle_thread_vec_.resize(nThreadNum);
    handle_thread_packet_buffer_.resize(nThreadNum);

    for (int i = 0; i < nThreadNum; i++) {
      handle_thread_vec_[i] = new std::thread(
          std::bind(&Lidar::HandleThread, this, std::placeholders::_1), i);
    }
  }

  handle_thread_count_ = nThreadNum;
  recieve_packet_thread_ptr_ =
      new std::thread(std::bind(&Lidar::RecieveUdpThread, this));
  parser_thread_ptr_ =
      new std::thread(std::bind(&Lidar::ParserThread, this));    
}
template <typename T_Point>
void Lidar<T_Point>::ClearPacketBuffer() {
  decoded_packets_buffer_.eff_clear();
  if (handle_thread_count_ > 1) {
    for (int i = 0; i < handle_thread_count_; i++) {
      handle_thread_packet_buffer_[i].clear();
    }
  }
}
template <typename T_Point>
void Lidar<T_Point>::SetSource(Source **source) {
  source_ = *source;
}


template <typename T_Point>
bool Lidar<T_Point>::IsPlayEnded()
{
  if (source_ == nullptr)
  {
    return false;
  }
  return source_->is_pcap_end;
}

template <typename T_Point>
int Lidar<T_Point>::SetStandbyMode(PtcClient *Ptc_client, int standby_mode)
{
  u8Array_t input1, output1;
  input1.push_back(static_cast<uint8_t>(standby_mode));
  return Ptc_client->QueryCommand(input1, output1, CMD_SET_STANDBY_MODE);
}

template <typename T_Point>
int Lidar<T_Point>::SetSpinSpeed(PtcClient *Ptc_client, int speed)
{
  u8Array_t input2, output2;
  input2.push_back(static_cast<uint8_t>(speed >> 8));
  input2.push_back(static_cast<uint8_t>(speed));
  return Ptc_client->QueryCommand(input2, output2, CMD_SET_SPIN_SPEED);
}

template <typename T_Point>
UdpParser<T_Point> *Lidar<T_Point>::GetUdpParser() { return udp_parser_; }

template <typename T_Point>
void Lidar<T_Point>::SetUdpParser(UdpParser<T_Point> *udpParser) {
  udp_parser_ = udpParser;
}

template <typename T_Point>
void Lidar<T_Point>::EnableRecordPcap(bool bRecord) {
  is_record_pcap_ = bRecord;
}

