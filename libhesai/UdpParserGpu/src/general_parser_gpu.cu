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
#include "general_parser_gpu.h"
#include <string>
#include <sstream>
using namespace hesai::lidar;
template <typename T_Point>
GeneralParserGpu<T_Point>::GeneralParserGpu() {}
template <typename T_Point>
GeneralParserGpu<T_Point>::~GeneralParserGpu() {
  if (corrections_loaded_) {
    corrections_loaded_ = false;
  }
}

template <typename T_Point>
void GeneralParserGpu<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  std::ifstream inFile(firetimes_path, std::ios::in);
  if (inFile.is_open()) {
    std::string lineStr;
    //skip first line
    std::getline(inFile, lineStr); 
    while (getline(inFile, lineStr)) {
      std::stringstream ss(lineStr);
      std::string index, deltTime;
      std::getline(ss, index, ',');
      std::getline(ss, deltTime, ',');
      this->firetime_correction_[std::stoi(index) - 1] = std::stod(deltTime);
    }
  } else {
    LogError("Open correction file failed");
  }
  return;
}

template <typename T_Point>
int GeneralParserGpu<T_Point>::LoadFiretimesString(char *correction_string) {
  LogInfo("load firetimes string");
  return 0;
}

template <typename T_Point>
void GeneralParserGpu<T_Point>::SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw) {
  transform_.x = x;
  transform_.y = y;
  transform_.z = z;
  transform_.roll = roll;
  transform_.yaw = yaw;
  transform_.pitch = pitch;
}

template <typename T_Point>
void GeneralParserGpu<T_Point>::SetOpticalCenterCoordinates(std::string lidar_type) {
  if (lidar_type == "Pandar128E4X" || lidar_type == "OT") {
    optical_center.x = -10.0 / 1000.0;
    optical_center.y = 45.0 / 1000.0;
    optical_center.z = 0.0 / 1000.0;
  } else if(lidar_type == "Pandar128E3X" || lidar_type == "Pandar64E3X" || lidar_type == "Pandar40E3X" || lidar_type == "Pandar90E3X") {
    optical_center.x = -12.0 / 1000.0;
    optical_center.y = 43.56 / 1000.0;
    optical_center.z = 0.0 / 1000.0;
  } else if(lidar_type == "Pandar64E2X" || lidar_type == "Pandar40E2X") {
    optical_center.x = -12.0 / 1000.0;
    optical_center.y = 38.73 / 1000.0;
    optical_center.z = 0.0 / 1000.0;
  } else if(lidar_type == "PandarQT") {
    optical_center.x = -7.2 / 1000.0;
    optical_center.y = 29.8 / 1000.0;
    optical_center.z = 0.0 / 1000.0;
  } else if(lidar_type == "QT128C2X") {
    optical_center.x = 7.2 / 1000.0;
    optical_center.y = 35.4 / 1000.0;
    optical_center.z = 0.0 / 1000.0;
  } else if(lidar_type == "PandarXT32M1" || lidar_type == "PandarXT16M1") {
    optical_center.x = -13.0 / 1000.0;
    optical_center.y = 31.5 / 1000.0;
    optical_center.z = 0.0 / 1000.0;
  } else if(lidar_type == "PandarXT32M2X" || lidar_type == "XTM") {
    optical_center.x = -13.0 / 1000.0;
    optical_center.y = 30.5 / 1000.0;
    optical_center.z = 0.0 / 1000.0;
  } else {
    LogWarning("Parameter(distance_correction_lidar_type) is set to null or error to not enable distance correction");
  }
}

template <typename T_Point>
int GeneralParserGpu<T_Point>::SetXtSpotCorrecion(std::string lidar_type) {
  if (lidar_type == "PandarXT32M1" || lidar_type == "PandarXT16M1" ) {
    xt_spot_correction = true;
  } else {
    return -1;
  }
  return 0;
}