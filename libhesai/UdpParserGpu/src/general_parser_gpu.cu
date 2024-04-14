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
int GeneralParserGpu<T_Point>::LoadCorrectionFile(std::string correction_path) {
  int ret = 0;
  std::ifstream fin(correction_path);
  if (fin.is_open()) {
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    ret = LoadCorrectionString(buffer);
    if (ret != 0) {
      std::cout << "Parse local correction file Error" << std::endl;
      return -1;
    }
  } else {
    std::cout << "Open correction file failed" << std::endl;
    return -1;
  }
  return 0;
}
template <typename T_Point>
int GeneralParserGpu<T_Point>::LoadCorrectionString(char *correction_content) {
  std::string correction_content_str = correction_content;
  std::istringstream ifs(correction_content_str);
  std::string line;

  std::getline(ifs, line);  // skip first line "Laser id,Elevation,Azimuth" or "eeff"

  // float elevation_list[MAX_LASER_NUM], azimuth_list[MAX_LASER_NUM];

  std::vector<std::string> vfirstLine;
  split_string(vfirstLine, line, ',');
  if (vfirstLine[0] == "EEFF" || vfirstLine[0] == "eeff") {
    std::getline(ifs, line);  // skip second line
  }

  int lineCount = 0;
  while (std::getline(ifs, line)) {
    std::vector<std::string> vLineSplit;
    split_string(vLineSplit, line, ',');
    if (vLineSplit.size() < 3) {  // skip error line or hash value line
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
      return -1;
    }
    // elevation_list[laserId - 1] = elevation;
    // azimuth_list[laserId - 1] = azimuth;
  }
  // this->elevation_correction_.resize(lineCount);
  // this->azimuth_collection_.resize(lineCount);

  // for (int i = 0; i < lineCount; ++i) {
  //   this->elevation_correction_[i] = elevation_list[i];
  //   this->azimuth_collection_[i] = azimuth_list[i];
  // }
  return 0;
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
    std::cout << "Open correction file failed" << std::endl;
  }
  return;
}

template <typename T_Point>
int GeneralParserGpu<T_Point>::LoadFiretimesString(char *correction_string) {
  printf("load firetimes string\n");
  return 0;
}

template <typename T_Point>
int GeneralParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
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

