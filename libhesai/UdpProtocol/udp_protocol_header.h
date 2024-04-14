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
#include "plat_utils.h"
#ifndef LIDAR_PROTOCOL_HEADER_H
#define LIDAR_PROTOCOL_HEADER_H
#ifdef _MSC_VER
#include <winsock2.h>
#include <windows.h>
#endif
namespace hesai
{
namespace lidar
{
#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

static bool IsLittleEndian() {
  const int a = 1;
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&a);

  return *p == 1 ? true : false;
}

template <typename T>
T little_to_native(T data) {
  T out = 0;
  if (IsLittleEndian()) {
    out = data;
  } else {
    unsigned char *pSrc = reinterpret_cast<unsigned char *>(&data +
                                                            sizeof(data) - 1),
                  *pDst = reinterpret_cast<unsigned char *>(&out);
    for (size_t i = 0; i < sizeof(data); i++) {
      *pDst++ = *pSrc--;
    }
  }
  return out;
}

struct HS_LIDAR_PRE_HEADER {
  static const uint16_t kDelimiter = 0xffee;
  // major version
  static const uint8_t kME = 0x01;  // mechanical lidar
  static const uint8_t kQT = 0x03;
  static const uint8_t kST = 0x04;
  static const uint8_t kFT = 0x07;
  static const uint8_t kXT = 0x06;

  // minor version
  static const uint8_t kV1 = 0x01;  // reserved
  static const uint8_t kV2 = 0x02;  // used by P128 series / POROS
  static const uint8_t kV3 = 0x03;  // used by P128 series
  static const uint8_t kV4 = 0x04;  // used by P128 series

  // status info version
  static const uint8_t kStatusInfoV0 = 0;

  uint16_t m_u16Delimiter;
  uint8_t m_u8VersionMajor;
  uint8_t m_u8VersionMinor;
  uint8_t m_u8StatusInfoVersion;
  uint8_t m_u8Reserved1;

  bool IsValidDelimiter() const {
    return little_to_native(m_u16Delimiter) == kDelimiter;
  }
  uint16_t GetDelimiter() const { return little_to_native(m_u16Delimiter); }
  uint8_t GetVersionMajor() const { return m_u8VersionMajor; }
  uint8_t GetVersionMinor() const { return m_u8VersionMinor; }
  uint8_t GetStatusInfoVersion() const { return m_u8StatusInfoVersion; }

  void Init(uint8_t u8VerMajor, uint8_t u8VerMinor, 
            uint8_t u8StatusInfoVer = kStatusInfoV0) {
    m_u16Delimiter = 0xffee;
    m_u8VersionMajor = u8VerMajor;
    m_u8VersionMinor = u8VerMinor;
    m_u8StatusInfoVersion = u8StatusInfoVer;
  }

  void Print() const {
    printf("HS_LIDAR_PRE_HEADER:\n");
    printf("Delimiter:%02x, valid:%d, Ver Major: %02x, minor: %02x, "
           "StatusInfoVer:%02x\n",
           GetDelimiter(), IsValidDelimiter(), GetVersionMajor(),
           GetVersionMinor(), GetStatusInfoVersion());
  }
} PACKED;

struct ReservedInfo1 {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;

  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }
} PACKED;

struct ReservedInfo2 {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;
  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }

  void Print() const {
    printf("lowerBoard ID:%u, STS:0x%02x\n", m_u8ID, m_u16Sts);
  }
} PACKED;

struct ReservedInfo3 {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;
  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }
} PACKED;

#ifdef _MSC_VER
#pragma pack(pop)
#endif
}  // namespace lidar
}  // namespace hesai
#endif
