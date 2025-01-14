/*
 * Copyright (C) 2018 Hesai Tech<http://www.hesaitech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * File:   serial_source.cc
 * Author: chang xingshuo<changxingshuo@hesaitech.com>
 *
 * Created on Sep 10, 2024, 19:56 PM  
 */

#include "serial_source.h"
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <chrono>
#include <sys/ioctl.h>
using namespace hesai::lidar;
SerialSource::SerialSource(const std::string dev, int baudrate, int point_cloud_baudrate) : m_iFd(-1) {
  dev_ = dev;
  baudrate_ = baudrate;
  serialData = new uint8_t[kDataMaxLength];
  serialData_backup = new uint8_t[kDataMaxLength];
  dataIndex = 0;
  dataLength = 0;
  point_cloud_baudrate_ = point_cloud_baudrate;
}

SerialSource::~SerialSource() {
  delete[] serialData;
  delete[] serialData_backup;
  Close();
}

bool SerialSource::Open() {
  int baudrate = 0;
  if (receiveStype == SERIAL_POINT_CLOUD_RECV) 
    baudrate = point_cloud_baudrate_;
  else if (receiveStype == SERIAL_COMMAND_RECV)
    baudrate = baudrate_;
  else {
    LogError("not support receiveStype, open error");
    return false;
  }
  if (Open(dev_.c_str(), baudrate) == 0)
    return true;
  return false;
}

bool SerialSource::IsOpened() {
  if (m_iFd >= 0) {
    return true;
  }
  return false;
}

int SerialSource::Open(const char *dev, int baudrate) {
  Close();
  /* Open SerialSource port */
  if ((m_iFd = open(dev, O_RDWR | O_NOCTTY)) < 0) {
    // perror("open");
    return -1;
  }

  struct termios2 tio { };
  if (0 != ioctl(m_iFd, TCGETS2, &tio)) {
    // perror("ioctl");
    close(m_iFd);
    m_iFd= -1;
    return -1;
  }
  /**
  * Setting serial port,8E2, non-blocking.100Kbps
  */
  tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
          | IXON);
  tio.c_iflag |= (INPCK | IGNPAR);
  tio.c_oflag &= ~OPOST;
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tio.c_cflag &= ~(CSIZE | CBAUD);
  /**
  * use BOTHER to specify speed directly in c_[io]speed member
  */
  tio.c_cflag |= (CS8 | CLOCAL | BOTHER | CREAD);
  tio.c_cflag &= ~(CSTOPB | PARENB);
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;
  tio.c_cc[VMIN] = 25;
  tio.c_cc[VTIME] = 1;
  if (0 != ioctl(m_iFd, TCSETS2, &tio)) {
    // perror("ioctl");
    close(m_iFd);
    m_iFd= -1;
    return -1;
  }
  return 0;
}

void SerialSource::Close() {
  if (m_iFd >= 0) {
    close(m_iFd);
    m_iFd = -1;
  }
}

int SerialSource::WaitRead(int32_t timeout) {
  int ret;
  fd_set rfds;
  struct timeval tv_timeout;

  FD_ZERO(&rfds);
  FD_SET(m_iFd, &rfds);

  if (timeout >= 0) {
    tv_timeout.tv_sec = timeout / 1000;
    tv_timeout.tv_usec = (timeout % 1000) * 1000;

    ret = select(m_iFd + 1, &rfds, NULL, NULL, &tv_timeout);
  } else {
    ret = select(m_iFd + 1, &rfds, NULL, NULL, NULL);
  }

  if (ret < 0) {
    return ret;
  } else if (ret == 0) ///< Timeout / nothing more to read
  {
    return 0;
  }

  return 1;
}

int SerialSource::Receive(UdpPacket& udpPacket, uint16_t u16Len, int flags, int timeout) {
  if (receiveStype == SERIAL_POINT_CLOUD_RECV) {
    if (kDataMaxLength < u16Len) {
      return -1;
    }
    ssize_t ret = 0;

    if (m_iFd < 0)
      return -EINVAL;

    /* Wait until data coming in*/
    ret = WaitRead(timeout);
    if (ret <= 0)
      return ret;
    if (dataLength + u16Len > kDataMaxLength) {
      memcpy(serialData_backup, serialData + dataIndex, dataLength - dataIndex);
      memcpy(serialData, serialData_backup, dataLength - dataIndex);
      dataLength = dataLength - dataIndex;
      dataIndex = 0;
    }
    if (dataLength + u16Len <= kDataMaxLength) {
      ret = read(m_iFd, serialData + dataLength, u16Len);
      if (ret > 0) {
        dataLength += ret;
      }
    }
    while (dataLength - dataIndex >= 80) {
      if (serialData[dataIndex] != 0xee || serialData[dataIndex + 1] != 0xff) {
        dataIndex++;
        continue;
      }
      if (serialData[dataIndex + 5] == 0x00) {
        uint32_t expectedCRC = (serialData[dataIndex + 79] << 24) |
                       (serialData[dataIndex + 78] << 16) |
                       (serialData[dataIndex + 77] << 8)  |
                       serialData[dataIndex + 76];
        uint32_t ret = CRCCalc(&serialData[dataIndex], 76, 0);
        if ( ret != expectedCRC) {
          dataIndex += 80;
          continue;
        }
        memcpy(udpPacket.buffer, serialData + dataIndex, 80);
        dataIndex += 80;
        return 80;
      } else if (serialData[dataIndex + 5] == 0x01) {
        uint32_t expectedCRC = (serialData[dataIndex + 33] << 24) |
                       (serialData[dataIndex + 32] << 16) |
                       (serialData[dataIndex + 31] << 8)  |
                       serialData[dataIndex + 30];
        uint32_t ret = CRCCalc(&serialData[dataIndex], 30, 0);
        if (ret != expectedCRC) {
          dataIndex += 34;
          continue;
        }
        memcpy(udpPacket.buffer, serialData + dataIndex, 34);
        dataIndex += 34;
        return 34;
      } else {
        dataIndex += 6;
      }
    }
  } else if (receiveStype == SERIAL_COMMAND_RECV) {
    if (flags == 0) return -1;
    auto time_beg = std::chrono::steady_clock::now();
    ssize_t ret = 0;
    while (1) {
      auto time_end = std::chrono::steady_clock::now();
      std::chrono::duration<double, std::milli> time_diff = time_end - time_beg;
      if (time_diff.count() > timeout) return -1;
      ret = WaitRead(100);
      if (ret <= 0) {
        continue;
      }
      if (dataLength + u16Len > kDataMaxLength) {
        memcpy(serialData_backup, serialData + dataIndex, dataLength - dataIndex);
        memcpy(serialData, serialData_backup, dataLength - dataIndex);
        dataLength = dataLength - dataIndex;
        dataIndex = 0;
      }
      if (dataLength + u16Len <= kDataMaxLength) {
        ret = read(m_iFd, serialData + dataLength, u16Len);
        if (ret > 0) {
          dataLength += ret;
        }
      }
      while (dataLength - dataIndex >= 7) {
        if (serialData[dataIndex] != 0x24 || serialData[dataIndex + 1] != 0x4C || serialData[dataIndex + 2] != 0x44
            || serialData[dataIndex + 3] != 0x41 || serialData[dataIndex + 4] != 0x43 || serialData[dataIndex + 5] != 0x4B
            || serialData[dataIndex + 6] != 0x2C) {
          dataIndex++;
          continue;
        }
        int i = 7;
        while (dataLength - dataIndex - i >= 2) {
          if (serialData[dataIndex + i] == 0xEE && serialData[dataIndex + i + 1] == 0xFF) {
            memcpy(udpPacket.buffer, serialData + dataIndex, i + 2);
            dataIndex += i + 2;
            return i + 2;
          }
          i++;
        }
        break;
      }
    }
  }
  return -1;
}

int SerialSource::Send(uint8_t *u8Buf, uint16_t u16Len, int flags) {
  if (m_iFd < 0)
    return -EINVAL;

  return write(m_iFd, u8Buf, u16Len);
}

int SerialSource::Flush() {
  if (m_iFd >= 0)
    return tcflush(m_iFd, TCIOFLUSH);

  return 0;
}

int SerialSource::FlushInput() {
  if (m_iFd >= 0)
    return tcflush(m_iFd, TCIFLUSH);

  return 0;
}

int SerialSource::FlushOutput() {
  if (m_iFd >= 0)
    return tcflush(m_iFd, TCOFLUSH);

  return 0;
}

void SerialSource::SetReceiveStype(int type) {
  FlushInput();
  receiveStype = type;
  dataIndex = 0;
  dataLength = 0;
}

uint32_t SerialSource::CRCCalc(const uint8_t *bytes, int len, int zeros_num) {
  CRCInit();
    uint32_t i_crc = 0xffffffff;
    for (int i = 0; i < len; i++)
        i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ bytes[i]) & 0xff];
    for (int i = 0; i < zeros_num; i++)
        i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ 0) & 0xff];
    return i_crc;
}

void SerialSource::CRCInit() {
    static bool initialized = false;
    if (initialized) {
        return;
    }
    initialized = true;

    uint32_t i, j, k;
    for (i = 0; i < 256; i++) {
        k = 0;
        for (j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1)
            k = (k << 1) ^ (((k ^ j) & 0x80000000) ? 0x04c11db7 : 0);
        m_CRCTable[i] = k;
    }
}