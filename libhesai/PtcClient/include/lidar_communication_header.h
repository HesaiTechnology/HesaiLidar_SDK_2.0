/*
 * Copyright (C) 2019 Hesai Tech<http://www.hesaitech.com>
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
 * File:        lidar_communication_header.h
 * Author:      Felix Zou<zouke@hesaitech.com>
 * Description:
 */

#ifndef LIDARCOMMUNICATIONHEADER_H
#define LIDARCOMMUNICATIONHEADER_H

#include <endian.h>

struct LidarCommunicationHeader {
  // [5:4] 00:NOP, 2'b01:read, 2'b10:write
  static const uint8_t kNOP = 0x00;
  static const uint8_t kRead = 0x10;
  static const uint8_t kWrite = 0x20;
  // [3:2] operation width, 0: 1byte, 1: 2bytes, 2: 4bytes
  static const uint8_t kOperationByte1 = 0x00;
  static const uint8_t kOperationByte2 = 0x04;
  static const uint8_t kOperationByte4 = 0x08;
  // [1] 0: auto increament, 1: addr fixed
  static const uint8_t kAddrAutoIncrement = 0x00;
  static const uint8_t kAddrFixed = 0x02;
  // [0] 1: send ack flag
  static const uint8_t kAckFlag = 0x01;

  uint8_t m_u8Indicator;
  uint8_t m_u8CmdOption;
  uint32_t m_u32Addr;
  uint16_t m_u16Length;

  void Init(uint8_t u8CmdOption, uint32_t u32Addr, uint16_t u16Len,
            uint8_t u8Indicator = 0xA0) {
    m_u8Indicator = u8Indicator;
    m_u8CmdOption = u8CmdOption;
    m_u32Addr = htobe32(u32Addr);
    m_u16Length = htobe16(u16Len);
  }

  uint32_t GetAddr() const { return be32toh(m_u32Addr); }

  uint16_t GetOpertationLength() const { return be16toh(m_u16Length); }
  uint8_t GetOperationByte() const {
    return (m_u8CmdOption & kOperationByte4)
               ? 4
               : ((m_u8CmdOption & kOperationByte2) ? 2 : 1);
  }

  uint16_t GetDataLength() const {
    return (m_u8CmdOption & kRead)
               ? 0
               : (GetOperationByte() * be16toh(m_u16Length));
  }
  uint16_t GetRspDataLength() const {
    return (m_u8CmdOption & kWrite)
               ? 0
               : (GetOperationByte() * be16toh(m_u16Length));
  }

  uint16_t GetDataOffset() const { return sizeof(LidarCommunicationHeader); }
  uint16_t GetCrcOffset() const {
    return sizeof(LidarCommunicationHeader) + GetDataLength();
  }

  // size: header + data + crc32
  uint16_t GetPacketSize() const { return GetCrcOffset() + sizeof(uint32_t); }
  uint16_t GetRspPacketSize() const {
    return (m_u8CmdOption & kWrite)
               ? (m_u8CmdOption & kAckFlag
                      ? sizeof(LidarCommunicationHeader) + sizeof(uint32_t) + 1
                      : 0)
               : (sizeof(LidarCommunicationHeader) +
                  GetOperationByte() * be16toh(m_u16Length) + sizeof(uint32_t));
  }

} __attribute__((packed));


#endif
