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
 * File:   source.h
 * Author: Felix Zou<zouke@hesaitech.com>
 *
 * Created on Sep 5, 2019, 10:46 AM
 */

#ifndef STREAMER_H
#define STREAMER_H

#include <stdint.h>
#include <string>
#include <lidar_types.h>
namespace hesai
{
namespace lidar
{
class Source {
 public:
  Source();
  ~Source();
  virtual bool Open() = 0;
  virtual void Close();
  virtual bool IsOpened() = 0;
  virtual int Send(uint8_t* u8Buf, uint16_t u16Len, int flags = 0) = 0;
  virtual int Receive(UdpPacket& udpPacket, uint16_t u16Len, int flags = 0,
                      int timeout = 1000) = 0; 
};
}  // namespace lidar
}  // namespace hesai
#endif /* STREAMER_H */
