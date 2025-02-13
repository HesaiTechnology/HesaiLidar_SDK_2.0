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
 * File:   serial_source.h
 * Author: chang xingshuo<changxingshuo@hesaitech.com>
 *
 * Created on Sep 10, 2024, 19:56 PM  
 */

#ifndef PANDARSERIAL_H
#define PANDARSERIAL_H

#include <stdint.h>
#include <string>
#include <vector>
#include <iostream>
#include <string.h>
#include "source.h"
#include "blocking_ring.h"
#ifdef _MSC_VER
#include <Windows.h>
#else
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace hesai
{
namespace lidar
{
          
#define    BOTHER 0010000
#define    HS_NCCS 19
#define    SERIAL_POINT_CLOUD_RECV  0
#define    SERIAL_COMMAND_RECV      1
#define    SERIAL_CLEAR_RECV_BUF    2

typedef std::vector<uint8_t> u8Array_t;
#ifndef _MSC_VER
struct termios2 {
  tcflag_t c_iflag;		/* input mode flags */
  tcflag_t c_oflag;		/* output mode flags */
  tcflag_t c_cflag;		/* control mode flags */
  tcflag_t c_lflag;		/* local mode flags */
  cc_t c_line;			/* line discipline */
  cc_t c_cc[HS_NCCS];		/* control characters */
  speed_t c_ispeed;		/* input speed */
  speed_t c_ospeed;		/* output speed */
};
#endif
class SerialSource : public Source {
public:
  ~SerialSource();
  SerialSource(const std::string dev, int baudrate, int point_cloud_baudrate = 3125000);
  /* Disable copy constructor */
  SerialSource(const SerialSource &orig) = delete;

  /**
   * @param dev         path to serial device
   * @param baudrate    number of baud per second
   * @return 0 on success
   */
  int Open(const char *dev, int baudrate);
  virtual bool Open();
  virtual bool IsOpened();
  virtual void Close();
  virtual int Receive(UdpPacket& udpPacket, uint16_t u16Len, int flags = 0,
                      int timeout = 1000);
  virtual int Send(uint8_t* u8Buf, uint16_t u16Len, int flags = 0);
  virtual void SetSocketBufferSize(uint32_t u32BufSize) { return; };
  /**
   * @Desc   flush the COM
   * @return 0 on success, otherwise failure
   */
  int Flush(void);
  int FlushInput(void);
  int FlushOutput(void);
  /**
   * @Desc Check if the FD is valid.
   * @return true for valid
   */
  virtual void SetReceiveStype(int type);

private:
  void ReceivedThread();
  bool running;
  std::thread* runningRecvThreadPtr;
  BlockingRing<UdpPacket, 10> cmdAckRecvBuf;
  BlockingRing<UdpPacket, kPacketBufferSize> pointCloudRecvBuf;
  static const uint32_t kOneRecvLength = 80;
  static const uint32_t kDataMaxLength = 1024 * 80; // is greater than the u16Len parameter of the function Receive
#ifdef _MSC_VER
  HANDLE m_iFd;
#else
  int32_t m_iFd;
#endif
  std::string dev_;
  int baudrate_;
  int point_cloud_baudrate_;
  uint8_t* serialData;
  uint8_t* serialData_backup;
  uint32_t dataIndex;
  uint32_t dataLength;
  int receiveStype = SERIAL_POINT_CLOUD_RECV;

private:
  int WaitRead(int32_t timeout);
};
}
}
#endif /* PANDARSERIAL_H */

