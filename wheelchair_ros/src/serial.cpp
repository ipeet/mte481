/******************************************************************************
 * serial.cpp
 * Copyright 2011 Iain Peet
 *
 * Provides basic termios serial communication logic.
 ******************************************************************************
 * This program is distributed under the of the GNU Lesser Public License. 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "wheelchair_ros/serial.h"

Serial::Serial(const char* serialDev) : m_serialDev(serialDev), m_serialFd(-1)
{
  openSerial();
}

Serial::~Serial()
{
  closeSerial();
}

void Serial::openSerial()
{
  if (m_serialFd != -1) return; //already open.

  /* Open the serial dev */
  m_serialFd = open(m_serialDev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (m_serialFd < 0) {
    throw new SerialException(strerror(errno));
  }
  if (!isatty(m_serialFd)) {
    closeSerial();
    throw new SerialException("Device is not a TTY.");
  }

  /* Set up serial config goop */
  struct termios serialTcAttr;
  if (tcgetattr(m_serialFd, &serialTcAttr)) {
    closeSerial();
    throw new SerialException(strerror(errno));
  }
  m_initialTcAttr = serialTcAttr;
  serialTcAttr.c_iflag = 0;
  serialTcAttr.c_oflag = 0;
  serialTcAttr.c_cflag = CREAD | CLOCAL | CS8;
  serialTcAttr.c_lflag = 0;
  cfsetispeed(&serialTcAttr, B115200);
  cfsetospeed(&serialTcAttr, B115200);
  if (tcsetattr(m_serialFd, TCSANOW, &serialTcAttr)) {
    closeSerial();
    throw new SerialException(strerror(errno));
  } 
}

void Serial::closeSerial()
{
  // NB: called by destructor, shouldn't throw exceptions.
  if (m_serialFd == -1) return; // already closed.

  if (tcsetattr(m_serialFd, TCSANOW, &m_initialTcAttr)) {
    perror("tcsetattr");
  }
  close(m_serialFd);
  m_serialFd = -1;
}

