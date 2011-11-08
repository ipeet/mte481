/******************************************************************************
 * can.cpp
 * Copyright 2011 Iain Peet
 *
 * Communicates with the Can dongle.
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

#include "wheelchair_ros/can.h"

Can::Can(const char* canDev) : m_canDev(canDev), m_canFd(-1)
{
  openCan();
}

Can::~Can()
{
  closeCan();
}

void Can::openCan()
{
  if (m_canFd != -1) return; //already open.

  /* Open the serial dev */
  m_canFd = open(m_canDev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (m_canFd < 0) {
    throw new CanException(strerror(errno));
  }
  if (!isatty(m_canFd)) {
    closeCan();
    throw new CanException("Device is not a TTY.");
  }

  /* Set up serial config goop */
  struct termios canTcAttr;
  if (tcgetattr(m_canFd, &canTcAttr)) {
    closeCan();
    throw new CanException(strerror(errno));
  }
  m_initialTcAttr = canTcAttr;
  canTcAttr.c_iflag = 0;
  canTcAttr.c_oflag = 0;
  canTcAttr.c_cflag = CREAD | CLOCAL | CS8;
  canTcAttr.c_lflag = 0;
  cfsetispeed(&canTcAttr, B115200);
  cfsetospeed(&canTcAttr, B115200);
  if (tcsetattr(m_canFd, TCSANOW, &canTcAttr)) {
    closeCan();
    throw new CanException(strerror(errno));
  } 
}

void Can::closeCan()
{
  // NB: called by destructor, shouldn't throw exceptions.
  if (m_canFd == -1) return; // already closed.

  if (tcsetattr(m_canFd, TCSANOW, &m_initialTcAttr)) {
    perror("tcsetattr");
  }
  close(m_canFd);
  m_canFd = -1;
}

