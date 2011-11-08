/******************************************************************************
 * can.h
 * Copyright 2011 Iain Peet
 *
 * Provides a class for interfacing with a CAN dongle.
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

#ifndef CAN_H_
#define CAN_H_

#include <termios.h>

class Can {
public:
  class CanException {
  public:
    CanException(const char* m): msg(m) 
      {}
    const char* msg;
  };

private:
  const char* m_canDev;
  // The POSIX file descriptor for the dongle's serial line
  int m_canFd;
  // The original termios state for the TTY
  struct termios m_initialTcAttr;

  /* Functions managing the dongle's file descriptor. */
  void openCan();
  void closeCan();

public:
  Can(const char* canDev);
  ~Can();
};


#endif //CAN_H_

