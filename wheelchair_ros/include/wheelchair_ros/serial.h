/******************************************************************************
 * serial.h
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

#ifndef SERIAL_H_
#define SERIAL_H_

#include <cstdlib>
#include <termios.h>

class Serial {
public:
  class Exception {
  public:
    Exception(const char* m): msg(m) 
      {}
    const char* msg;
  };

private:
  const char* m_serialDev;
  // The original termios state for the TTY
  struct termios m_initialTcAttr;

  /* Functions managing the dongle's file descriptor. */
  void openSerial();
  void closeSerial();

protected:
  // The POSIX file descriptor for the dongle's serial line
  int m_serialFd;

public:
  Serial(const char* serialDev);
  virtual ~Serial();

  void setBlocking(bool blocking);
  ssize_t send(const void* buf, size_t count) throw(Serial::Exception*);
  ssize_t receive(void* buf, size_t count) throw(Serial::Exception*);
};


#endif //SERIAL_H_

