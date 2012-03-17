/******************************************************************************
 * protocol_dispatcher.h
 *
 * Class responsible for reading from and writing to the microcontroller.
 * Writes are synchronous.
 * Reads may be handled asynchronously by a registered SerialHandler, or 
 * a SerialMessage of specified type may be read synchronously.
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

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <errno.h>

#include "wheelchair_ros/protocol_dispatcher.h"
#include "wheelchair_ros/protocol_handlers.h"

using namespace std;
using namespace std::tr1;

shared_ptr<SerialDispatcher> SerialDispatcher::singleton;

void SerialDispatcher::createInstance(const char *serialDev) throw(Serial::Exception*) {
  static bool created = false;
  if (!created) {
    singleton = shared_ptr<SerialDispatcher>(new SerialDispatcher(serialDev));
    created = true;
    return;
  }
  throw new Serial::Exception("Instance already exists");
}

shared_ptr<SerialDispatcher> SerialDispatcher::instance() {
  return singleton;
}

SerialDispatcher::SerialDispatcher(const char *serialDev) :
  Serial(serialDev)
{ }

void SerialDispatcher::setHandler(MessageType type, const SerialHandler *handler) {
  m_handlers[type] = handler;
}

void SerialDispatcher::clearHandler(MessageType type) {
  m_handlers.erase(type);
}

void SerialDispatcher::writeMsg(const SerialMessage &msg) throw(Serial::Exception*) {
  setBlocking(true);
  uint8_t sync[] = {SYNC_BYTE_1, SYNC_BYTE_2};
  send(&sync, 2);
  send(&msg, msg.length + MSG_HEADER_LENGTH);
}

void SerialDispatcher::readMsg(MessageType type, SerialMessage *message) throw(Serial::Exception*) {
  setBlocking(true);
  char ch;
  while (true) {
    receive(&ch, 1);
    ParseStatus stat = pr_push(ch);
    switch (stat) {
      case OK: 
        break;

      case COMPLETE:
        if (pr_getmsg()->type == type) {
          memcpy(message, pr_getmsg(), sizeof(SerialMessage));
          return;
        } else {
          dispatch(*pr_getmsg());
        }
        break;

      case BAD_SYNC:
        cerr << "Synchronization Error" << endl;
        break;

      case BAD_CHECKSUM:
        cerr << "Bad checksum" << endl;
        break; 

      case INVALID:
        throw new Serial::Exception("Invalid parser");
    }
  }
}

void SerialDispatcher::pump() throw(Serial::Exception*) {
  setBlocking(false);
  uint8_t ch;
  while (read(m_serialFd, &ch, 1)==1) {
    ParseStatus stat = pr_push(ch);
    switch (stat) {
      case OK: 
        break;

      case COMPLETE:
        dispatch(*pr_getmsg());
        break;

      case BAD_SYNC:
        cerr << "Synchronization Error" << endl;
        break;

      case BAD_CHECKSUM:
        cerr << "Bad checksum" << endl;
        break;

      case INVALID:
        throw new Serial::Exception("Invalid parser");
    }
  }

  /* Make sure that we broke out due to empty buffer */
  if (errno != EAGAIN) {
    throw new Serial::Exception(strerror(errno));
  }
}

void SerialDispatcher::dispatch(const SerialMessage &msg) {
  MessageType t = static_cast<MessageType>(msg.type);
  if (m_handlers.find(t) != m_handlers.end()) {
    m_handlers[t]->handle(msg);
  } else {
    cerr << "Dropped message.  Type: " << msg.type << endl;
  }
}

