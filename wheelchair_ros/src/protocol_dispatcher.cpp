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

SerialDispatcher::SerialDispatcher(const char *serialDev) :
  Serial(serialDev),
  m_bufCount(0),
  m_readState(SYNC_1)
{ }

void SerialDispatcher::setHandler(MessageType type, const SerialHandler *handler) {
  m_handlers[type] = handler;
}

void SerialDispatcher::clearHandler(MessageType type) {
  m_handlers.erase(type);
}

void SerialDispatcher::writeMsg(const SerialMessage &msg) throw(Serial::Exception*) {
  setBlocking(true);
  send(reinterpret_cast<const char*>(&msg), msg.length + MSG_HEADER_LENGTH);
}

void SerialDispatcher::readMsg(MessageType type, SerialMessage &message) throw(Serial::Exception*) {
  setBlocking(true);
  char ch;
  while (true) {
    receive(&ch, 1);
    try {
      if (push(ch)) {
        /* Just received a new message */
        if (m_msgBuf[1] == type) {
          // correct type.
          memcpy(&message, m_msgBuf, m_bufCount);
          return;
        } else {
          // wrong type, dispatch normally.
          dispatch();
        }
      }
    } catch (Serial::Exception *e) {
      /* Exception is advisory, carry on */
      cerr << e->msg << endl;
      delete e;
    }
  }
}

void SerialDispatcher::pump() throw(Serial::Exception*) {
  setBlocking(false);
  char ch;
  while (read(m_serialFd, &ch, 1)==1) {
    try {
      if (push(ch)) {
        dispatch();
      }
    } catch (Serial::Exception *e) {
      /* Exception is advisory, carry on */
      cerr << e->msg << endl;
      delete e;
    }
  }

  /* Make sure that we broke out due to empty buffer */
  if (errno != EAGAIN) {
    throw new Serial::Exception(strerror(errno));
  }
}

bool SerialDispatcher::push(uint8_t byte) throw(Serial::Exception*) {
  if (m_readState == COMPLETE) {
    // Time to start on a new message
    m_readState = SYNC_1;
    m_bufCount = 0;
  }

  m_msgBuf[m_bufCount] = byte;
  m_bufCount++;

  switch (m_readState) {
    case SYNC_1:
      m_bufCount = 0;  // sync bytes not part of msg
      if (byte == SYNC_BYTE_1) {
        m_readState = SYNC_2;
      } else {
        throw new Serial::Exception("Expected sync 1");
      }
      break;

    case SYNC_2:
      m_bufCount = 0; // sync bytes not part of msg
      if (byte == SYNC_BYTE_2) {
        m_readState = CHECKSUM;
      } else {
        m_readState = SYNC_1;
        throw new Serial::Exception("Expected sync 2");
      }
      break;

    case CHECKSUM:
      m_readState = TYPE;
      break;

    case TYPE:
      m_readState = LENGTH;
      break;

    case LENGTH:
      m_readState = PAYLOAD;

      // Intentional fall-through to handle 0-length messages
    case PAYLOAD:
      if (m_bufCount >= MSG_HEADER_LENGTH + m_msgBuf[2]) {
        m_readState = COMPLETE;
      }
      break;

    default:
      abort();
  }

  if (m_readState == COMPLETE) {
    // Just completed a message.  Check the checksum.
    uint8_t check = 0;
    for (unsigned i=0; i<m_bufCount; ++i) {
      check ^= m_msgBuf[i];
    }
    if (check == m_msgBuf[0]) {
      return true;
    } else {
      throw new Serial::Exception("Bad checksum");
    }
  }
  return false;
}

void SerialDispatcher::dispatch() {
  MessageType t = static_cast<MessageType>(m_msgBuf[1]);
  if (m_handlers.find(t) != m_handlers.end()) {
    m_handlers[t]->handle(* reinterpret_cast<SerialMessage*>(m_msgBuf));
  }
}

