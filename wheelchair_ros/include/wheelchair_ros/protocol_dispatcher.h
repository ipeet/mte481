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

#ifndef PROTOCOL_DISPATCHER_H_
#define PROTOCOL_DISPATCHER_H_

#include <map>
#include <memory>
#include <cstdlib>
#include "wheelchair_ros/serial.h"
#include "wheelchair_ros/protocol.h"

class SerialHandler;

class SerialDispatcher : public Serial {
public:
  SerialDispatcher(const char *serialDev);

  void setHandler(MessageType type, const SerialHandler *handler);
  void clearHandler(MessageType type);

  void writeMsg(const SerialMessage &msg) throw(Serial::Exception*);
  void readMsg(MessageType type, SerialMessage &msg) throw(Serial::Exception*);

  /* Reads and dispatches any buffered input data */
  void pump() throw(Serial::Exception*);

private:
  /* Parse a byte.  
   * @return true if m_msgBuf contains a newly-complete message.
   * @throw if a parse error occurs (non-fatal) */
  bool push(uint8_t byte) throw(Serial::Exception*);

  /* Dispatch the messge currently contained in m_buf */
  void dispatch();

  std::map<MessageType, const SerialHandler*> m_handlers;

  uint8_t m_msgBuf[MAX_MSG_SIZE];
  size_t  m_bufCount;
  
  enum ReadStates {
    SYNC_1,
    SYNC_2,
    CHECKSUM,
    TYPE,
    LENGTH,
    PAYLOAD,
    COMPLETE
  };
  ReadStates m_readState;
};

#endif //PROTOCOL_DISPATCHER_H_
