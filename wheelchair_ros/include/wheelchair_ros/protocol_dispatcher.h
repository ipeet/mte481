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
#include <tr1/memory>
#include <cstdlib>
#include "wheelchair_ros/serial.h"
#include "protocol.h"

class SerialHandler;

class SerialDispatcher : public Serial {
public:
  static void createInstance(const char *serialDev) throw(Serial::Exception*);
  static std::tr1::shared_ptr<SerialDispatcher> instance();

  void setHandler(MessageType type, const SerialHandler *handler);
  void clearHandler(MessageType type);

  void writeMsg(const SerialMessage &msg) throw(Serial::Exception*);
  void readMsg(MessageType type, SerialMessage *msg) throw(Serial::Exception*);

  /* Reads and dispatches any buffered input data */
  void pump() throw(Serial::Exception*);

protected:
  /* Singleton, because the C parser has static internal state */
  SerialDispatcher(const char *serialDev);

private:
  /* Dispatch the messge currently contained in m_buf */
  void dispatch(const SerialMessage &msg);

  std::map<MessageType, const SerialHandler*> m_handlers;

  static std::tr1::shared_ptr<SerialDispatcher> singleton;
};

#endif //PROTOCOL_DISPATCHER_H_
