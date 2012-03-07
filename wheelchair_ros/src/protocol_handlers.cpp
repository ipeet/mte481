/******************************************************************************
 * protocol_handlers.cpp
 *
 * Defines classes which are responsible for asynchronous handling of various
 * serial messages.  Typically, they will package up the data and publish to
 * a ros topic.
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

#include "protocol.h"
#include "wheelchair_ros/protocol_handlers.h"
#include "wheelchair_ros/InputData.h"

using namespace std;
using namespace ros;
using namespace wheelchair_ros;

void SonarHandler::handle(const SerialMessage &msg) const {
  cerr << "Sonar:";
  for (int i=0; i<4; ++i) {
    cerr << " " << unsigned(msg.sonar.msmts[i]);
  }
  cerr << endl;
}

void JoystickRequestHandler::handle(const SerialMessage &msg) const {
  cerr << "Joystick: ";
  cerr << 127 - int(msg.jsReq.forward) << " ";
  cerr << 127 - int(msg.jsReq.lateral) << endl;
}

DigitalDataHandler::DigitalDataHandler(NodeHandle &node) :
  m_pub(node.advertise<InputData>("digital_data", 100))
{ }

void DigitalDataHandler::handle(const SerialMessage &msg) const {
  InputData d;
  d.input = msg.digitalData.input;
  d.value = msg.digitalData.value;
  m_pub.publish(d);
}

AdcDataHandler::AdcDataHandler(NodeHandle &node) :
  m_pub(node.advertise<InputData>("analog_data", 100))
{ }

void AdcDataHandler::handle(const SerialMessage &msg) const {
  InputData d;
  d.input = msg.adcData.input;
  d.value = msg.adcData.value;
  m_pub.publish(d);
}

