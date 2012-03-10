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
#include <geometry_msgs/Twist.h>

#include "protocol.h"
#include "wheelchair_ros/protocol_handlers.h"
#include "wheelchair_ros/InputData.h"
#include "wheelchair_ros/Sonar.h"

using namespace std;
using namespace ros;
using namespace wheelchair_ros;

SonarHandler::SonarHandler(NodeHandle &node) :
  m_pub(node.advertise<Sonar>("sonar", 1))
{ }

void SonarHandler::handle(const SerialMessage &msg) const {
  Sonar::Ptr out (new Sonar());
  for (int i=0; i<4; ++i) {
    out->ranges.push_back(2.0*msg.sonar.msmts[i] * 0.0254);
  }
  m_pub.publish(out);
}

JoystickRequestHandler::JoystickRequestHandler(NodeHandle &node) :
  m_pub(node.advertise<geometery_msgs::Twist>("wheelchair_js", 1))
{ }

void JoystickRequestHandler::handle(const SerialMessage &msg) const {
  geometry_msgs::Twist::Ptr out (new geometry_msgs::Twist());
  out.linear.x = double(127 - int(msg.jsReq.lateral)) / 127.0;
  out.linear.y = double(127 - int(msg.jsReq.forward)) / 127.0;
  m_pub.publish(out);
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

