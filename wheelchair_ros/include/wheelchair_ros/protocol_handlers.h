/******************************************************************************
 * protocol_handlers.h
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

#ifndef PROTOCOL_HANDLERS_
#define PROTOCOL_HANDLERS_

#include <ros/ros.h>

struct SerialMessage;

/* Abstract base for handler classes */
class SerialHandler {
public:
  virtual ~SerialHandler()  {};
  virtual void handle(const SerialMessage &msg) const = 0;
};

/* Receives sonar data */
class SonarHandler : public SerialHandler {
private:
  ros::Publisher m_pub;

public:
  SonarHandler(ros::NodeHandle &node);
  virtual void handle(const SerialMessage &msg) const;
};

/* Receives joystick requests */
class JoystickRequestHandler : public SerialHandler {
private:
  ros::Publisher m_pub;

public:
  JoystickRequestHandler(ros::NodeHandle &node);
  virtual void handle(const SerialMessage &msg) const;
};

/* Publishes digital read data */
class DigitalDataHandler : public SerialHandler {
public:
  DigitalDataHandler(ros::NodeHandle &node);
  virtual void handle(const SerialMessage &msg) const;

private:
  ros::Publisher m_pub;
};

/* Publishes adc read data */
class AdcDataHandler : public SerialHandler {
public:
  AdcDataHandler(ros::NodeHandle &node);
  virtual void handle(const SerialMessage &msg) const;

private:
  ros::Publisher m_pub;
};

#endif //PROTOCOL_HANDLERS_

