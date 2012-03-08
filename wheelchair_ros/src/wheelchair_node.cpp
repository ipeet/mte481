/******************************************************************************
 * wheelchair_node.cpp
 *
 * ROS node which communicates with the wheelchair controller.
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

#include "ros/ros.h"
#include "protocol.h"
#include "wheelchair_ros/protocol_handlers.h"
#include "wheelchair_ros/protocol_dispatcher.h"
#include "wheelchair_ros/InputData.h"

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "wheelchair_node");
  ros::NodeHandle node;

  const char* devName = "/dev/ttyUSB0";
  try {
    SerialDispatcher::createInstance(devName);
  } catch (Serial::Exception *ex) {
    cerr << "Failed to open " << devName << ": " << ex->msg << endl;
    exit(1);
  }

  SonarHandler sonar (node);
  SerialDispatcher::instance()->setHandler(SONAR_MSG, &sonar);
  JoystickRequestHandler js (node);
  SerialDispatcher::instance()->setHandler(JS_REQ, &js);

  ros::Rate loop_rate(50);
  while(ros::ok()) {
    SerialDispatcher::instance()->pump();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

