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
#include <cstdio>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "protocol.h"
#include "wheelchair_ros/protocol_handlers.h"
#include "wheelchair_ros/protocol_dispatcher.h"
#include "wheelchair_ros/InputData.h"

using namespace std;
using geometry_msgs::Twist;

void printSerialMessage(const SerialMessage &msg) {
  const uint8_t *bMsg = reinterpret_cast<const uint8_t*>(&msg);
  for (unsigned i=0; i < msg.length + MSG_HEADER_LENGTH; ++i) {
    fprintf(stderr, "%02x ", int(bMsg[i]));
  }
  fprintf(stderr, "\n");
}

void jsOutCallback(const Twist::ConstPtr &msg) {
  /* Send command to wheelchair */
  struct SerialMessage cmd;
  cmd.type = JS_LIMIT;
  cmd.length = 2;
  cmd.jsReq.forward = 127.5 + msg->linear.y*127;
  cmd.jsReq.lateral = 127.5 + msg->linear.x*127;
  cmd.checksum = pr_checksum(&cmd);
  try {
    SerialDispatcher::instance()->writeMsg(cmd);
  } catch (Serial::Exception *ex) {
    cerr << "Write error: " << ex->msg << endl;
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "wheelchair_node");
  ros::NodeHandle node;

  ros::Rate chair_retry(0.5);
  const char* devName = "/dev/ttyUSB0";
  while (ros::ok()) {
    try {
      SerialDispatcher::createInstance(devName);
      break;
    } catch (Serial::Exception *ex) {
      cerr << "Failed to open " << devName << ": " << ex->msg << endl;
    }
    chair_retry.sleep();
  }

  ros::Subscriber js_out_sub = node.subscribe<Twist> (
      "wheel_js_out", 1, jsOutCallback);

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

