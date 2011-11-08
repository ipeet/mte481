/******************************************************************************
 * can_node.cpp
 *
 * ROS node which communicates with the wheelchair motor controllers over
 * CAN.
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
#include <cstdlib>

#include "wheelchair_ros/can.h"
#include "ros/ros.h"

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "can_node");
  ros::NodeHandle node;

  try {
    Can can ("/dev/ttyUSB0");
  } catch (Can::CanException * e) {
    cerr << e->msg << endl;
    exit(1);
  }

  while (ros::ok());

  return 0;
}

