/******************************************************************************
 * sonar_node.cpp
 *
 * ROS node which communicates with the ultrasonic rangefinders.
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
#include <vector>
#include <cstdlib>

#include <unistd.h>

#include "wheelchair_ros/sonar.h"
#include "ros/ros.h"

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sonar_node");
  ros::NodeHandle node;

  const char* dev = "/dev/ttyACM0";

  try {
    Sonar sonar (dev, 0x17);
    sonar.clear();
    cout << "Opened serial device: " << dev << endl; 
  
    while (ros::ok()) {
      vector<Sonar::Reading> readings = sonar.readSonar();
      for (int i=0; i<readings.size(); ++i) {
        cout << readings[i].channel << ": " << readings[i].value << endl;
      }
      sleep(1);
    }
  } catch (Serial::Exception * e) {
    cerr << e->msg << endl;
  }

  return 1;
}

