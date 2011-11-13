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
#include "sensor_msgs/Range.h"
#include "ros/ros.h"

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sonar_node");
  ros::NodeHandle node;

  const char* dev = "/dev/ttyACM0";
  uint16_t sonarChans = 0x17;
  ros::Rate rate (20);

  /* Create a Range publisher for each channel */
  vector<ros::Publisher> publishers;
  int chan = 0;
  uint16_t imask = sonarChans;
  const int bufSize = 32;
  char buf[bufSize];
  while (imask) {
    if (imask & 0x1) {
      snprintf(buf, bufSize, "range%d", chan);
      publishers.push_back(node.advertise<sensor_msgs::Range>(buf, 100));
    }
    ++chan;
    imask = imask >> 1;
  }

  /* Open ubw and start reading data */
  try {
    Sonar sonar (dev, sonarChans);
    sonar.clear();
    cout << "Opened serial device: " << dev << endl; 
 
    while (ros::ok()) {
      vector<Sonar::Reading> readings = sonar.readSonar();
      for (unsigned i=0; i<readings.size(); ++i) {
        sensor_msgs::Range range;
        range.radiation_type = sensor_msgs::Range::ULTRASOUND;
        range.field_of_view = 0.3;
        range.min_range = 0.1524;
        range.max_range = 6.477;
        range.range = readings[i].value * 0.0254 / 4.0;
        publishers[i].publish(range);
      }
      rate.sleep();
    }
  } catch (Serial::Exception * e) {
    cerr << "Serial::Exception:" << e->msg << endl;
  }

  return 1;
}

