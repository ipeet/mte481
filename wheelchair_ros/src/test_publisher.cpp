/* Ros node which publishes data used for the testing of other nodes */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/config.hpp"

using geometry_msgs::Twist;
using nav_msgs::OccupancyGrid;

void publishMap(ros::Publisher &pub) {
  OccupancyGrid::Ptr map2 (new OccupancyGrid);
  map2->info.resolution = 0.2;
  map2->info.width = 100;
  map2->info.height = 100;
  map2->info.origin.position.x = -10;
  map2->info.origin.position.y = -2;
  map2->data.resize(100*100);
  for (int i=0; i<100; ++i) {
    for (int j=0; j<100; ++j) {
      if (i==20) {
        map2->data[j*100 + i] = 100;
      } else if (i==j) {
        map2->data[j*100 + i] = 100;
      } else if (j==30) {
        map2->data[j*100 + i] = 100;
      } else {
        map2->data[j*100 + i] = 0;
      }
    }
  }
  pub.publish(map2);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle node;

  ros::Publisher wheelJs (node.advertise<Twist>("wheel_js_in", 1));
  ros::Publisher map2 (node.advertise<OccupancyGrid>("map2d", 1));

  ros::Rate loopRate(5);
  while (ros::ok()) {
    Twist::Ptr js (new Twist);
    js->linear.x = -0.5;
    js->linear.y = 2.0;
    wheelJs.publish(js);

    publishMap(map2);
    
    loopRate.sleep();
  }
  return 0;
}
