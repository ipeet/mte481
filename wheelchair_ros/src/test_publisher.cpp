/* Ros node which publishes data used for the testing of other nodes */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using geometry_msgs::Twist;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle node;

  ros::Publisher wheelJs (node.advertise<Twist>("wheel_js_in", 1));

  ros::Rate loopRate(5);
  while (ros::ok()) {
    Twist::Ptr js (new Twist);
    js->linear.x = 0.2;
    js->linear.y = 0.5;
    wheelJs.publish(js);
    
    loopRate.sleep();
  }
  return 0;
}
