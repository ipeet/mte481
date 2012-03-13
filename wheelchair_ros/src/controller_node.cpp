#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "wheelchair_ros/controller.hpp"

using geometry_msgs::Twist;

static Controller *controller;

void wheelchairJsCallback(const Twist::ConstPtr &msg) {
}

void auxJsCallback(const Twist::ConstPtr &msg) {
}

int main (int argc, char *argv[]) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle node;

  controller = new Controller(node);

  ros::Subscriber wheelchairIn = node.subscribe<Twist> (
      "wheel_js_in", 1, wheelchairJsCallback);
  ros::Subscriber wheelchairAux = node.subscribe<Twist> (
      "wheel_js_aux", 1, auxJsCallback);

  ros::spin();
}

