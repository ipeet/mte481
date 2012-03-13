#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "wheelchair_ros/controller.hpp"

using namespace std;
using geometry_msgs::Twist;

static auto_ptr<Controller> controller;

void wheelchairJsCallback(const Twist::ConstPtr &msg) {
  controller->handleJs(msg);
}

void auxJsCallback(const Twist::ConstPtr &msg) {
  controller->handleAuxJs(msg);
}

int main (int argc, char *argv[]) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle node;

  controller = auto_ptr<Controller> (new Controller(node));

  ros::Subscriber wheelchairIn = node.subscribe<Twist> (
      "wheel_js_in", 1, wheelchairJsCallback);
  ros::Subscriber wheelchairAux = node.subscribe<Twist> (
      "wheel_js_aux", 1, auxJsCallback);

  ros::spin();
}

