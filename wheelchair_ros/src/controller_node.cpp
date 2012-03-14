#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include "wheelchair_ros/controller.hpp"

using namespace std;
using geometry_msgs::Twist;
using nav_msgs::OccupancyGrid;

static auto_ptr<Controller> controller;

void wheelchairJsCallback(const Twist::ConstPtr &msg) {
  controller->handleJs(msg);
}

void auxJsCallback(const Twist::ConstPtr &msg) {
  controller->handleAuxJs(msg);
}

void mapCallback(const OccupancyGrid::ConstPtr &msg) {
  controller->handleMap(msg);
}

int main (int argc, char *argv[]) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle node;

  controller = auto_ptr<Controller> (new Controller(node));

  ros::Subscriber wheelchairIn = node.subscribe<Twist> (
      "wheel_js_in", 1, wheelchairJsCallback);
  ros::Subscriber wheelchairAux = node.subscribe<Twist> (
      "wheel_js_aux", 1, auxJsCallback);
  ros::Subscriber map = node.subscribe<OccupancyGrid> (
      "map2d", 1, mapCallback);

  ros::spin();
}

