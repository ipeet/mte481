#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Joy.h>

#include "wheelchair_ros/Sonar.h"
#include "wheelchair_ros/controller.hpp"

using namespace std;
using geometry_msgs::Twist;
using nav_msgs::OccupancyGrid;
using wheelchair_ros::Sonar;
using sensor_msgs::Joy;

static auto_ptr<Controller> controller;

class JoyRepeater {
private:
  Joy::ConstPtr m_lastJoy;
  bool m_haveJoy;

public:
  JoyRepeater() : m_haveJoy(false) {}

  void handle(const Joy::ConstPtr &msg) {
    m_lastJoy = msg;
    m_haveJoy = true;
    controller->handleAuxJs(msg);
  }

  void spinOnce() {
    if (m_haveJoy) {
      controller->handleAuxJs(m_lastJoy);
    }
  }
};
static JoyRepeater repeater;

void wheelchairJsCallback(const Twist::ConstPtr &msg) {
  controller->handleJs(msg);
}

void auxJsCallback(const Joy::ConstPtr &msg) {
  repeater.handle(msg);
}

void mapCallback(const OccupancyGrid::ConstPtr &msg) {
  controller->handleMap(msg);
}

void sonarCallback(const Sonar::ConstPtr &msg) {
  controller->handleSonar(msg);
}

int main (int argc, char *argv[]) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle node;

  controller = auto_ptr<Controller> (new Controller(node));

  ros::Subscriber wheelchairIn = node.subscribe<Twist> (
      "wheel_js_in", 1, wheelchairJsCallback);
  ros::Subscriber wheelchairAux = node.subscribe<Joy> (
      "wheel_js_aux", 1, auxJsCallback);
  ros::Subscriber map = node.subscribe<OccupancyGrid> (
      "map2d", 1, mapCallback);
  ros::Subscriber sonar = node.subscribe<Sonar> (
      "sonar", 1, sonarCallback);

  ros::Rate loopRate (50);
  while (ros::ok()) {
    ros::spinOnce();
    repeater.spinOnce();
    loopRate.sleep();
  }
}

