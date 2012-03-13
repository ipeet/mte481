#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "wheelchair_ros/controller.hpp"

using namespace std;
using nav_msgs::Path;
using geometry_msgs::Twist;
using geometry_msgs::PoseStamped;

Controller::Controller(ros::NodeHandle &nh) :
  m_pathPub(nh.advertise<nav_msgs::Path>("predicted_path", 1))
{}

void Controller::handleJs(const Twist::ConstPtr &msg) {
  cerr << "Contoller JS: " << msg->linear.x << ", " << msg->linear.y << endl; 
}

void Controller::handleAuxJs(const Twist::ConstPtr &msg) {
  cerr << "Contoller Aux JS: " << msg->linear.x << ", " << msg->linear.y << endl; 
}

Path::Ptr Controller::predictPath(const Twist::ConstPtr &input) {
  State curState;
  curState.pose = PoseStamped::Ptr(new PoseStamped);
  curState.pose->pose.position.x = 0;
  curState.pose->pose.position.y = 0;
  curState.pose->pose.position.z = 0;
  curState.pose->pose.orientation.x = 0;
  curState.pose->pose.orientation.y = 0;
  curState.pose->pose.orientation.z = 1;
  curState.pose->pose.orientation.w = 0.5*M_PI;

  Path::Ptr ret (new Path);
  ret->poses.push_back(*(curState.pose));
  for (int i=0; i<10; ++i) {
    curState = predict(curState, input, 0.2);
    ret->poses.push_back(*(curState.pose));
  }
  return ret;
}

Controller::State Controller::predict(
    const Controller::State &prev, const Twist::ConstPtr &input, double step) 
{
  State ret;
  ret.pose = PoseStamped::Ptr(new PoseStamped);

  // Initialize values from motion-in-plane assumption:
  ret.pose->pose.position.z = 0;
  ret.pose->pose.orientation.x = 0;
  ret.pose->pose.orientation.y = 0;
  ret.pose->pose.orientation.z = 1;

  // Compute current velocities:
  double angular = input->linear.x;
  double forward = input->linear.y;
  double heading = prev.pose->pose.orientation.w; // convenient

  // Forward difference computation of next state:
  ret.pose->pose.orientation.w = heading + step * angular;
  ret.pose->pose.position.x = 
    prev.pose->pose.position.x + forward*cos(heading);
  ret.pose->pose.position.y =
    prev.pose->pose.position.y + forward*sin(heading);

  return ret;
}

