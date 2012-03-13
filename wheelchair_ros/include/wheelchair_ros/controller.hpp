/* Class responsible for accepting control inputs, projecting them forward,
 * and deciding if modification is required. */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Controller {
private:
  ros::Publisher m_pathPub;

public:
  Controller(ros::NodeHandle &nh);

};

#endif //CONTROLLER_HPP_
