/* Class responsible for accepting control inputs, projecting them forward,
 * and deciding if modification is required. */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class Controller {
private:
  ros::Publisher m_pathPub;

public:
  Controller(ros::NodeHandle &nh);

  void handleJs(const geometry_msgs::Twist::ConstPtr &msg);
  void handleAuxJs(const geometry_msgs::Twist::ConstPtr &msg);

protected:
  nav_msgs::Path::Ptr predictPath(const geometry_msgs::Twist::ConstPtr &input);

  struct State {
    geometry_msgs::PoseStamped::Ptr pose;
  };

  State predict(
      const State &prev, 
      const geometry_msgs::Twist::ConstPtr& input, 
      double step);
};

#endif //CONTROLLER_HPP_
