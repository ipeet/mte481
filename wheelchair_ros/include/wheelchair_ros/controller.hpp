/* Class responsible for accepting control inputs, projecting them forward,
 * and deciding if modification is required. */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/PredictedPath.h"
#include "wheelchair_ros/Sonar.h"

class Controller {
private:
  ros::Publisher m_pathPub;
  ros::Publisher m_cmdPub;

  nav_msgs::OccupancyGrid::ConstPtr m_map;
  bool m_haveMap;

  wheelchair_ros::Sonar::ConstPtr m_sonar;
  bool m_haveSonar;

public:
  Controller(ros::NodeHandle &nh);

  void handleJs(const geometry_msgs::Twist::ConstPtr &msg);
  void handleAuxJs(const geometry_msgs::Twist::ConstPtr &msg);
  void handleMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void handleSonar(const wheelchair_ros::Sonar::ConstPtr &msg);

protected:
  wheelchair_ros::PredictedPath::Ptr 
    predictPath(const geometry_msgs::Twist::ConstPtr &input);

  struct State {
    geometry_msgs::PoseStamped::Ptr pose;
  };

  State predict(
      const State &prev, 
      const geometry_msgs::Twist::ConstPtr& input, 
      double step);

  bool collides(double x, double y, double w);
};

#endif //CONTROLLER_HPP_
