/* Class responsible for accepting control inputs, projecting them forward,
 * and deciding if modification is required. */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Joy.h>
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

  enum InputStates {
    MAIN_NEUTRAL,
    MAIN_ENGAGED,
    AUX_ENGAGED
  };
  InputStates m_inputState;

public:
  Controller(ros::NodeHandle &nh);

  void handleJs(const geometry_msgs::Twist::ConstPtr &msg);
  void handleAuxJs(const sensor_msgs::Joy::ConstPtr &msg);
  void handleMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void handleSonar(const wheelchair_ros::Sonar::ConstPtr &msg);

protected:
  void handleInput(double lateral, double forward);
  wheelchair_ros::PredictedPath::Ptr predictPath(
      const geometry_msgs::Twist::ConstPtr &input);
  geometry_msgs::PoseStamped::Ptr predict(
      geometry_msgs::PoseStamped::Ptr prev, 
      const geometry_msgs::Twist::ConstPtr& input, 
      double step);

  bool collides(double x, double y, double w);
};

#endif //CONTROLLER_HPP_
