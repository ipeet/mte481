#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/PredictedPath.h"
#include "wheelchair_ros/Sonar.h"
#include "wheelchair_ros/controller.hpp"
#include "wheelchair_ros/config.hpp"
#include "wheelchair_ros/geometry.hpp"


using namespace std;
using geometry_msgs::Twist;
using geometry_msgs::PoseStamped;
using nav_msgs::OccupancyGrid;
using sensor_msgs::Joy;
using wheelchair_ros::PredictedPath;
using wheelchair_ros::Sonar;

Controller::Controller(ros::NodeHandle &nh) :
  m_node(nh),
  m_pathPub(nh.advertise<PredictedPath>("predicted_path", 1)),
  m_cmdPub(nh.advertise<Twist>("wheel_js_out", 1)),
  m_haveMap(false),
  m_haveSonar(false),
  m_inputState(MAIN_NEUTRAL)
{}

void Controller::handleJs(const Twist::ConstPtr &msg) {
  // Is this a netural position?
  bool mainIsNeutral = true; 
  if (abs(msg->linear.x - config::LAT_OFFSET) > 0.2) {
    mainIsNeutral = false;
  }
  if (abs(msg->linear.y - config::FWD_OFFSET) > 0.2) {
    mainIsNeutral = false;
  }

  if (! mainIsNeutral) {
    m_inputState = MAIN_ENGAGED;
    handleInput(msg->linear.x, msg->linear.y);
  } else if (m_inputState != AUX_ENGAGED) {
    m_inputState = MAIN_NEUTRAL;
    handleInput(msg->linear.x, msg->linear.y);
  }
}

void Controller::handleAuxJs(const Joy::ConstPtr &msg) {
  if (m_inputState == MAIN_ENGAGED) {
    return;
  }
  bool auxIsNeutral = true;
  if (abs(msg->axes[0]) > 0.05) {
    auxIsNeutral = false;
  }
  if (abs(msg->axes[1]) > 0.05) {
    auxIsNeutral = false;
  }

  if (auxIsNeutral) {
    m_inputState = MAIN_NEUTRAL;
    return;
  } else {
    m_inputState = AUX_ENGAGED;
  }

  // Wheelchair js is neutral, accept auxiliary joystick command.
  handleInput(0.3*(-msg->axes[0] - config::LAT_OFFSET), 
      0.3*(msg->axes[1] + config::FWD_OFFSET));
}

void Controller::handleInput(double lateral, double forward) {
  Twist::Ptr cmd (new Twist);
  cmd->linear.x = lateral;
  cmd->linear.y = forward;

  /* Check rotation against sonars.  We will modify only
   * the rotation rate.  Do this before predicting the path, so that
   * path prediction accounts for the modified angular rate. */
  if (m_haveSonar) {
    double minRange = m_sonar->ranges[0];
    for (int i=1; i<4; ++i) {
      if (m_sonar->ranges[i] < minRange) {
        minRange = m_sonar->ranges[i];
      }
    }
    if (minRange < 0.6) {
      double scale = minRange / 0.6;
      cmd->linear.x *= scale;
    }
  }

  PredictedPath::Ptr path = predictPath(cmd);
  m_pathPub.publish(path);

  /* Check path against occupancy grid */
  double tCol = path->timestep * ((*path).poses.size() -1);
  if ( tCol < config::TIMESTEP ) {
    cmd->linear.x = config::LAT_OFFSET;
    cmd->linear.y = config::FWD_OFFSET;
  } else if (tCol < config::SOFTSTOP_BEGIN) {
    double scale = tCol / config::SOFTSTOP_BEGIN;
    cmd->linear.x *= scale;
    cmd->linear.y *= scale;
  } 

  /* Check if driving is actually enabled */
  if (! driveEnabled()) {
    cmd->linear.x = config::LAT_OFFSET;
    cmd->linear.y = config::FWD_OFFSET;
  }

  m_cmdPub.publish(cmd);
}

void Controller::handleMap(const OccupancyGrid::ConstPtr &msg) {
  m_map = msg;
  m_haveMap = true;
}

void Controller::handleSonar(const Sonar::ConstPtr &msg) {
  m_sonar = msg;
  m_haveSonar = true;
}

PredictedPath::Ptr Controller::predictPath(const Twist::ConstPtr &input) {
  PoseStamped::Ptr curState (new PoseStamped);
  curState->pose.position.x = 0;
  curState->pose.position.y = 0;
  curState->pose.position.z = 0;
  curState->pose.orientation.x = 0;
  curState->pose.orientation.y = 0;
  curState->pose.orientation.z = 1;
  curState->pose.orientation.w = 0.5*M_PI;

  PredictedPath::Ptr ret (new PredictedPath);
  ret->poses.push_back(*(curState));
  ret->poseCollides.push_back(false);
  ret->timestep = config::TIMESTEP;
  for (double t=0; t <= config::SIM_LENGTH; t += config::TIMESTEP) {
    curState = predict(curState, input, config::TIMESTEP);
    ret->poses.push_back(*curState);
    double curX = curState->pose.position.x;
    double curY = curState->pose.position.y;
    bool col = collides(curX, curY, curState->pose.orientation.w);
    ret->poseCollides.push_back(col);
    if (col) break;
  }
  return ret;
}

PoseStamped::Ptr Controller::predict(
    PoseStamped::Ptr prev, const Twist::ConstPtr &input, double step) 
{
  PoseStamped::Ptr ret (new PoseStamped);

  // Initialize values from motion-in-plane assumption:
  ret->pose.position.z = 0;
  ret->pose.orientation.x = 0;
  ret->pose.orientation.y = 0;
  ret->pose.orientation.z = 1;

  // Compute current velocities:
  double angular = config::K_ROT * (-input->linear.x + config::LAT_OFFSET);
  double forward = config::K_FWD * (input->linear.y - config::FWD_OFFSET); 
  double heading = prev->pose.orientation.w; // convenient

  // Forward difference computation of next state:
  ret->pose.orientation.w = heading + step * angular;
  ret->pose.position.x = 
    prev->pose.position.x + forward*cos(heading);
  ret->pose.position.y =
    prev->pose.position.y + forward*sin(heading);

  return ret;
}

bool Controller::collides(double x, double y, double w) {
  if (!m_haveMap) return false;
  Polygon wheel (config::getWheelchairBounds());
  double res = m_map->info.resolution;
  double orig_x = m_map->info.origin.position.x;
  double orig_y = m_map->info.origin.position.y;

  wheel = Matrix4x4::rotation(w, Vector3D(0, 0, 1)) * wheel;
  wheel = Matrix4x4::translation(Vector3D(
        x-orig_x, y-orig_y-config::KINECT_OFFSET, 0)) * wheel;
  wheel = Matrix4x4::scale(1.0/res, 1.0/res, 1.0) * wheel;
  
  for (unsigned i=0; i < m_map->info.width; ++i) {
    for (unsigned j=0; j < m_map->info.height; ++j) {
      if (m_map->data[i + j*(m_map->info.width)]) {
        if (wheel.contains(Point3D(i, j, 0))) {
          return true;
        }
      }
    }
  }

  return false;
}

bool Controller::driveEnabled() {
  bool ret;
  m_node.param<bool>("drive_enabled", ret, true);
  return ret;
}

