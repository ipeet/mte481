/* Contains a number of configuration constants used throughout the system */
#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <vector>
#include <cmath>
#include "wheelchair_ros/geometry.hpp"
#include "wheelchair_ros/occupancy.hpp"

namespace config {

/* Map constants.  All in meters. */
static const double RESOLUTION = 0.10; 
static const double WIDTH = 10.0;  
static const double DEPTH = 10.0;
static const double HEIGHT = 1.0; 
static const double ORIG_X = -5.0;
static const double ORIG_Y = -1.0;
static const double ORIG_Z = -2.0;

/* Wheelchair bounding polygon.  (Must be convex).  Kinect is at origin. */
inline Polygon getWheelchairBounds() {
  Polygon ret;
  ret.push(Point3D(-0.5, -0.3, 0));
  ret.push(Point3D(0.2, -0.3, 0));
  ret.push(Point3D(0.2, 0.3, 0));
  ret.push(Point3D(-0.5, 0.3, 0));
  return ret;
}

inline std::vector<Occupancy::SonarPose> getSonarPoses() {
  std::vector<Occupancy::SonarPose> ret;
  ret.push_back(Occupancy::SonarPose(0,    0.3,  0.75*M_PI));
  ret.push_back(Occupancy::SonarPose(-0.3, 0.3,  0.25*M_PI));
  ret.push_back(Occupancy::SonarPose(-0.3, -0.3, -0.75*M_PI));
  ret.push_back(Occupancy::SonarPose(0,    -0.3,  -0.25*M_PI));
  return ret;
}

}; // namespace config


#endif // CONFIG_HPP_

