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

/* Distance from wheelchair center of rotation to kinect */
static const double KINECT_OFFSET = 0.2;

/* Wheelchair bounding polygon.  (Must be convex). */
inline Polygon getWheelchairBounds() {
  Polygon ret;
  ret.push(Point3D(-0.5, -0.3, 0));
  ret.push(Point3D(0.5, -0.3, 0));
  ret.push(Point3D(0.5, 0.3, 0));
  ret.push(Point3D(-0.5, 0.3, 0));
  return ret;
}

static const double WHEELCHAIR_LENGTH = 1.0;
static const double WHEELCHAIR_WIDTH = 0.6;

static const double SONAR_FOV = 0.4*M_PI;

inline std::vector<Occupancy::SonarPose> getSonarPoses() {
  std::vector<Occupancy::SonarPose> ret;
  ret.push_back(Occupancy::SonarPose(-0.3,  0.0, -0.8*M_PI));
  ret.push_back(Occupancy::SonarPose(-0.3, -0.3,  0.8*M_PI));
  ret.push_back(Occupancy::SonarPose(0.3,  -0.3,  0.2*M_PI));
  ret.push_back(Occupancy::SonarPose(0.3,   0.0, -0.2*M_PI));
  return ret;
}

/* Motion model parameters */
static const double TIMESTEP = 0.25;
static const double SIM_LENGTH = 10.0;
static const double K_FWD = 0.3;
static const double K_ROT = 0.3;
static const double SOFTSTOP_BEGIN = 5.0;  // (seconds to collision)
static const double LAT_OFFSET = 0.04;  // zero point for wheelchair js.
static const double FWD_OFFSET = 0.055;

}; // namespace config


#endif // CONFIG_HPP_

