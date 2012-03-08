/* Provides a class which takes care of occupancy map generation */
#ifndef OCCUPANCY_HPP_
#define OCCUPANCY_HPP_

#include <utility>
#include <vector>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/Occupancy3D.h"
#include "wheelchair_ros/Sonar.h"

class Occupancy {
private:
  ros::Publisher m_pub2d;
  std::vector<int> m_kinect2d;
  std::vector<int> m_sonar2d;

  ros::Publisher m_pub3d;
  std::vector<int> m_kinect3d;
  std::vector<int> m_sonar3d;

  double m_resolution;
  double m_width;
  double m_depth;
  double m_height;
  double m_orig_x;
  double m_orig_y;
  double m_orig_z;

  double m_ranges[4];
  bool m_haveSonar;

  class SonarPose {
  private:
    double m_x; // displacment from origin [m]
    double m_y;
    double m_dir;  // orientation, in radians
  public:
    SonarPose(double x, double y, double dir);
    std::pair<double, double> inOccupancy
      (const Occupancy &occ, double rad, double ang);
  };

  std::vector<SonarPose> m_sonars;

public:
  Occupancy(ros::NodeHandle node, 
      double w,  // width [m]
      double d,  // depth [m]
      double h,  // heigh [m]
      double ox, // origin x pos [m]
      double oy, // origin y pos [m]
      double oz, // origin z pos [m]
      double res // grid resolution [m]
  ); 

  void handlePointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg); 
  void handleSonar(const wheelchair_ros::Sonar::ConstPtr &msg);

  void publish();
};

#endif // OCCUPANCY_HPP_

