/* Provides a class which takes care of occupancy map generation */
#ifndef OCCUPANCY_HPP_
#define OCCUPANCY_HPP_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/Occupancy3D.h"

class Occupancy {
private:
  ros::Publisher m_pub2d;
  nav_msgs::OccupancyGrid::Ptr m_map2d; 

  ros::Publisher m_pub3d;
  wheelchair_ros::Occupancy3D::Ptr m_map3d;

  double m_resolution;
  double m_width;
  double m_depth;
  double m_height;
  double m_orig_x;
  double m_orig_y;
  double m_orig_z;

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
};

#endif // OCCUPANCY_HPP_

