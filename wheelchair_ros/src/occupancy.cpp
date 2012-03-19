#include <cmath>
#include <iostream>

#include "wheelchair_ros/occupancy.hpp"
#include "wheelchair_ros/config.hpp"

using namespace std;
using namespace nav_msgs;
using namespace wheelchair_ros;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
Occupancy::Occupancy(ros::NodeHandle node, 
  double w,  // width [m]
  double d,  // depth [m]
  double h,  // heigh [m]
  double ox, // origin x pos [m]
  double oy, // origin y pos [m]
  double oz,
  double res // grid resolution [m]
) : 
  m_pub2d(node.advertise<OccupancyGrid>("map2d", 1)),
  m_pub3d(node.advertise<Occupancy3D>("map3d", 1)),
  m_resolution(res),
  m_width(w),
  m_depth(d),
  m_height(h),
  m_orig_x(ox),
  m_orig_y(oy),
  m_orig_z(oz),
  m_haveSonar(false),
  m_sonars(config::getSonarPoses())
{
  /* Convenience - grid size */
  int gw = m_width / m_resolution;
  int gd = m_depth / m_resolution;
  int gh = m_height / m_resolution;
  m_kinect2d.resize(gw*gd);
  m_sonar2d.resize(gw*gd);
  m_kinect3d.resize(gw*gd*gh);
  m_sonar3d.resize(gw*gd*gh);
}

void Occupancy::handlePointcloud(const PointCloud::ConstPtr &msg) {
  /* Convenience */
  int w = m_width / m_resolution;
  int d = m_depth / m_resolution;
  int h = m_height / m_resolution;

  m_kinect2d.clear();
  m_kinect2d.resize(w*d);

  m_kinect3d.clear();
  m_kinect3d.resize(w*d*h);

  for (unsigned i=0; i < msg->points.size(); ++i) {
    double x = msg->points.at(i).x;
    double y = -msg->points.at(i).y;
    double z = msg->points.at(i).z;

    /* Bounds check the point */
    if (isnan(x) || isnan(y) || isnan(z)) continue;
    if (x < m_orig_x) continue;
    if (x >= m_orig_x + m_width) continue;
    if (y < m_orig_y) continue;
    if (y >= m_orig_y + m_height) continue;
    if (z < m_orig_z) continue;
    if (z >= m_orig_z + m_depth) continue;
    int xi = (x - m_orig_x) / m_resolution;
    int yi = (y - m_orig_y) / m_resolution;
    int zi = (z - m_orig_z) / m_resolution;
    m_kinect2d[zi*w + xi] = 100;
    m_kinect3d[yi*d*w + zi*w + xi] = 1;
  }
}

void Occupancy::handleSonar(const Sonar::ConstPtr &msg) {
  /* Convenience */
  int w = m_width / m_resolution;
  int d = m_depth / m_resolution;
  int h = m_height / m_resolution;

  m_sonar2d.clear();
  m_sonar2d.resize(w*d);

  m_sonar3d.clear();
  m_sonar3d.resize(w*d*h);

  for (int i=0; i<4; ++i) {
    for (double j=-0.5*config::SONAR_FOV; j < 0.5*config::SONAR_FOV; j+=0.02) {
      pair<double, double> pos = m_sonars[i].inOccupancy(
          *this, msg->ranges[i], j);
      double x = pos.first;
      double z = pos.second;
      int xi = (x - m_orig_x) / m_resolution + 0.5;
      int zi = (z - m_orig_z) / m_resolution + 0.5;
      if (xi < 0) continue;
      if (xi >= m_width) continue;
      if (zi < 0) continue;
      if (zi >= m_depth) continue;
      m_sonar2d[zi*w + xi] = 100;
      m_sonar3d[zi*w + xi] = 1;
    }
    m_ranges[i] = msg->ranges[i];
  }
  m_haveSonar = true;
}

void Occupancy::publish() {
  /* Convenience - grid size */
  int gw = m_width / m_resolution;
  int gd = m_depth / m_resolution;
  int gh = m_height / m_resolution;

  OccupancyGrid::Ptr map2 (new OccupancyGrid);
  map2->info.resolution = m_resolution;
  map2->info.width = gw;
  map2->info.height = gd;
  map2->info.origin.position.x = m_orig_x;
  map2->info.origin.position.y = m_orig_z;
  map2->data.resize(gw*gd);

  for (unsigned i=0; i<m_kinect2d.size(); ++i) {
    map2->data[i] = (m_kinect2d[i] || m_sonar2d[i]) ? 100 : 0;
  }
  
  m_pub2d.publish(map2);

  Occupancy3D::Ptr map3 (new Occupancy3D);
  map3->width = gw;
  map3->depth = gd;
  map3->height = gh;
  map3->data.resize(gw*gd*gh);

  for (unsigned i=0; i<m_kinect3d.size(); ++i) {
    map3->data[i] = m_kinect3d[i] || m_sonar3d[i];
  }

  m_pub3d.publish(map3);
}

Occupancy::SonarPose::SonarPose(double x, double y, double dir) :
  m_x(x), m_y(y), m_dir(dir)
{ }

pair<double, double> Occupancy::SonarPose::inOccupancy
  (const Occupancy &occ, double rad, double ang) 
{
  if (rad < 3*config::RESOLUTION) {
    rad = 3*config::RESOLUTION;
  }
  double x = m_x + rad * cos(ang + m_dir);
  double y = m_y + rad * sin(ang + m_dir);
  return pair<double, double> (x, y);
}

