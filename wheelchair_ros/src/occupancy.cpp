#include "wheelchair_ros/occupancy.hpp"

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
  m_map2d(new OccupancyGrid),
  m_pub3d(node.advertise<Occupancy3D>("map3d", 1)),
  m_map3d(new Occupancy3D),
  m_resolution(res),
  m_width(w),
  m_depth(d),
  m_height(h),
  m_orig_x(ox),
  m_orig_y(oy),
  m_orig_z(oz)
{
  /* Convenience - grid size */
  int gw = m_width / m_resolution;
  int gd = m_depth / m_resolution;
  int gh = m_height / m_resolution;

  m_map2d->info.resolution = m_resolution;
  m_map2d->info.width = gw;
  m_map2d->info.height = gd;
  m_map2d->info.origin.position.x = m_orig_x;
  m_map2d->info.origin.position.y = m_orig_y;

  m_map3d->width = gw;
  m_map3d->depth = gd;
  m_map3d->height = gh;
}

void Occupancy::handlePointcloud(const PointCloud::ConstPtr &msg) {
  /* Convenience */
  int w = m_width / m_resolution;
  int d = m_depth / m_resolution;
  int h = m_height / m_resolution;

  m_map2d->data.clear();
  m_map2d->data.resize(w*d);

  m_map3d->data.clear();
  m_map3d->data.resize(w*d*h);

  for (unsigned i=0; i < msg->points.size(); ++i) {
    double x = msg->points.at(i).x;
    double y = -msg->points.at(i).y;
    double z = msg->points.at(i).z;

    /* Bounds check the point */
    if (isnan(x) || isnan(y) || isnan(z)) continue;
    if (x < m_orig_x) continue;
    if (x >= m_orig_x + m_width) continue;
    if (y <= m_orig_y) continue;
    if (y > m_orig_y + m_height) continue;
    if (z < m_orig_z) continue;
    if (z >= m_orig_z + m_depth) continue;
    int xi = (x - m_orig_x) / m_resolution;
    int yi = (y - m_orig_y) / m_resolution;
    int zi = (z - m_orig_z) / m_resolution;
    m_map2d->data[xi*w + zi] = 100;
    m_map3d->data[yi*d*w + zi*w + xi] = 1;
  }
  m_pub2d.publish(m_map2d);
  m_pub3d.publish(m_map3d);
}

