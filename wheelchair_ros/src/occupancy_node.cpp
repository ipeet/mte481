/******************************************************************************
 * occupancy_node.cpp
 * Copyright 2011 Iain Peet
 *
 * Computes an occupancy grid.
 ******************************************************************************
 * This program is distributed under the of the GNU Lesser Public License. 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *****************************************************************************/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <math.h>
#include <GL/gl.h>

#include "wheelchair_ros/Occupancy3D.h"

using namespace std;
using namespace nav_msgs;
using namespace wheelchair_ros;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub2d;
ros::Publisher pub3d;

const double RESOLUTION = 0.10; // in m
const double WIDTH = 10.0;  // in m
const double DEPTH = 10.0;
const double HEIGHT = 0.5; // in m
const double ORIG_X = -5.0;
const double ORIG_Y = -2.0;

void pointcloudCallback(const PointCloud::ConstPtr &msg) {
  /* Convenience */
  int w = WIDTH / RESOLUTION;
  int d = DEPTH / RESOLUTION;
  int h = HEIGHT / RESOLUTION;

  /* Publish the occupancy grids */
  OccupancyGrid::Ptr map (new OccupancyGrid);
  map->info.resolution = RESOLUTION;
  map->info.width = w;
  map->info.height = d;
  map->data.resize(w * d);
  map->info.origin.position.x = ORIG_X;
  map->info.origin.position.y = ORIG_Y;

  Occupancy3D::Ptr map3 (new Occupancy3D);
  map3->width = w;
  map3->depth = d;
  map3->height = h;
  map3->data.resize(w * d * h);

  for (unsigned i=0; i < msg->points.size(); ++i) {
    double x = msg->points.at(i).x;
    double y = -msg->points.at(i).y;
    double z = msg->points.at(i).z;

    /* Bounds check the point */
    if (isnan(x) || isnan(y) || isnan(z)) continue;
    if (x < ORIG_X) continue;
    if (x >= ORIG_X + WIDTH) continue;
    if (z < ORIG_Y) continue;
    if (z >= ORIG_Y + DEPTH) continue;
    if (y <= 0.0) continue;
    if (y > HEIGHT) continue;
    int xi = (x - ORIG_X) / RESOLUTION;
    int zi = (z - ORIG_Y) / RESOLUTION;
    int yi = y / RESOLUTION;
    map->data[xi*w + zi] = 100;
    map3->data[yi*d*w + zi*w + xi] = 1;
  }
  pub2d.publish(map);
  pub3d.publish(map3);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "occupancy_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>(
      "/camera/depth/points", 1, pointcloudCallback);
  pub2d = nh.advertise<OccupancyGrid> ("map", 1);
  pub3d = nh.advertise<Occupancy3D> ("map3d", 1);
  ros::spin();
}
