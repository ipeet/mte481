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

using namespace std;
using namespace nav_msgs;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;

const double RESOLUTION = 0.10; // in m
const double WIDTH = 10.0;  // in m
const double HEIGHT = 10.0;
const double ORIG_X = -5.0;
const double ORIG_Y = -1.0;

void pointcloudCallback(const PointCloud::ConstPtr &msg) {
  OccupancyGrid::Ptr map (new OccupancyGrid);
  map->info.resolution = RESOLUTION;
  map->info.width = WIDTH / RESOLUTION;
  map->info.height = WIDTH / RESOLUTION;
  map->data.resize(map->info.width * map->info.height);
  map->info.origin.position.x = ORIG_X;
  map->info.origin.position.y = ORIG_Y;
  for (unsigned i=0; i < msg->points.size(); ++i) {
    double x = msg->points.at(i).x;
    double y = msg->points.at(i).y;
    double z = msg->points.at(i).z;

    /* Bounds check the point */
    if (isnan(x) || isnan(y) || isnan(z)) continue;
    if (x < ORIG_X) continue;
    if (x >= ORIG_X + WIDTH) continue;
    if (z < ORIG_Y) continue;
    if (z >= ORIG_Y + HEIGHT) continue;
    if (y < 0.0) continue;
    if (y > 0.5) continue;
    int xi = (x - ORIG_X) / RESOLUTION;
    int yi = (z - ORIG_Y) / RESOLUTION;
    map->data[xi*(map->info.width) + yi] = 100;
  }
  pub.publish(map);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "occupancy_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>(
      "/camera/depth/points", 1, pointcloudCallback);
  pub = nh.advertise<OccupancyGrid> ("map", 1);
  ros::spin();
}
