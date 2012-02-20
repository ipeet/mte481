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

void pointcloudCallback(const PointCloud::ConstPtr &msg) {
  OccupancyGrid::Ptr map (new OccupancyGrid);
  map->info.resolution = 0.10;
  map->info.width = 20;
  map->info.height = 20;
  map->data.resize(map->info.width * map->info.height);
  map->info.origin.position.x = -1;
  map->info.origin.position.y = 1;
  for (int i=0; i < msg->points.size(); ++i) {
    double x = msg->points.at(i).x;
    double y = msg->points.at(i).y;
    double z = msg->points.at(i).z;
    if ((x<=-1.0) || (x>=1.0) || isnan(x)) continue;
    if ((y<=-1.0) || (y>=1.0) || isnan(y)) continue;
    if (isnan(z)) continue;
    int xi = (x+1.0)/0.1;
    int yi = (y+1.0)/0.1;
    map->data[xi*20 + yi] = 100;
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
