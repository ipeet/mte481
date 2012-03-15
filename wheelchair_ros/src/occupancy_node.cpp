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
#include <memory>

#include "wheelchair_ros/Occupancy3D.h"
#include "wheelchair_ros/Sonar.h"
#include "wheelchair_ros/occupancy.hpp"
#include "wheelchair_ros/config.hpp"

using namespace std;
using namespace nav_msgs;
using namespace wheelchair_ros;
using namespace config;

auto_ptr<Occupancy> occ;

void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg) {
  occ->handlePointcloud(msg);
}

void sonarCallback(const Sonar::ConstPtr &msg) {
  occ->handleSonar(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "occupancy_node");
  ros::NodeHandle nh;
  ros::Subscriber points = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(
      "/camera/depth/points", 1, pointcloudCallback);
  ros::Subscriber sonar = nh.subscribe<Sonar>("sonar", 1, sonarCallback);
 
  occ = auto_ptr<Occupancy> (new Occupancy
    (nh, WIDTH, DEPTH, HEIGHT, ORIG_X, ORIG_Y, ORIG_Z, RESOLUTION));
 
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    occ->publish();
    loop_rate.sleep();
  }
  ros::spin();
}

