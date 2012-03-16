#!/bin/bash

if [[ $(uname -n) == "lap" ]]; then
  export ROS_MASTER_URI="http://JTOP:11311"
else
  export ROS_MASTER_URI="http://lap:11311"
fi

roslaunch wheelchair_ros ui.launch
