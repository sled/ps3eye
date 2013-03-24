#!/bin/bash
BASEDIR=`pwd`

source /opt/ros/groovy/setup.sh

cd $BASEDIR/catkin_ws/src
catkin_init_workspace
cd $BASEDIR/catkin_ws
catkin_make

source devel/setup.bash

cd $BASEDIR/rosbuild_ws
rosws init . $BASEDIR/catkin_ws/devel

