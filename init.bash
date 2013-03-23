#!/bin/bash
BASEDIR=$(dirname $0)

catkin_init_workspace src/
wstool init $BASEDIR/src /opt/ros/groovy
