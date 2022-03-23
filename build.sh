#!/bin/bash

cd ~/workspace

source /opt/ros/noetic/setup.bash

catkin_make_isolated --install --use-ninja -DPYTHON_EXECUTABLE=$(which python3) $*
