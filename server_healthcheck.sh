#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
if rostopic list | grep '/judgement/markers_time'; then
    exit 0
else
    exit 1
fi
