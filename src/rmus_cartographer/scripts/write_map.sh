#!/bin/bash
set -e

map_dir=$(realpath "$1")
mkdir -p "$map_dir"

rosservice call /finish_trajectory 0
rosservice call /write_state "filename: '$map_dir/map.pbstream'"
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
    -map_filestem="$map_dir/map" \
    -pbstream_filename="$map_dir/map.pbstream" \
    -resolution=0.05
