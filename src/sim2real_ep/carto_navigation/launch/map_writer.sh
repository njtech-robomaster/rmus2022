#!/bin/bash
rosservice call /finish_trajectory 0
rosservice call /write_state "filename: '/home/sim2real/ep_ws/src/sim2real_ep/carto_navigation/maps/map.pbstream'"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/sim2real/ep_ws/src/sim2real_ep/carto_navigation/maps/map -pbstream_filename=/home/sim2real/ep_ws/src/sim2real_ep/carto_navigation/maps/map.pbstream -resolution=0.05 -initial_pose '{to_trajectory_id = 0, relative_pose = { translation = { 0., 0., 0. }, rotation = { 0.0, 0.0, 0.0 } } }'
