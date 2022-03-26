#!/bin/bash

cd ~/workspace

source /opt/ros/noetic/setup.bash

if [[ "$BUILD_RELEASE" == "true" ]]; then
	catkin_make_isolated --install --use-ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_EXECUTABLE=$(which python3) $*
else
	catkin_make_isolated --install --use-ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPYTHON_EXECUTABLE=$(which python3) $*
	mkdir -p build
	jq -s 'map(.[])' build_isolated/*/compile_commands.json > build/compile_commands.json
fi
