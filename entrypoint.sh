#!/bin/bash

__DOCKER_CMD=$*

cd ~/workspace

source /opt/ros/noetic/setup.bash
source devel/setup.bash

if [[ "$WAIT_SIMULATOR" == "true" ]]; then
    #### Wait for simulator to start
    echo "Wait for topic /cmd_vel" >&2
    while true; do
        if rostopic list | grep '/cmd_vel' 2>/dev/null >/dev/null; then
            break
        fi
    done
    echo "/cmd_vel topic is now subscribed" >&2
fi

$__DOCKER_CMD
