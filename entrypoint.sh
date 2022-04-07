#!/bin/bash

cd ~/workspace

source devel/setup.bash

#### Wait for simulator to start
echo "Wait for topic /cmd_vel" >&2
while true; do
    if rostopic list | grep '/cmd_vel' 2>/dev/null >/dev/null; then
        break
    fi
done
echo "/cmd_vel topic is now subscribed" >&2

$*
