#!/bin/bash
set -em

#### Setup ROS noetic
source /opt/ros/noetic/setup.bash

#### Setup conda
__conda_setup="$('/home/sim2real/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/sim2real/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/sim2real/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/sim2real/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
conda activate habitat

#### Setup ROS workspace
export PYTHONPATH=$PYTHONPATH:/home/sim2real/ros_x_habitat_ws/src/ros_x_habitat/
source /home/sim2real/ros_x_habitat_ws/devel/setup.bash

#### Start ROS core
echo "Starting roscore" >&2
roscore &
onexit(){
	set +e
	if jobs -rp | grep -P "\d+" > /dev/null; then
	    echo "Stopping roscore" >&2
		pkill -15 -P $(jobs -rp)
		fg 1
	fi
}
trap onexit EXIT

# Wait for roscore to start
while true; do
    if curl -s http://localhost:11311/ > /dev/null; then
        break
    fi
done
echo "roscore started!" >&2

#### Start simulator
cd ~/ros_x_habitat_ws/src/ros_x_habitat/

if [[ "$SERVER_PATCH" != "" ]]; then
    git add -A
    git reset HEAD --hard
    echo "Applying patch $SERVER_PATCH"
    git apply "/server_patches/$SERVER_PATCH"
fi

python3 src/scripts/roam_with_joy.py --hab-env-config-path ./configs/roam_configs/pointnav_rgbd_roam_mp3d_test_scenes.yaml
