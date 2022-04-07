#!/bin/bash
set -e -o pipefail

cd ~/workspace

if [[ "$GENERATE_COMPILE_COMMANDS" == "true" ]]; then
	catkin build $*
else
	catkin build $*
	jq -s 'map(.[])' build/*/compile_commands.json > build/compile_commands.json
fi
