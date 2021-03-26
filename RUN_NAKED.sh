#!/bin/sh

# Source environment variables from NAKED/devel
/bin/sh devel/setup.sh

# Start ros core and fork it to the background
roscore &

# Start the NAKED stack on top of ROSCORE
roslaunch control NAKED.launch
