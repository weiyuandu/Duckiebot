#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch odometry node
rosrun my_package odometry_node.py

# wait for app to end
dt-launchfile-join
