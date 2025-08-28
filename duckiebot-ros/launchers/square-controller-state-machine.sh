#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# Set environment variables
export VEHICLE_NAME=duck

# Setup ROS and run the square controller state machine node
dt-exec rosrun my_package square_controller_state_machine_node.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
