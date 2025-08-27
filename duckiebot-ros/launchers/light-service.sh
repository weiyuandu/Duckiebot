#!/bin/bash


source /environment.sh


dt-launchfile-init
rosrun my_package light_service_node.py
dt-launchfile-join
