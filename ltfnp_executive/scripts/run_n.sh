#!/bin/bash

N_COUNT=$1

for (( i=0; i<${N_COUNT}; i++ )); do
    roslaunch ltfnp_executive ltfnp_automated.launch
    rosrun ltfnp_executive mongodb_log
    rosrun ltfnp_executive semrec
    rosrun ltfnp_executive gzserver
    rosrun ltfnp_executive gazebo
    rosrun ltfnp_executive /opt/ros/
    rosrun ltfnp_executive nav_pcontroller
done
