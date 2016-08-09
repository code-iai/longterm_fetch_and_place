#!/bin/bash

N_COUNT=$1

for (( i=0; i<${N_COUNT}; i++ )); do
    roslaunch ltfnp_executive ltfnp_automated.launch
    ./kill_type mongodb_log
    ./kill_type semrec
    ./kill_type gzserver
    ./kill_type gazebo
done
