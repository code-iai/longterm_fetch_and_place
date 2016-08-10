#!/bin/bash

N_COUNT=$1

for (( i=0; i<${N_COUNT}; i++ )); do
    roslaunch ltfnp_executive ltfnp_automated.launch
    ./kill_type.sh mongodb_log
    ./kill_type.sh semrec
    ./kill_type.sh gzserver
    ./kill_type.sh gazebo
done
