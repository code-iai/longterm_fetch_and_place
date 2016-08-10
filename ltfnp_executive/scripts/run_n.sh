#!/bin/bash

N_COUNT=$1

for (( i=0; i<${N_COUNT}; i++ )); do
    roslaunch ltfnp_executive ltfnp_automated.launch
    rosrun ltfnp_executive kill_all_relevant.sh
done
