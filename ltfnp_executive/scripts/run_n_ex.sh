#!/bin/bash

N_COUNT=$1
POST_EXEC=$2

function printUsage() {
    echo "Error: $1"
    echo "Usage: $0 <n> [<post-exec-script>]"
}

if [ "${N_COUNT}" != "" ]; then
    for i in `seq 1 ${N_COUNT}`; do
	roslaunch ltfnp_executive ltfnp_automated.launch
	
	if [ "${POST_EXEC}" != "" ]; then
	    `${POST_EXEC}`
	fi
    done
else
    printUsage "Experiment count n not set"
fi
