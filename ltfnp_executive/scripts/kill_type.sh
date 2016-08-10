#!/bin/bash

TERM=$1

PIDS=`ps aux | grep ${TERM} | grep -v grep | awk '{print $2}'`
echo "$1 = $PIDS"
printf '%s\n' "$PIDS" | while IFS= read -r PID
do
    echo "Doing:     kill -s SIGKILL ${PID}"
    kill -s SIGKILL ${PID}
done
