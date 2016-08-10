#!/bin/bash

TERM=$1

PIDS=`ps aux | grep ${TERM} | grep -v grep | awk '{print $2}'`

printf '%s\n' "$PIDS" | while IFS= read -r PID
do
    kill -s SIGKILL ${PID} &>/dev/null 2>&1
done
