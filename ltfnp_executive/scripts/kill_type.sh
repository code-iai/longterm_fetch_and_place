#!/bin/bash

TERM=$1

PIDS=`ps aux | grep ${TERM} | grep -v grep | awk '{print $2}'`

kill -s SIGKILL ${PIDS}
