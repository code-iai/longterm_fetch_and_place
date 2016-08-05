#!/bin/bash

# This script assumes that the path for stored logs on the local
# machine wasn't changed from its default
# (~/sr_experimental_data/). If it was changed, this needs to be
# reflected here; otherwise, packaging them doesn't work from
# `continuous.py`.

LOGS_PATH=~/sr_experimental_data/

cd $LOGS_PATH
./package.sh
