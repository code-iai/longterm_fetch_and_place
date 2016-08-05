#!/bin/bash

mongo roslog --eval "db.dropDatabase()"
rosrun mongodb_log mongodb_log /tf /logged_designators /logged_metadata

echo "MongoDB ready for logging"
