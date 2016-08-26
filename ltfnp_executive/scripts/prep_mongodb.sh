#!/bin/bash

mongo roslog --eval "db.dropDatabase()"
rosrun mongodb_log mongodb_log /tf /logged_designators /logged_metadata /head_mount_kinect/rgb/image_raw
