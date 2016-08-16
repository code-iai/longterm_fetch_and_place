#!/bin/bash

CAMNO=$1

cd ~/sr_experimental_data/
cd current-experiment
mkdir scene_camera

echo "Camera prepared"

rosrun image_view image_saver image_view image_saver image:="/scene_camera_${CAMNO}/image_raw" _filename_format:="${HOME}/sr_experimental_data/current-experiment/scene_camera/scene_camera_${CAMNO}_%05d.jpg"
