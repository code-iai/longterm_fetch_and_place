#!/bin/bash

cd ~/sr_experimental_data/
cd current-experiment
mkdir scene_camera

echo "Camera prepared"

rosrun image_view image_saver image_view image_saver image:=/scene_camera/image_raw _filename_format:=${HOME}/sr_experimental_data/current-experiment/scene_camera/cap_%04d.jpg
