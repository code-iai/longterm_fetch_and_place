#!/bin/bash

# This script assumes that the path for stored logs on the local
# machine wasn't changed from its default
# (~/sr_experimental_data/). If it was changed, this needs to be
# reflected here; otherwise, packaging them doesn't work from
# `continuous.py`.

LOGS_PATH=~/sr_experimental_data/

cd $LOGS_PATH

if [ -d "current-experiment/scene_camera" ]; then
    cd current-experiment/scene_camera
    
    # Process camera perspectives
    for camera_index in 1 2 3 4; do
	echo "Processing scene camera ${camera_index}"
	
	# Mogrification can do more; its just not used very thoroughly
	# at the moment.
	#mogrify -resize 800x800 "scene_camera_${camera_index}_*.jpg"
	
	# For stretching the individual frames
	#convert *.jpg -delay 1 -morph 1 mogrified_${camera_index}_%05d.jpg
	
	# Actually stitching them together to form a video
	#avconv -i "mogrified_${camera_index}_%05d.jpg" -r 30 -c:v libx264 -crf 20 -pix_fmt yuv420p ../preview_${camera_index}.mov
	avconv -i "scene_camera_${camera_index}_%05d.jpg" -r 30 -c:v libx264 -crf 20 -pix_fmt yuv420p ../preview_${camera_index}.mov
    done
    
    # Cleanup
    cd -
    rm -Rf current-experiment/scene_camera
    
    # Stitch together videos
    cd current-experiment
    avconv -i preview_1.mov -i preview_2.mov -filter_complex "[0:v:0]pad=iw*2:ih[bg]; [bg][1:v:0]overlay=w" preview_top.mov
    avconv -i preview_3.mov -i preview_4.mov -filter_complex "[0:v:0]pad=iw*2:ih[bg]; [bg][1:v:0]overlay=w" preview_bottom.mov
    avconv -i preview_top.mov -i preview_bottom.mov -filter_complex "[0:v:0]pad=iw:2*ih[bg]; [bg][1:v:0]overlay=0:main_h/2" preview.mov
    
    rm -Rf preview_1.mov preview_2.mov preview_3.mov preview_4.mov preview_top.mov preview_bottom.mov
    cd -
fi

./package.sh
