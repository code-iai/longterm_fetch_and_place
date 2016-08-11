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
	mogrify -resize 800x800 "scene_camera_${camera_index}_*.jpg"
	
	# For stretching the individual frames
	convert *.jpg -delay 1 -morph 1 mogrified_${camera_index}_%05d.jpg
	
	# Actually stitching them together to form a video
	avconv -i "mogrified_${camera_index}_%05d.jpg" -r 30 -c:v libx264 -crf 20 -pix_fmt yuv420p ../preview_${camera_index}.mov
    done
    
    # Cleanup
    cd -
    rm -Rf current-experiment/scene_camera
fi

./package.sh
