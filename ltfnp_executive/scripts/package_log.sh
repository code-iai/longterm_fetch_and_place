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
    mogrify -resize 800x600 *.jpg
    convert *.jpg -delay 1 -morph 1 %05d.jpg
    #ffmpeg -r 25 -qscale 2 -i %05d.jpg ../preview.mp4
    avconv -i "%05d.jpg" -r 30 -c:v libx264 -crf 20 -pix_fmt yuv420p ../preview.mov
    cd -
    rm -Rf scene_camera
fi

./package.sh
