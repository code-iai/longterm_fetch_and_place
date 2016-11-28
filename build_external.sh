#!/bin/bash

mkdir -p build
cd build
cmake ../external
make
mkdir -p ../ltfnp_gazebo/plugins
mv ./gazebo_attache_plugin/libattache.so ../ltfnp_gazebo/plugins/
cd -
rm -Rf build
