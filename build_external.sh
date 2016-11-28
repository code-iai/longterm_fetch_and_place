#!/bin/bash

mkdir -p build
cd build
cmake ../external
make
mv ./gazebo_attache_plugin/libattache.so ../ltfnp_gazebo/plugins
cd -
rm -Rf build

