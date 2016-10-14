#!/bin/bash

#rosrun ltfnp_executive scripts/kill_type.sh roslaunch
rosrun ltfnp_executive scripts/kill_type.sh mongodb_log
rosrun ltfnp_executive scripts/kill_type.sh semrec
rosrun ltfnp_executive scripts/kill_type.sh gzserver
rosrun ltfnp_executive scripts/kill_type.sh gazebo
#rosrun ltfnp_executive scripts/kill_type.sh /opt/ros/
rosrun ltfnp_executive scripts/kill_type.sh nav_pcontroller
rosrun ltfnp_executive scripts/kill_type.sh "args4j/args4j/"
rosrun ltfnp_executive scripts/kill_type.sh sbcl
rosrun ltfnp_executive scripts/kill_type.sh /opt/ros/indigo/lib/image_view/image_saver

killall gzclient -q
killall gzserver -q

echo "Cleanup done."
