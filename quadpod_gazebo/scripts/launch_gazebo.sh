#!/bin/bash
#
# Script to launch gazebo with world file -
# avoids roslaunch's [gazebo_gui-3]
#
# Joseph Coombe
# Tue 24 Oct 2017
#
# Use: #TODO
#

function showHelp(){
  echo
  echo "This script can launch gazebo"
  echo
}

echo $@

if [ "$1" = "-h" ]; then
  showHelp
else
  rosrun gazebo_ros gzserver $@
  #rosrun gazebo_ros gzclient TODO: move this to its own shell script
fi

trap "killall gzserver" EXIT
