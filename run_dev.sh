#!/bin/bash
##############################################################################
##                   Build the image, using dev.Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")

#pass some arguments and settings to the dev.Dockerfile while building the image (dev.Dockerfile)
#name of the image builded here: utomaton-dev/ros-render:"ROS-Distribution eg humble"

#--no-cache
docker build \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f dev.Dockerfile \
  -t automaton-dev/ros-render:"$ROS_DISTRO" .

##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/logilab/ros2_ws/src
SRC_HOST="$(pwd)"/src                           #use a src on host machine to develop code, mout it to the container to run code inside the container

docker run \
  --name gazebo_testenviroment \
  --rm \
  -it \
  --net=host \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -e DISPLAY="$DISPLAY" \
  automaton-dev/ros-render:"$ROS_DISTRO"

# display and network access is already passed to the container