#!/bin/bash
ROS_DISTRO=humble
SRC_CONTAINER=/home/logilab/ros2_ws/src
SRC_HOST="$(pwd)"/src                           #use a src on host machine to develop code, mout it to the container to run code inside the container

docker run \
  --name sew_navigation \
  --restart always \
  -it \
  --net=host \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -e DISPLAY="$DISPLAY" \
  -v /dev:/dev \
  --privileged \
  --env-file .env \
  logilab-sew-maxo-mts/ros-render:"$ROS_DISTRO"