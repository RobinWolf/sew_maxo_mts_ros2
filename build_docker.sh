#!/bin/bash
##############################################################################
##                   Build the image, using dev.Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")

#pass some arguments and settings to the dev.Dockerfile while building the image (dev.Dockerfile)

#--no-cache
docker build \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f dev.Dockerfile \
  -t logilab-sew-maxo-mts/ros-render:"$ROS_DISTRO" .
