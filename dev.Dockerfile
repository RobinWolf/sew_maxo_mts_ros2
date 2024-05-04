##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop as base

# Configure DDS
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Create user
ARG USER=logilab
ARG PASSWORD=automaton
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID -p "$(openssl passwd -1 $PASSWORD)" \
    --shell $(which bash) $USER -G sudo

# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws/src
WORKDIR /home/$USER/ros2_ws

##############################################################################
##                           2. stage: set up gazebo                        ##
##############################################################################
FROM base as gazebo_testenviroment

#install some ros packages
USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-xacro
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-teleop-twist-joy
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-teleop-twist-keyboard
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joy
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-gz
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
USER $USER

#copy the description package into this ws
# --> TODO