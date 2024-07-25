##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble

# For PC with amd64: (https://hub.docker.com/r/osrf/ros/tags?page=1&page_size=&name=&ordering=?
FROM osrf/ros:$ROS_DISTRO-desktop AS base

#For RaspberryPi with arm64: (https://hub.docker.com/r/arm64v8/ros/tags)
#FROM arm64v8/ros:$ROS_DISTRO AS base

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
##       2. stage: set up description and install ros2 control pkg          ##
##############################################################################
FROM base as sew_description

USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-xacro
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
USER $USER

USER root
RUN apt-get update && apt-get install -y ros-humble-controller-interface 
RUN apt-get update && apt-get install -y ros-humble-controller-manager 
RUN apt-get update && apt-get install -y ros-humble-hardware-interface 
RUN apt-get update && apt-get install -y ros-humble-pluginlib 
RUN apt-get update && apt-get install -y ros-humble-rclcpp
RUN apt-get update && apt-get install -y ros-humble-rclcpp-lifecycle
RUN apt-get update && apt-get install -y ros-humble-ros2-control
RUN apt-get update && apt-get install -y ros-humble-ros2-controllers
USER $USER

##############################################################################
##                           3. stage: set up gazebo                        ##
##############################################################################
FROM sew_description as gazebo_testenviroment

USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joy
    # ros-${ROS_DISTRO}-ros-gz \
    # ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # ros-${ROS_DISTRO}-gazebo-ros2-control

RUN apt-get update && apt-get install -y joystick
RUN apt-get update && apt-get install -y xterm
USER $USER

##############################################################################
##                         4. stage: set up nav2 stack                      ##
##############################################################################
FROM gazebo_testenviroment as sew_navigation

USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-* \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-twist-mux

USER $USER

# Install rviz2 because its not included in arm64 base image 
USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rviz2
USER $USER


#CMD [/bin/bash]
