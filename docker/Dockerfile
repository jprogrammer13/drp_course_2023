FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install sudo 
RUN apt-get install -qqy lsb-release gnupg2 curl -y

ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# install ros package
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rqt-robot-steering  \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

RUN apt-get install -y nano tmux htop gedit