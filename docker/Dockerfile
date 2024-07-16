FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN bash -c "chmod +x /ros_entrypoint.sh"

RUN apt-get update
RUN apt-get install sudo 
RUN apt-get install -qqy lsb-release gnupg2 curl -y

ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS2 packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y ros-humble-desktop
RUN apt-get install -y python3-colcon-common-extensions
RUN apt-get install -y libzmq3-dev
RUN apt-get install -y libncurses5-dev
RUN apt-get install -y mesa-utils libgl1-mesa-glx
RUN apt-get install -y ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-cartographer
RUN apt-get install -y ros-humble-cartographer-ros
RUN apt-get install -y ros-humble-gazebo-*
RUN apt-get install -y ros-humble-test-msgs
RUN apt-get install -y ros-humble-xacro
RUN apt-get install -y ros-humble-joint-state-publisher-gui
RUN apt-get install -y ros-humble-joint-state-publisher 
RUN apt-get install -y ros-humble-diff-drive-controller 
RUN apt-get install -y ros-humble-control-* 
RUN apt-get install -y ros-humble-teleop-twist-keyboard 
RUN apt-get install -y net-tools
RUN apt-get install -y iproute2 

RUN apt-get install -y nano gedit tmux

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]