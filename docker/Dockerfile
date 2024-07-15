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
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-desktop && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install -y libzmq3-dev && \
    apt-get install -y libncurses5-dev && \
    apt-get install -y mesa-utils libgl1-mesa-glx && \
    apt-get install -y ros-humble-nav2-bringup && \
    apt-get install -y ros-humble-cartographer && \
    apt-get install -y ros-humble-cartographer-ros && \
    apt-get install -y ros-humble-gazebo-ros && \
    apt-get install -y ros-humble-gazebo-ros-pkgs && \
    apt-get install -y ros-humble-test-msgs && \
    apt-get install -y ros-humble-xacro && \
    apt-get install -y ros-humble-joint-state-publisher-gui && \
    apt-get install -y net-tools && \
    apt-get install -y iproute2 

RUN apt-get install -y nano gedit tmux

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]