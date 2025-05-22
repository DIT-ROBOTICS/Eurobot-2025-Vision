#!bin/sh
. "/opt/ros/humble/setup.sh"
apt-get update
apt-get install -y \
    git \
    vim \
    sudo \
    curl \
    wget \
    ccache \
    usbutils \
    v4l-utils \
    net-tools \
    iputils-ping \
    python3-pip \
    apt-transport-https \
    software-properties-common \
    ros-humble-launch-pytest \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-realsense2-camera-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins
    
apt-get clean -y 
rm -rf /var/lib/apt/lists/*

