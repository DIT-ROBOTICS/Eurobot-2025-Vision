FROM ros:humble AS base
LABEL org.opencontainers.image.authors="ohin.kyuu@gmail.com"
LABEL org.opencontainers.image.vendor="DIT-Robotics"
ENV TERM=xterm-256color
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
    git \
    sudo \
    curl \
    wget \
    usbutils \
    v4l-utils \
    net-tools \
    iputils-ping \
    apt-transport-https \
    software-properties-common \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \    
    ros-humble-rmw-cyclonedds-cpp \
    && apt clean -y && rm -rf /var/lib/apt/lists/*

FROM base AS librealsense-builder
ARG LIBREALSENSE_VERSION
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
    cmake \
    pkg-config \
    build-essential \
    python3 \
    python3-dev \
    ca-certificates \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
    && apt clean -y && rm -rf /var/lib/apt/lists/*  
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBREALSENSE_VERSION -o librealsense.tar.gz && \
    tar -zxf librealsense.tar.gz && \
    rm librealsense.tar.gz && \
    ln -s /usr/src/librealsense-$LIBREALSENSE_VERSION /usr/src/librealsense
# Build librealsense from source
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \    
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    && make -j$(($(nproc)-1)) all \
    && make install 

###### RealSense Module ######
FROM base AS realsense
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib
# Install dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev \
    ca-certificates \
    ros-humble-librealsense2 \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    ros-humble-launch-pytest \
    ros-humble-sensor-msgs-py \
    python3-tqdm \
    python3-requests \
    && apt clean -y && rm -rf /var/lib/apt/lists/*
# Add user and setup workspace
RUN groupadd --gid $USER_GID $USER && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc && \
    echo "source /home/$USER/vision-ws/install/local_setup.bash" >> /home/$USER/.bashrc
USER $USER
RUN mkdir -p /home/$USER/vision-ws/src && \
    # Install ROS2 Realsense package
    git clone --branch ros2-master https://github.com/IntelRealSense/realsense-ros.git /home/$USER/vision-ws/src/realsense-ros && \
    rosdep update && \
    rosdep install -i --from-path /home/$USER/vision-ws/src --rosdistro humble -y && \
    cd /home/$USER/vision-ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                colcon build --symlink-install"
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]

###### Aruco Module ######
FROM base AS aruco 
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USER && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc && \
    echo "source /home/$USER/vision-ws/install/local_setup.bash" >> /home/$USER/.bashrc
USER $USER
RUN mkdir -p /home/$USER/vision-ws/src && \
    # Install ROS2 Aruco package
    git clone --branch humble-devel https://github.com/pal-robotics/aruco_ros.git /home/$USER/vision-ws/src/aruco-ros && \
    rosdep update && \
    rosdep install -i --from-path /home/$USER/vision-ws/src --rosdistro humble -y && \
    cd /home/$USER/vision-ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                colcon build --symlink-install"
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]

###### GUI Module ######
FROM base AS gui
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    && apt clean -y && rm -rf /var/lib/apt/list/*
RUN groupadd --gid $USER_GID $USER && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
USER $USER
CMD [ "/bin/bash" ]

###### Ultralytics ROS Module - AMD64 ######
FROM base AS ultralytics-ros-amd64
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USER && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc && \
    echo "source /home/$USER/vision-ws/install/local_setup.bash" >> /home/$USER/.bashrc
USER $USER
RUN mkdir -p /home/$USER/vision-ws/src
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt-get install -y python3-pip && \
    pip3 install --upgrade pip && \
    pip3 install ultralytics numpy==1.25.0
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]


###### Ultralytics ROS Module - Jetson ######
FROM ultralytics/ultralytics:latest-jetson-jetpack6 AS ultralytics-ros-jetson
LABEL org.opencontainers.image.authors="ohin.kyuu@gmail.com"
LABEL org.opencontainers.image.vendor="DIT-Robotics"
ENV TERM=xterm-256color
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Copy binaries from ros humble base stage
COPY --from=base /opt/ros/humble /opt/ros/humble
COPY --from=base /usr/bin/ /usr/bin/
COPY --from=base /usr/lib/ /usr/lib/
COPY --from=base /usr/local/lib/ /usr/local/lib/
# Install dependencies
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
    sudo \
    libblas3 liblapack3 \
    && apt clean -y && rm -rf /var/lib/apt/list/*
# Add user and setup workspace
RUN groupadd --gid $USER_GID $USER && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc && \
    echo "source /home/$USER/vision-ws/install/local_setup.bash" >> /home/$USER/.bashrc
USER $USER
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]