ARG ARCH
ARG BASE_IMAGE
# -----------------------------------------------------------------------------
# Base Stage: nvidia container
# -----------------------------------------------------------------------------
FROM --platform=${ARCH} ${BASE_IMAGE} AS base
LABEL org.opencontainers.image.authors="ohin.kyuu@gmail.com"
LABEL org.opencontainers.image.vendor="DIT-Robotics"
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
COPY ../scripts/build/ /tmp/
RUN sh /tmp/install_depend.sh

# -----------------------------------------------------------------------------
# Builder Stage:
#   - librealsense-builder
# -----------------------------------------------------------------------------
FROM base AS librealsense-builder
ARG LIBREALSENSE_VERSION
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    cmake \
    pkg-config \
    build-essential \
    python3 \
    python3-dev \
    ca-certificates \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
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

# -----------------------------------------------------------------------------
# Release Stage:
#   - Center
#   - Realsense
#   - Aruco
#   - GUI
# -----------------------------------------------------------------------------
###### Center Module ######
FROM base AS center
ARG USER
ARG USER_UID
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
USER $USER
RUN mkdir -p /home/$USER/vision-ws/src && \
    sh /tmp/rosdep_init.sh $USER
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]

###### Realsense Module ######
FROM base AS realsense
ARG REALSENSE_ROS_VERSION
ARG USER
ARG USER_UID
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib
# Install dependencies
RUN apt-get update && apt-get install -y \
    --no-install-recommends \
    libusb-1.0-0 \
    udev \
    ca-certificates \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    python3-tqdm \
    python3-requests \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
# Add user and setup workspace
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
USER $USER
RUN mkdir -p /home/$USER/vision-ws/src && \
    # Install ROS2 Realsense package
    git clone --branch $REALSENSE_ROS_VERSION \
        https://github.com/IntelRealSense/realsense-ros.git \
        /home/$USER/vision-ws/src/realsense-ros
COPY ../scripts/temp/ /home/$USER/vision-ws/src/realsense-ros/realsense2_camera/launch/
RUN sh /tmp/rosdep_init.sh $USER
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]

###### Aruco Module ######
FROM base AS aruco 
ARG USER
ARG USER_UID
ARG USER_GID=$USER_UID
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
USER $USER
RUN mkdir -p /home/$USER/vision-ws/src && \
    # Install ROS2 Aruco package
    git clone --branch humble-devel \
        https://github.com/pal-robotics/aruco_ros.git \
        /home/$USER/vision-ws/src/aruco-ros && \
    sh /tmp/rosdep_init.sh $USER
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]

###### GUI Module ######
FROM base AS gui
ARG USER
ARG USER_UID
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    ros-humble-foxglove-bridge \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
USER $USER
WORKDIR /home/$USER
CMD [ "/bin/bash" ]