ARG ARCH
ARG BASE_IMAGE
ARG NVIDIA_BASE_IMAGE
ARG NVIDIA_CUDA_IMAGE
ARG OPENCV_VERSION=4.5.3

FROM --platform=${ARCH} ${BASE_IMAGE} AS ros-builder
ARG DEBIAN_FRONTEND=noninteractive
COPY ../scripts/build/ /tmp/
RUN sh /tmp/install_depend.sh

###### Ultrayltics Module ######
FROM --platform=${ARCH} ${NVIDIA_BASE_IMAGE} AS ultralytics-ros
ARG USER
ARG USER_UID
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV ROS_DISTRO=humble
ENV ROS_WS_PATH=/home/$USER/vision-ws
# Copy binaries from ros humble base stage
COPY --from=ros-builder /opt/ros/humble /opt/ros/humble
COPY --from=ros-builder /usr/bin/ /usr/bin/
COPY --from=ros-builder /usr/lib/ /usr/lib/
COPY --from=ros-builder /usr/local/lib/ /usr/local/lib/
COPY ../scripts/build/ /tmp/
COPY ../scripts/entrypoint/ros_entrypoint.sh /ros_entrypoint.sh
RUN chown root:root /ros_entrypoint.sh && \
    chmod 755 /ros_entrypoint.sh && \
    apt-get update && apt-get install -y \
    libblas3 liblapack3 \
    && apt-get clean -y && rm -rf /var/lib/apt/list/* && \
    sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
ENTRYPOINT [ "/ros_entrypoint.sh" ]
USER $USER
RUN pip install --no-cache-dir ultralytics && \
    pip install --no-cache-dir cupy-cuda12x && \
    pip install --no-cache-dir "numpy<2.0" && \
    pip uninstall -y opencv-python-headless && \
    mkdir -p $ROS_WS_PATH/src && \
    sh /tmp/rosdep_init.sh $USER
WORKDIR $ROS_WS_PATH
CMD [ "/bin/bash" ]

###### Stitcher Module ######
FROM --platform=${ARCH} ${NVIDIA_CUDA_IMAGE} AS stitcher
ARG USER
ARG USER_UID
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV ROS_DISTRO=humble
ENV ROS_WS_PATH=/home/$USER/vision-ws
# Copy binaries from ros humble base stage
COPY --from=ros-builder /opt/ros/humble /opt/ros/humble
COPY --from=ros-builder /usr/bin/ /usr/bin/
COPY --from=ros-builder /usr/lib/ /usr/lib/
COPY --from=ros-builder /usr/local/lib/ /usr/local/lib/
COPY ../scripts/build/ /tmp/
COPY ../scripts/entrypoint/ros_entrypoint.sh /ros_entrypoint.sh
RUN chown root:root /ros_entrypoint.sh && \
    chmod 755 /ros_entrypoint.sh && \
    apt-get update && apt-get install -y \
    libblas3 liblapack3 sudo\
    && apt-get clean -y && rm -rf /var/lib/apt/list/* && \
    sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
# Install CuPy with CUDA support
RUN pip install --no-cache-dir cupy-cuda12x && \
    pip install --no-cache-dir "numpy<2.0"

ENTRYPOINT [ "/ros_entrypoint.sh" ]
USER $USER
WORKDIR $ROS_WS_PATH
RUN mkdir -p $ROS_WS_PATH/src && \
    sh /tmp/rosdep_init.sh $USER
CMD [ "/bin/bash" ]