#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#
ARG BASE_IMAGE=nvcr.io/nvidia/cuda:12.4.1-base-ubuntu22.04
FROM ${BASE_IMAGE}

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /tmp

RUN apt-get update && apt-get install -y locales
# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8
# 
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN	update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 
# install development packages
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev \
		&& rm -rf /var/lib/apt/lists/* \
		&& apt-get clean

# 
# install OpenCV (with CUDA)
#
ARG OPENCV_URL=https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz
ARG OPENCV_DEB=openCV-4.10.0.tar.gz
ARG OPENCV_CONTRIB_URL=https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.tar.gz
ARG OPENCV_CONTRIB_DEB=openCV_contrib-4.10.0.tar.gz
ARG OPENCV_VERSION=4.10.0
COPY scripts/opencv_install.sh /tmp/opencv_install.sh
RUN cd /tmp && ./opencv_install.sh ${OPENCV_URL} ${OPENCV_DEB} ${OPENCV_CONTRIB_URL} ${OPENCV_CONTRIB_DEB} ${OPENCV_VERSION}
        
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  software-properties-common \
		  apt-transport-https \
		  ca-certificates \
		  gnupg \
	pip3 install --upgrade --no-cache-dir --verbose cmake && \
	apt purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal" && \
    ls -ll /usr/bin/python* \
	&& rm -rf /var/lib/apt/lists/* \
    && apt-get clean
    
RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
		launch_xml \
		launch_yaml \
		launch_testing \
		launch_testing_ament_cmake \
		demo_nodes_cpp \
		demo_nodes_py \
		example_interfaces \
		camera_calibration_parsers \
		camera_info_manager \
		cv_bridge \
		v4l2_camera \
		vision_opencv \
		vision_msgs \
		image_geometry \
		image_pipeline \
		image_transport \
		compressed_image_transport \
		compressed_depth_image_transport \
		> ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    rm -r ${ROS_ROOT}/src/ament_cmake && \
    git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO} && \
    apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rosdep install -y \
    	  --ignore-src \
       --from-paths src \
	  --rosdistro ${ROS_DISTRO} \
	  --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf ${ROS_ROOT}/src && \
    rm -rf ${ROS_ROOT}/logs && \
    rm -rf ${ROS_ROOT}/build && \
    rm ${ROS_ROOT}/*.rosinstall
    
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#
# setup entrypoint
#
COPY ./packages/ros_entrypoint.sh /ros_entrypoint.sh

RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /