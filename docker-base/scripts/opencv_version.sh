#!/usr/bin/env bash
# this script selects which OpenCV deb packages to install based on the platform

# do this first:
# source scripts/docker_base.sh

if [ $ARCH = "aarch64" ]; then
	echo "selecting OpenCV for L4T R$L4T_VERSION..."

	if [[ $L4T_RELEASE -eq 32 ]]; then
		OPENCV_URL="https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz"
		OPENCV_DEB="OpenCV-4.5.0-aarch64.tar.gz"
	elif [[ $L4T_RELEASE -eq 34 || $L4T_RELEASE -eq 35 ]]; then
		OPENCV_URL="https://nvidia.box.com/shared/static/2hssa5g3v28ozvo3tc3qwxmn78yerca9.gz"
		OPENCV_DEB="OpenCV-4.5.0-aarch64.tar.gz"
	fi
	
elif [ $ARCH = "x86_64" ]; then
	echo "selecting OpenCV for $ARCH"

	OPENCV_URL="https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz"
	OPENCV_DEB="openCV-4.10.0.tar.gz"
	OPENCV_CONTRIB_URL="https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.tar.gz"
	OPENCV_CONTRIB_DEB="openCV_contrib-4.10.0.tar.gz"
fi

echo "OPENCV_URL=$OPENCV_URL"
echo "OPENCV_DEB=$OPENCV_DEB"
echo "OPENCV_CONTRIB_URL=$OPENCV_CONTRIB_URL"
echo "OPENCV_CONTRIB_DEB=$OPENCV_CONTRIB_DEB"