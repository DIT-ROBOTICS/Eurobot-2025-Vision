#!/bin/bash
#

version="4.10.0"
folder="workspace"

set -e

echo "** Remove other OpenCV first"
apt -y purge *libopencv*

echo "------------------------------------"
echo "** Install requirement (1/4)"
echo "------------------------------------"
apt-get update
apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3.10-dev python3-numpy
apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils qv4l2
apt-get install -y curl unzip


echo "------------------------------------"
echo "** Download opencv "${version}" (2/4)"
echo "------------------------------------"
mkdir $folder
cd ${folder}
curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
unzip opencv-${version}.zip
unzip opencv_contrib-${version}.zip
rm opencv-${version}.zip opencv_contrib-${version}.zip
cd opencv-${version}/

export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

echo "------------------------------------"
echo "** Build opencv "${version}" (3/4)"
echo "------------------------------------"
mkdir release
cd release/
cmake -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D CUDA_ARCH_BIN="8.7" \
      -D CUDA_ARCH_PTX="" \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${version}/modules \
      -D WITH_GSTREAMER=ON \
      -D WITH_LIBV4L=ON \
      -D BUILD_opencv_python3=ON \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D WITH_NVCUVID=OFF \
      -D WITH_NVCUVENC=OFF \
      ..
make -j$(nproc)


echo "------------------------------------"
echo "** Install opencv "${version}" (4/4)"
echo "------------------------------------"
make install
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PYTHONPATH=/usr/local/lib/python3.10/site-packages/:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc

echo "** Install opencv "${version}" successfully"
echo "** Bye :)"