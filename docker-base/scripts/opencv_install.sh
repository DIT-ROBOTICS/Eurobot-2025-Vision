#!/usr/bin/env bash
# this script installs OpenCV from deb packages that it downloads
# the opencv_version.sh script selects which packages to use

set -e -x

OPENCV_URL=$1
OPENCV_DEB=$2
OPENCV_CONTRIB_URL=$3
OPENCV_CONTRIB_DEB=$4
OPENCV_VERSION=$5

echo "OPENCV_URL = $OPENCV_URL"
echo "OPENCV_DEB = $OPENCV_DEB"
echo "OPENCV_CONTRIB_URL = $OPENCV_CONTRIB_URL"
echo "OPENCV_CONTRIB_DEB = $OPENCV_CONTRIB_DEB"
echo "OPENCV_VERSION = $OPENCV_VERSION"

ARCH=$(uname -i)
echo "ARCH:  $ARCH"

# remove previous OpenCV installation if it exists
apt-get purge -y '.*opencv.*' || echo "previous OpenCV installation not found"

# download and extract the deb packages
mkdir opencv
cd opencv
wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${OPENCV_URL} -O ${OPENCV_DEB}
wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${OPENCV_CONTRIB_URL} -O ${OPENCV_CONTRIB_DEB}
tar -xzvf ${OPENCV_DEB}
tar -xzvf ${OPENCV_CONTRIB_DEB}

cd opencv-${OPENCV_VERSION}
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${OPENCV_VERSION}/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN=8.6 \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D OPENCV_GENERATE_PKGCONFIG=ON ..
make -j"$(nproc)"
sudo make install

# Clean up
cd ../../
rm -rf opencv

# manage some install paths
PYTHON3_VERSION=`python3 -c 'import sys; version=sys.version_info[:3]; print("{0}.{1}".format(*version))'`

if [ $ARCH = "aarch64" ]; then
	local_include_path="/usr/local/include/opencv4"
	local_python_path="/usr/local/lib/python${PYTHON3_VERSION}/dist-packages/cv2"

	if [ -d "$local_include_path" ]; then
		echo "$local_include_path already exists, replacing..."
		rm -rf $local_include_path
	fi
	
	if [ -d "$local_python_path" ]; then
		echo "$local_python_path already exists, replacing..."
		rm -rf $local_python_path
	fi
	
	ln -s /usr/include/opencv4 $local_include_path
	ln -s /usr/lib/python${PYTHON3_VERSION}/dist-packages/cv2 $local_python_path
	
elif [ $ARCH = "x86_64" ]; then
	opencv_conda_path="/opt/conda/lib/python${PYTHON3_VERSION}/site-packages/cv2"
	
	if [ -d "$opencv_conda_path" ]; then
		echo "$opencv_conda_path already exists, replacing..."
		rm -rf $opencv_conda_path
		ln -s /usr/lib/python${PYTHON3_VERSION}/site-packages/cv2 $opencv_conda_path
	fi
fi