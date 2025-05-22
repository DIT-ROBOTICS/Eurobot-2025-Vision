#!/bin/sh
USER=$1

. "/opt/ros/humble/setup.sh"
rosdep init
rosdep update
rosdep install --from-paths /home/$USER/vision-ws/src --rosdistro humble --skip-keys=librealsense2 -y
cd /home/$USER/vision-ws/
colcon build --symlink-install
echo "source /home/$USER/vision-ws/install/setup.bash" >> /home/$USER/.bashrc