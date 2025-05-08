variable "ARCH" { default = "linux/amd64" }
variable "BASE_IMAGE" { default = "ros:humble" }
variable "NVIDIA_BASE_IMAGE" { default = "nvcr.io/nvidia/pytorch:24.07-py3" }
variable "USER_UID" { default = "1000" }

variable "LIBREALSENSE_VERSION" { default = "2.55.1" }
variable "REALSENSE_ROS_VERSION" { default = "4.55.1" }

group "default" {
  targets = ["center", "realsense", "aruco", "ultralytics", "gui", "stitch"]
}

target "center" {
  context = "../"
  dockerfile = "dockerfiles/ros.Dockerfile"
  target = "center"
  tags = ["vision-main/center:latest"]
  args = {
    ARCH = "${ARCH}"
    BASE_IMAGE = "${BASE_IMAGE}"
    USER = "center"
    USER_UID = "${USER_UID}"
  }
}

target "realsense" {
  context = "../"
  dockerfile = "dockerfiles/ros.Dockerfile"
  target = "realsense"
  tags = ["vision-main/realsense:latest"]
  args = {
    ARCH = "${ARCH}"
    BASE_IMAGE = "${BASE_IMAGE}"
    USER = "realsense"
    USER_UID = "${USER_UID}"
    LIBREALSENSE_VERSION = "${LIBREALSENSE_VERSION}"
    REALSENSE_ROS_VERSION = "${REALSENSE_ROS_VERSION}"
  }
}

target "aruco" {
  context = "../"
  dockerfile = "dockerfiles/ros.Dockerfile"
  target = "aruco"
  tags = ["vision-main/aruco:latest"]
  args = {
    ARCH = "${ARCH}"
    BASE_IMAGE = "${BASE_IMAGE}"
    USER = "aruco"
    USER_UID = "${USER_UID}"
  }
}

target "ultralytics" {
  context = "../"
  dockerfile = "dockerfiles/nvidia.Dockerfile"
  target = "ultralytics-ros"
  tags = ["vision-main/ultralytics-ros:latest"]
  args = {
    ARCH = "${ARCH}"
    BASE_IMAGE = "${BASE_IMAGE}"
    NVIDIA_BASE_IMAGE = "${NVIDIA_BASE_IMAGE}"
    USER = "ultralytics"
    USER_UID = "${USER_UID}"
  }
}

target "gui" {
  context = "../"
  dockerfile = "dockerfiles/ros.Dockerfile"
  target = "gui"
  tags = ["vision-main/gui:latest"]
  args = {
    ARCH = "${ARCH}"
    BASE_IMAGE = "${BASE_IMAGE}"
    USER = "gui"
    USER_UID = "${USER_UID}"
  }
}

target "stitch" {
  context = "../"
  dockerfile = "dockerfiles/ros.Dockerfile"
  target = "stitch"
  tags = ["vision-main/stitch:latest"]
  args = {
    ARCH = "${ARCH}"
    BASE_IMAGE = "${BASE_IMAGE}"
    USER = "stitch"
    USER_UID = "${USER_UID}"
  }
}
