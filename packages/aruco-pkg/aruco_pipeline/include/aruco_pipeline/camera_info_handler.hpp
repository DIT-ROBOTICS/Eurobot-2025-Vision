#pragma once

#include <sensor_msgs/msg/camera_info.hpp>
#include <map>
#include <string>
#include <set>
#include <mutex>
#include <opencv2/core.hpp>

class CameraInfoHandler {
public:
  CameraInfoHandler(const std::set<std::string>& required_cameras)
    : required_cameras_(required_cameras) {}

  void setCameraInfo(const std::string& name, const sensor_msgs::msg::CameraInfo& info) {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_infos_[name] = info;
    // K matrix 
    assert(info.k.size() == 9 && "CameraInfo::k must have 9 elements");
    camera_matrices_[name] = cv::Mat(3, 3, CV_64F);
    std::memcpy(camera_matrices_[name].data, info.k.data(), 9 * sizeof(double));

    // D distortion
    cv::Mat dist(info.d.size(), 1, CV_64F);
    std::memcpy(dist.data, info.d.data(), info.d.size() * sizeof(double));
    dist_coeffs_[name] = dist;
  }

  bool hasAllCameras() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& name : required_cameras_) {
      if (camera_infos_.count(name) == 0) {
        return false;
      }
    }
    return true;
  }

  sensor_msgs::msg::CameraInfo get(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return camera_infos_.at(name);
  }

  cv::Mat getCameraMatrix(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return camera_matrices_.at(name);
  }

  cv::Mat getDistCoeffs(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dist_coeffs_.at(name);
  }

private:
  std::map<std::string, sensor_msgs::msg::CameraInfo> camera_infos_;
  std::map<std::string, cv::Mat> camera_matrices_;
  std::map<std::string, cv::Mat> dist_coeffs_;
  std::set<std::string> required_cameras_;
  mutable std::mutex mutex_;
};
