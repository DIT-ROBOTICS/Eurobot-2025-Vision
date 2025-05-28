#pragma once

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

class ArucoTransformer {
public:
    ArucoTransformer(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, float markerLength);
    void estimatePose(const std::vector<std::vector<cv::Point2f>>& corners);
    void drawMarkers(
        cv::Mat& image,
        const std::vector<int>& ids,
        const std::vector<std::vector<cv::Point2f>>& corners);

        tf2::Transform getMarkerInMapTf(const cv::Vec3d& rvec, const cv::Vec3d& tvec, const tf2::Transform& tf_cam_to_map);
    const cv::Mat& getCameraMatrix() const { return K_; }
    const cv::Mat& getDistCoeffs() const { return D_; }
    const std::vector<cv::Vec3d>& getRvecs() const { return rvecs_; }
    const std::vector<cv::Vec3d>& getTvecs() const { return tvecs_; }
    
private:
    cv::Mat K_;
    cv::Mat D_;
    float markerLength_; // meter
    std::vector<cv::Vec3d> rvecs_;
    std::vector<cv::Vec3d> tvecs_;
    std::vector<cv::Point3f> objectPoints_;
};
