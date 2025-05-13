#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <vector>

class ArucoTransformer {
public:
    ArucoTransformer(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, float markerLength);
    void estimatePose(const std::vector<std::vector<cv::Point2f>>& corners);

    const std::vector<cv::Vec3d>& getRvecs() const;
    const std::vector<cv::Vec3d>& getTvecs() const;

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    float markerLength_; // meter
    std::vector<cv::Vec3d> rvecs_;
    std::vector<cv::Vec3d> tvecs_;
};
