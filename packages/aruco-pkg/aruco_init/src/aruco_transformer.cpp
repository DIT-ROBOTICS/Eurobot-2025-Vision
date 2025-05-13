#include "aruco_init/aruco_transformer.hpp"

ArucoTransformer::ArucoTransformer(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, float markerLength)
    : cameraMatrix_(cameraMatrix), distCoeffs_(distCoeffs), markerLength_(markerLength) {}

void ArucoTransformer::estimatePose(const std::vector<std::vector<cv::Point2f>>& corners) {
    rvecs_.clear();
    tvecs_.clear();

    // Estimate Pose for all detected markers
    if (!corners.empty()) {
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength_, cameraMatrix_, distCoeffs_, rvecs_, tvecs_);
    }
}

const std::vector<cv::Vec3d>& ArucoTransformer::getRvecs() const {
    return rvecs_;
}

const std::vector<cv::Vec3d>& ArucoTransformer::getTvecs() const {
    return tvecs_;
}
