#include "aruco_pipeline/aruco_transformer.hpp"

ArucoTransformer::ArucoTransformer(const cv::Mat& K, const cv::Mat& D, float markerLength)
    : K_(K), D_(D), markerLength_(markerLength) {}

void ArucoTransformer::estimatePose(const std::vector<std::vector<cv::Point2f>>& corners) {
    rvecs_.clear();
    tvecs_.clear();

    // Estimate Pose for all detected markers
    if (!corners.empty()) {
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength_, K_, D_, rvecs_, tvecs_);
    }
}

void ArucoTransformer::drawMarkers(
    cv::Mat& image, 
    const std::vector<int>& ids,
    const std::vector<std::vector<cv::Point2f>>& corners) 
{
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
        for (size_t i = 0; i < ids.size(); ++i) {
            cv::aruco::drawAxis(image, K_, D_, rvecs_[i], tvecs_[i], markerLength_ * 0.8f);
        }
    }
}

tf2::Transform ArucoTransformer::getMarkerInMapTf(
    const cv::Vec3d& rvec, const cv::Vec3d& tvec, const tf2::Transform& tf_cam_to_map) 
{
    tf2::Transform tf_marker_in_cam;
    tf2::Vector3 marker_translation(tvec[0], tvec[1], tvec[2]);

    cv::Mat rot_matrix;
    cv::Rodrigues(rvec, rot_matrix);

    tf2::Matrix3x3 tf_rot;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            tf_rot[i][j] = rot_matrix.at<double>(i, j);
        }
    }

    tf2::Quaternion q_marker;
    tf_rot.getRotation(q_marker);

    tf_marker_in_cam.setOrigin(marker_translation);
    tf_marker_in_cam.setRotation(q_marker);
    tf2::Transform tf_marker_in_map = tf_cam_to_map * tf_marker_in_cam;

    return tf_marker_in_map;
}
