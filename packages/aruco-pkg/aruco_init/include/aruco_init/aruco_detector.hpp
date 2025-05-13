#pragma once 

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

class ArucoDetector {
public:
    ArucoDetector();
    ~ArucoDetector();

    void detectMarkers(const cv::Mat& image);
    void drawDetectedMarkers(
        cv::Mat& image,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs,
        float markerLength,
        const std::vector<cv::Vec3d>& rvecs = {},
        const std::vector<cv::Vec3d>& tvecs = {});
    cv::Mat drawDebugImg(const cv::Mat& image);

    const std::vector<int>& getIds() const;
    const std::vector<std::vector<cv::Point2f>>& getCorners() const;
    const std::vector<std::vector<cv::Point2f>>& getRejectedCorners() const;
    
private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Mat preprocess(const cv::Mat& image);

    std::vector<int> ids_;
    std::vector<std::vector<cv::Point2f>> corners_;
    std::vector<std::vector<cv::Point2f>> rejectedCorners_;
};