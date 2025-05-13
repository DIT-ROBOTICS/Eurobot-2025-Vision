#include "aruco_init/aruco_detector.hpp"

ArucoDetector::ArucoDetector() {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters_ = cv::aruco::DetectorParameters::create();

    // === Parameter tuning ===
    parameters_->adaptiveThreshWinSizeMin = 3; // 3 ~ 5
    parameters_->adaptiveThreshWinSizeMax = 11;
    parameters_->adaptiveThreshWinSizeStep = 8;
    parameters_->adaptiveThreshConstant = 4;

    parameters_->minMarkerPerimeterRate = 0.002; // 0.002 ~ 0.007 (dynamic to static)
    parameters_->maxMarkerPerimeterRate = 0.5; // 0.5 ~ 0.7
    parameters_->polygonalApproxAccuracyRate = 0.07; // 0.03 ~ 0.05

    parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters_->cornerRefinementWinSize = 2; // 5 ~ 7
    parameters_->cornerRefinementMaxIterations = 30;
    parameters_->cornerRefinementMinAccuracy = 0.05;
}

ArucoDetector::~ArucoDetector() {
    // Destructor if needed
}

cv::Mat ArucoDetector::preprocess(const cv::Mat& image) {
    cv::Mat gray, binary;

    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    // cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
    return gray;
}

void ArucoDetector::detectMarkers(const cv::Mat& image) {
    ids_.clear();
    corners_.clear();
    rejectedCorners_.clear();

    cv::Mat processed = preprocess(image);
    cv::aruco::detectMarkers(processed, dictionary_, corners_, ids_, parameters_, rejectedCorners_);
}

void ArucoDetector::drawDetectedMarkers(
    cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    float markerLength,
    const std::vector<cv::Vec3d>& rvecs,
    const std::vector<cv::Vec3d>& tvecs)
{
    if (!ids_.empty()) {
        cv::aruco::drawDetectedMarkers(image, corners_, ids_);
        for (size_t i = 0; i < ids_.size(); ++i) {
            cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.8f);
        }
    }
}

cv::Mat ArucoDetector::drawDebugImg(const cv::Mat& image) {
    return preprocess(image); 
}

const std::vector<int>& ArucoDetector::getIds() const {
    return ids_;
}

const std::vector<std::vector<cv::Point2f>>& ArucoDetector::getCorners() const {
    return corners_;
}

const std::vector<std::vector<cv::Point2f>>& ArucoDetector::getRejectedCorners() const {
    return rejectedCorners_;
}
