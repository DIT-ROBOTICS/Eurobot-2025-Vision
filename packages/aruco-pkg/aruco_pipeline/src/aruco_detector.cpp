#include "aruco_pipeline/aruco_detector.hpp"

ArucoDetector::ArucoDetector() {
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
    
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

ArucoDetector::ArucoDetector(const ArucoDetector& other) {
    parameters_ = cv::aruco::DetectorParameters::create();
    
    parameters_->adaptiveThreshWinSizeMin = other.parameters_->adaptiveThreshWinSizeMin;
    parameters_->adaptiveThreshWinSizeMax = other.parameters_->adaptiveThreshWinSizeMax;
    parameters_->adaptiveThreshWinSizeStep = other.parameters_->adaptiveThreshWinSizeStep;
    parameters_->adaptiveThreshConstant = other.parameters_->adaptiveThreshConstant;
    
    parameters_->minMarkerPerimeterRate = other.parameters_->minMarkerPerimeterRate;
    parameters_->maxMarkerPerimeterRate = other.parameters_->maxMarkerPerimeterRate;
    parameters_->polygonalApproxAccuracyRate = other.parameters_->polygonalApproxAccuracyRate;
    
    parameters_->cornerRefinementMethod = other.parameters_->cornerRefinementMethod;
    parameters_->cornerRefinementWinSize = other.parameters_->cornerRefinementWinSize;
    parameters_->cornerRefinementMaxIterations = other.parameters_->cornerRefinementMaxIterations;
    parameters_->cornerRefinementMinAccuracy = other.parameters_->cornerRefinementMinAccuracy;
    
    if (other.dictionary_) {
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    }
    
    ids_ = other.ids_;
    corners_ = other.corners_;
    rejectedCorners_ = other.rejectedCorners_;
}

std::shared_ptr<ArucoDetector> ArucoDetector::clone() const {
    return std::make_shared<ArucoDetector>(*this);
}

ArucoDetector::~ArucoDetector() {
    parameters_.release();
    dictionary_.release();
}

cv::Mat ArucoDetector::preprocess(const cv::Mat& image) {
    cv::Mat gray;

    if (image.empty()) {
        return cv::Mat();
    }

    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    return gray;
}

void ArucoDetector::detectMarkers(const cv::Mat& image) {
    if (image.empty()) {
        return;
    }

    ids_.clear();
    corners_.clear();
    rejectedCorners_.clear();

    try {
        if (!dictionary_) {
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        }

        cv::Mat processed = preprocess(image);
        if (!processed.empty()) {
            cv::aruco::detectMarkers(processed, dictionary_, corners_, ids_, parameters_, rejectedCorners_);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV exception in detectMarkers: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception in detectMarkers: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in detectMarkers" << std::endl;
    }
}

cv::Mat ArucoDetector::drawDebugImg(const cv::Mat& image) {
    if (image.empty()) {
        return cv::Mat();
    }
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