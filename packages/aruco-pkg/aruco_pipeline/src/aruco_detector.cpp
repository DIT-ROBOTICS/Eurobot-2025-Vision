#include "aruco_pipeline/aruco_detector.hpp"
#include "utils/aruco_dictionary.hpp"
#include "utils/aruco_refinement.hpp"

ArucoDetector::ArucoDetector() {
    parameters_ = cv::aruco::DetectorParameters::create();
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

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

ArucoDetector::ArucoDetector(const std::string& yaml_path) {
    setParametersFromYaml(yaml_path);
}

ArucoDetector::~ArucoDetector() {
    parameters_.release();
    dictionary_.release();
}

ArucoDetector::ArucoDetector(const ArucoDetector& other) {
    if (other.parameters_) {
        parameters_ = cv::aruco::DetectorParameters::create();
        *parameters_ = *(other.parameters_);
    } else {
        parameters_ = nullptr;
    }

    dictionary_ = other.dictionary_;
    ids_ = other.ids_;
    corners_ = other.corners_;
    rejectedCorners_ = other.rejectedCorners_;
}

ArucoDetector& ArucoDetector::operator=(const ArucoDetector& other) {
    if (this != &other) {
        if (other.parameters_) {
            parameters_ = cv::aruco::DetectorParameters::create();
            *parameters_ = *(other.parameters_);
        } else {
            parameters_ = nullptr;
        }

        dictionary_ = other.dictionary_;

        ids_ = other.ids_;
        corners_ = other.corners_;
        rejectedCorners_ = other.rejectedCorners_;
    }
    return *this;
}

void ArucoDetector::setParametersFromYaml(const std::string& yaml_file_path) {
    if (!std::filesystem::exists(yaml_file_path)) {
        std::cerr << "[ArucoDetector] YAML file not found: " << yaml_file_path << std::endl;
        return;
    }

    std::ifstream file(yaml_file_path);
    if (!file.is_open()) {
        std::cerr << "[ArucoDetector] Failed to open file: " << yaml_file_path << std::endl;
        return;
    }

    std::stringstream buffer;
    buffer << "%YAML:1.0\n";
    buffer << file.rdbuf();

    cv::FileStorage fs(buffer.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
    if (!fs.isOpened()) {
        std::cerr << "[ArucoDetector] Failed to open detector yaml from memory: " << yaml_file_path << std::endl;
        parameters_ = cv::aruco::DetectorParameters::create();
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        return;
    }

    cv::FileNode node = fs["aruco_detector"];
    if (node.empty()) {
        std::cerr << "[ArucoDetector] No 'aruco_detector' node in yaml file: " << yaml_file_path << std::endl;
        parameters_ = cv::aruco::DetectorParameters::create();
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        return;
    }

    parameters_ = cv::aruco::DetectorParameters::create();

    node["adaptiveThreshWinSizeMin"] >> parameters_->adaptiveThreshWinSizeMin;
    node["adaptiveThreshWinSizeMax"] >> parameters_->adaptiveThreshWinSizeMax;
    node["adaptiveThreshWinSizeStep"] >> parameters_->adaptiveThreshWinSizeStep;
    node["adaptiveThreshConstant"] >> parameters_->adaptiveThreshConstant;

    node["minMarkerPerimeterRate"] >> parameters_->minMarkerPerimeterRate;
    node["maxMarkerPerimeterRate"] >> parameters_->maxMarkerPerimeterRate;
    node["polygonalApproxAccuracyRate"] >> parameters_->polygonalApproxAccuracyRate;

    std::string refinement_method = "SUBPIX";
    node["cornerRefinementMethod"] >> refinement_method;
    parameters_->cornerRefinementMethod = getCornerRefinementMethodByName(refinement_method);

    node["cornerRefinementWinSize"] >> parameters_->cornerRefinementWinSize;
    node["cornerRefinementMaxIterations"] >> parameters_->cornerRefinementMaxIterations;
    node["cornerRefinementMinAccuracy"] >> parameters_->cornerRefinementMinAccuracy;

    std::string dict_name = "DICT_4X4_50";
    node["dictionary"] >> dict_name;
    dictionary_ = cv::aruco::getPredefinedDictionary(getDictionaryIdByName(dict_name));

    fs.release();

    std::cout << "[ArucoDetector] Parameters loaded from " << yaml_file_path << std::endl;
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