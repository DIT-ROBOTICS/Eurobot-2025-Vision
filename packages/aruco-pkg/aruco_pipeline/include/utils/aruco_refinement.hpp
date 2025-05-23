#pragma once
#include <opencv2/aruco.hpp>
#include <unordered_map>
#include <string>
#include <iostream>

inline int getCornerRefinementMethodByName(const std::string& name) {
    static const std::unordered_map<std::string, int> method_map = {
        {"NONE", cv::aruco::CORNER_REFINE_NONE},
        {"SUBPIX", cv::aruco::CORNER_REFINE_SUBPIX},
        {"CONTOUR", cv::aruco::CORNER_REFINE_CONTOUR},
        {"APPROX", cv::aruco::CORNER_REFINE_APRILTAG},
    };

    auto it = method_map.find(name);
    if (it != method_map.end()) {
        return it->second;
    }

    std::cerr << "[ArucoDetector] Unknown corner refinement method: " << name
              << ", defaulting to SUBPIX\n";
    return cv::aruco::CORNER_REFINE_SUBPIX;
}
