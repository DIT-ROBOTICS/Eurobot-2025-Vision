#pragma once

#include <unordered_map>
#include <mutex>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>

struct ImageBuffer {
    using CvImagePtr = cv_bridge::CvImageConstPtr;
    
private:
    std::unordered_map<std::string, CvImagePtr> images_;
    mutable std::unordered_map<std::string, std::unique_ptr<std::mutex>> mutexes_;
    mutable std::mutex map_mutex_; 
    
    std::mutex& getMutex(const std::string& camera_id) const {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (mutexes_.find(camera_id) == mutexes_.end()) {
            mutexes_[camera_id] = std::make_unique<std::mutex>();
        }
        return *mutexes_[camera_id];
    }
    
public:
    void setImage(const std::string& camera_id, CvImagePtr img) {
        std::lock_guard<std::mutex> lock(getMutex(camera_id));
        images_[camera_id] = img;
    }
    
    CvImagePtr getImage(const std::string& camera_id) const {
        std::lock_guard<std::mutex> lock(getMutex(camera_id));
        auto it = images_.find(camera_id);
        return (it != images_.end()) ? it->second : nullptr;
    }
    
    void removeCamera(const std::string& camera_id) {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        std::lock_guard<std::mutex> img_lock(getMutex(camera_id));
        images_.erase(camera_id);
        mutexes_.erase(camera_id);
    }
    
    bool hasCamera(const std::string& camera_id) const {
        std::lock_guard<std::mutex> lock(getMutex(camera_id));
        return images_.find(camera_id) != images_.end();
    }
    
    std::vector<std::string> getCameraIds() const {
        std::lock_guard<std::mutex> lock(map_mutex_);
        std::vector<std::string> ids;
        for (const auto& pair : images_) {
            ids.push_back(pair.first);
        }
        return ids;
    }
    
    size_t getCameraCount() const {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return images_.size();
    }
    
    void reset() {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        std::vector<std::unique_lock<std::mutex>> locks;
        for (auto& [_, mutex_ptr] : mutexes_) {
            locks.emplace_back(*mutex_ptr);
        }
        images_.clear();
    }

    void resetCamera(const std::string& camera_id) {
        std::lock_guard<std::mutex> lock(getMutex(camera_id));
        auto it = images_.find(camera_id);
        if (it != images_.end()) {
            it->second.reset();
        }
    }
};
