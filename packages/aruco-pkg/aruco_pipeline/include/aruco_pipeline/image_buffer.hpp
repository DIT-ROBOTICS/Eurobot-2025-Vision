#pragma once

#include <mutex>
#include <cv_bridge/cv_bridge.h>

struct ImageBuffer
{
  using CvImagePtr = cv_bridge::CvImageConstPtr;

  CvImagePtr left;
  CvImagePtr mid;
  CvImagePtr right;

  // mutex lock
  mutable std::mutex mutex_left;
  mutable std::mutex mutex_mid;
  mutable std::mutex mutex_right;

  // set Image
  void setLeft(CvImagePtr img) {
    std::lock_guard<std::mutex> lock(mutex_left);
    left = img;
  }

  void setMid(CvImagePtr img) {
    std::lock_guard<std::mutex> lock(mutex_mid);
    mid = img;
  }

  void setRight(CvImagePtr img) {
    std::lock_guard<std::mutex> lock(mutex_right);
    right = img;
  }

  // get Image
  CvImagePtr getLeft() const {
    std::lock_guard<std::mutex> lock(mutex_left);
    return left;
  }

  CvImagePtr getMid() const {
    std::lock_guard<std::mutex> lock(mutex_mid);
    return mid;
  }

  CvImagePtr getRight() const {
    std::lock_guard<std::mutex> lock(mutex_right);
    return right;
  }

  // reset all images
  void reset() {
    std::scoped_lock lock(mutex_left, mutex_mid, mutex_right);
    left.reset();
    mid.reset();
    right.reset();
  }
};
