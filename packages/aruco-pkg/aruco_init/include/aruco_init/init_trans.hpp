#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tuple>
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "aruco_init/aruco_detector.hpp"
#include "aruco_init/thread_pool.hpp"

using sensor_msgs::msg::Image;
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;

class InitTransNode : public rclcpp::Node {
public:
  explicit InitTransNode(const rclcpp::NodeOptions &options);

  std::tuple<cv::Mat&, cv::Mat&, cv::Mat&> getCvImages();
  void initialize();

private:
  // Aruco
  ArucoDetector aruco_detector_;
  std::unique_ptr<ThreadPool> thread_pool_;

  // ImageTransport
  image_transport::Publisher publisher_left_;
  image_transport::Publisher publisher_mid_;
  image_transport::Publisher publisher_right_;

  // Pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  // Subscribers
  std::shared_ptr<message_filters::Subscriber<Image>> sub_left_, sub_mid_, sub_right_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Image data
  cv::Mat left_image_, mid_image_, right_image_;
  std::mutex mutex_left_, mutex_mid_, mutex_right_;

  // CameraInfo subscription
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_mid_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_right_;

  // Camera parameters
  cv::Mat camK_left_, camK_mid_, camK_right_;
  cv::Mat dist_left_, dist_mid_, dist_right_;
  bool cam_left_ready_ = false, cam_mid_ready_ = false, cam_right_ready_ = false;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr aruco_timer_;
  rclcpp::TimerBase::SharedPtr error_timer_;
  rclcpp::Time detected_time_;

  // Callbacks
  void processImages();
  void imageCallback(
    const ImageConstPtr &left_msg, 
    const ImageConstPtr &mid_msg, 
    const ImageConstPtr &right_msg
  );
  void getCameraInfo();

  // ArUco Processing
  void processOneImage(
    const cv::Mat &image,
    const cv::Mat &camera_matrix,
    const cv::Mat &dist_coeffs,
    image_transport::Publisher pub,
    const std::string &camera_frame,
    const std::string &label
  );

  void getSpecificIdPose(
    const std::vector<int>& ids, 
    const std::vector<cv::Vec<double, 3>>& rvecs, 
    const std::vector<cv::Vec<double, 3>>& tvecs,
    const std::string &camera_frame,
    const std::string &label
  );

  // Marker Checker
  void MarkerChecker();
};
