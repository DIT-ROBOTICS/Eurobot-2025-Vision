#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <set>

#include "camera_info_handler.hpp"
#include "aruco_detector.hpp"
#include "aruco_transformer.hpp"
#include "image_buffer.hpp"
#include "thread_pool.hpp"

using Image = sensor_msgs::msg::Image;
using ImageConstPtr = sensor_msgs::msg::Image::SharedPtr;

class ArucoPipeline : public rclcpp::Node {
public:
  explicit ArucoPipeline(const std::string &node_name);
  ~ArucoPipeline() {
    RCLCPP_INFO(get_logger(), "ArucoPipeline Destructor");
    image_buffer_.reset();
    thread_pool_.reset();
    detector_.reset();
    camera_info_.reset();
    transformer_.clear();
    image_subs_.clear();
    cam_info_subs_.clear();
    image_publishers_.clear();
    tf_buffer_.reset();
    tf_listener_.reset();
    if(aruco_timer_) {
      aruco_timer_->cancel();
    }
    aruco_timer_.reset();
    RCLCPP_INFO(get_logger(), "ArucoPipeline Destructor Done");
  }
  bool initialize();

private:
  // Functions
  void setupImageTransport();
  void setupCameraInfoSubscriptions();
  void setupImageSubscriptions();
  void initTransformers();
  void timerProcessAruco();
  std::shared_ptr<ArucoTransformer> createTransformer(const std::string &cam_name);

  // Callback
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr img, const std::string& cam_name);

  // Process
  void processAruco(const cv::Mat &image, const std::string &cam_name,
                   image_transport::Publisher* pub, const std::string &tf_frame_id);
  bool validateMarkerTf(tf2::Transform &tf_marker_in_map) const;

  // ROS params
  int num_threads_;
  double marker_length_;
  bool gui_;
  std::string tf_parent_frame_id_;
  std::string image_encoding_;
  std::vector<std::string> camera_lists_;
  std::map<std::string, std::string> cam_info_topics_;
  std::map<std::string, std::string> image_pub_topics_;
  std::map<std::string, std::string> image_sub_topics_;
  std::map<std::string, std::string> tf_frame_id_;
  std::string blue_pose_topic_;
  std::string yellow_pose_topic_;
  std::string superstar_pose_topic_;
  std::string sima_pose_topic_;

  int superstar_id_;
  std::vector<long> sima_group_id_;
  
  // Make Shared
  std::shared_ptr<CameraInfoHandler> camera_info_;
  std::shared_ptr<ArucoDetector> detector_;
  std::shared_ptr<ThreadPool> thread_pool_;
  std::shared_ptr<ImageBuffer> image_buffer_;
  std::map<std::string, std::shared_ptr<ArucoTransformer>> transformer_;
  
  std::unordered_map<int, geometry_msgs::msg::Pose> sima_pose_buffer_;

  // ROS Publisher & Subscriber
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_blue_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_yellow_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_superstar_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_sima_pose_array_;
  
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> cam_info_subs_;
  std::unordered_set<std::string> received_cam_info_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;

  // Image Transport
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::map<std::string, image_transport::Publisher> image_publishers_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer 
  rclcpp::TimerBase::SharedPtr aruco_timer_;
};
