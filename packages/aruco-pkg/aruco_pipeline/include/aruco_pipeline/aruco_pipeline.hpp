#pragma once
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "camera_info_handler.hpp"
#include "image_buffer.hpp"
#include "aruco_detector.hpp"
#include "aruco_transformer.hpp"
// #include "transform_validator.hpp"
#include "thread_pool.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArucoPipeline : public rclcpp_lifecycle::LifecycleNode {
public:
  ArucoPipeline(const std::string & node_name, size_t num_threads);

  // Lifecycle callback
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void processAruco(const cv::Mat &color, const std::string& cam_name, 
                    image_transport::Publisher &pub);

  std::shared_ptr<rclcpp::Node> get_helper_node() const { return helper_node_; }
  
private:
  rclcpp::Node::SharedPtr helper_node_;
  using Image = sensor_msgs::msg::Image;
  using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
    
  void setupImageTransport();
  void setupImageSubscriptions();
  void setupCameraInfoSubscriptions(const std::map<std::string, std::string>& cam_info_topics);
  void imageCallback(const ImageConstPtr &left, const ImageConstPtr &mid, const ImageConstPtr &right);
  void timerProcessAruco();
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string& camera_name);
  
  void initTransformers();
  std::shared_ptr<ArucoTransformer> createTransformer(const std::string& cam_name);

  // ROS params
  size_t num_threads_;
  double marker_length_;
  bool gui_{false};
  std::map<std::string, std::string> cam_info_topics_;
  std::map<std::string, std::string> image_pub_topics_;
  std::map<std::string, std::string> image_sub_topics_;
  int sync_queue_size_;
  std::string image_encoding_;

  std::shared_ptr<ImageBuffer> image_buffer_;
  std::shared_ptr<CameraInfoHandler> camera_info_;
  std::shared_ptr<ArucoDetector> detector_;
  // std::shared_ptr<ArucoTransformer> transformer_;
  std::map<std::string, std::shared_ptr<ArucoTransformer>> transformer_;
  // std::shared_ptr<TransformValidator> validator_;
  std::shared_ptr<ThreadPool> thread_pool_;

  // Pose publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_robot_pose_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_rival_pose_;

  // ImageTransport
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher pub_left_;
  image_transport::Publisher pub_mid_;
  image_transport::Publisher pub_right_;
  
  // Subscribers
  std::shared_ptr<message_filters::Subscriber<Image>> sub_left_;
  std::shared_ptr<message_filters::Subscriber<Image>> sub_mid_;
  std::shared_ptr<message_filters::Subscriber<Image>> sub_right_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // CameraInfoHandler 
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> cam_info_subs_;
  std::set<std::string> received_cams_;

  // TransformerMap
  std::map<std::string, std::shared_ptr<ArucoTransformer>> transformer_map_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr aruco_timer_;
};
