#include "aruco_pipeline/aruco_pipeline.hpp"
#include <rclcpp/rclcpp.hpp>

ArucoPipeline::ArucoPipeline(const std::string &node_name, size_t num_threads)
    : rclcpp_lifecycle::LifecycleNode(node_name), num_threads_(num_threads) {
  RCLCPP_INFO(get_logger(), "ArucoPipeline Constructor");
}

CallbackReturn ArucoPipeline::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring...");
  helper_node_ = std::make_shared<rclcpp::Node>("aruco_helper_node");

  // Declare parameters
  num_threads_ = this->declare_parameter<int>("num_threads", 3);
  marker_length_ = this->declare_parameter<double>("marker_length", 0.07);
  gui_ = this->declare_parameter<bool>("gui", false);
  
  cam_info_topics_["left"] = this->declare_parameter<std::string>("camera_info.left", "/cam_info_left");
  cam_info_topics_["mid"] = this->declare_parameter<std::string>("camera_info.mid", "/cam_info_mid");
  cam_info_topics_["right"] = this->declare_parameter<std::string>("camera_info.right", "/cam_info_right");

  image_pub_topics_["left"] = this->declare_parameter<std::string>("image_pub.left", "/image_pub_left");
  image_pub_topics_["mid"] = this->declare_parameter<std::string>("image_pub.mid", "/image_pub_mid");
  image_pub_topics_["right"] = this->declare_parameter<std::string>("image_pub.right", "/image_pub_right");

  image_sub_topics_["left"] = this->declare_parameter<std::string>("image_sub.left", "/image_sub_left");
  image_sub_topics_["mid"] = this->declare_parameter<std::string>("image_sub.mid", "/image_sub_mid");
  image_sub_topics_["right"] = this->declare_parameter<std::string>("image_sub.right", "/image_sub_right");

  sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 10);
  image_encoding_ = this->declare_parameter<std::string>("image_encoding", "bgr8");

  image_buffer_ = std::make_shared<ImageBuffer>();
  camera_info_ = std::make_shared<CameraInfoHandler>(std::set<std::string>{"left", "mid", "right"});
  detector_ = std::make_shared<ArucoDetector>();
  thread_pool_ = std::make_shared<ThreadPool>(num_threads_);

  pub_robot_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/robot_pose", 10);
  pub_rival_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/rival_pose", 10);

  setupImageTransport();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  setupCameraInfoSubscriptions(cam_info_topics_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArucoPipeline::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating...");

  pub_robot_pose_->on_activate();
  pub_rival_pose_->on_activate();

  if (camera_info_->hasAllCameras()) {
    setupImageSubscriptions();
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot activate: Camera info incomplete!");
    return CallbackReturn::FAILURE;
  }

  aruco_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                        std::bind(&ArucoPipeline::timerProcessAruco, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArucoPipeline::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating...");
  pub_robot_pose_->on_deactivate();
  pub_rival_pose_->on_deactivate();

  sub_left_.reset();
  sub_mid_.reset();
  sub_right_.reset();
  sync_.reset();

  if (aruco_timer_) {
    aruco_timer_->cancel();
    aruco_timer_.reset();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArucoPipeline::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up...");

  image_buffer_.reset();
  camera_info_.reset();
  detector_.reset();
  transformer_.clear();
  thread_pool_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  it_.reset();
  pub_left_ = image_transport::Publisher();
  pub_mid_ = image_transport::Publisher();
  pub_right_ = image_transport::Publisher();
  cam_info_subs_.clear();
  sub_left_.reset();
  sub_mid_.reset();
  sub_right_.reset();
  sync_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArucoPipeline::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down...");
  
  image_buffer_.reset();
  camera_info_.reset();
  detector_.reset();
  transformer_.clear();
  thread_pool_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  it_.reset();
  pub_left_ = image_transport::Publisher();
  pub_mid_ = image_transport::Publisher();
  pub_right_ = image_transport::Publisher();
  cam_info_subs_.clear();
  sub_left_.reset();
  sub_mid_.reset();
  sub_right_.reset();
  sync_.reset();

  return CallbackReturn::SUCCESS;
}

void ArucoPipeline::setupImageTransport() {
  if (gui_) {
    RCLCPP_INFO(get_logger(), "GUI is enabled. Setting up image transport...");
    it_ = std::make_shared<image_transport::ImageTransport>(helper_node_);
    pub_left_ = it_->advertise(image_pub_topics_["left"], 10);
    pub_mid_ = it_->advertise(image_pub_topics_["mid"], 10);
    pub_right_ = it_->advertise(image_pub_topics_["right"], 10);
  } else {
    RCLCPP_INFO(get_logger(), "GUI is disabled. No image transport setup.");
    it_ = nullptr;
    pub_left_ = image_transport::Publisher();  
    pub_mid_ = image_transport::Publisher();
    pub_right_ = image_transport::Publisher();
  }
}

void ArucoPipeline::setupImageSubscriptions() {
  sub_left_ = std::make_shared<message_filters::Subscriber<Image>>(
    helper_node_,
    image_sub_topics_["left"],
    rmw_qos_profile_sensor_data);
  
  sub_mid_ = std::make_shared<message_filters::Subscriber<Image>>(
    helper_node_,
    image_sub_topics_["mid"],
    rmw_qos_profile_sensor_data);
  
  sub_right_ = std::make_shared<message_filters::Subscriber<Image>>(
    helper_node_,
    image_sub_topics_["right"],
    rmw_qos_profile_sensor_data);

  SyncPolicy policy(sync_queue_size_);
  policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(policy);
  sync_->connectInput(*sub_left_, *sub_mid_, *sub_right_);

  sync_->registerCallback(std::bind(&ArucoPipeline::imageCallback, this,
                                   std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void ArucoPipeline::setupCameraInfoSubscriptions(const std::map<std::string, std::string> &cam_topics) {
  RCLCPP_INFO(get_logger(), "Setting up camera info subscriptions...");
  cam_info_subs_.clear();

  for (const auto &[name, topic] : cam_topics) {
    auto sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topic, 10, [this, name](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          if (camera_info_->hasAllCameras()) {
            return;
          }

          camera_info_->setCameraInfo(name, *msg);
          RCLCPP_INFO(this->get_logger(), "Received CameraInfo for %s", name.c_str());
          if (camera_info_->hasAllCameras()) {
            RCLCPP_INFO(this->get_logger(), "All CameraInfos received. Initializing transformers...");
            cam_info_subs_.clear();
            initTransformers();
            RCLCPP_INFO(this->get_logger(), "Transformers initialized.");
          }
        });
    cam_info_subs_.push_back(sub);
  }
}

void ArucoPipeline::initTransformers() {
  transformer_.clear();
  for (const auto &cam_name : {"left", "mid", "right"}) {
    transformer_[cam_name] = createTransformer(cam_name);
  }
}

std::shared_ptr<ArucoTransformer> ArucoPipeline::createTransformer(const std::string &cam_name) {
  cv::Mat K = camera_info_->getCameraMatrix(cam_name);
  cv::Mat D = camera_info_->getDistCoeffs(cam_name);
  return std::make_shared<ArucoTransformer>(K, D, marker_length_);
}

void ArucoPipeline::imageCallback(const ImageConstPtr &left_img, const ImageConstPtr &mid_img, const ImageConstPtr &right_img) {
  RCLCPP_INFO(get_logger(), "Image callback triggered");
  if (get_current_state().label() != "active") {
    return;
  }

  if (!left_img || !mid_img || !right_img) {
    RCLCPP_WARN(get_logger(), "Received null image(s), skipping processing");
    return;
  }

  auto left_cv_ptr = cv_bridge::toCvShare(left_img, image_encoding_);
  auto mid_cv_ptr = cv_bridge::toCvShare(mid_img, image_encoding_);
  auto right_cv_ptr = cv_bridge::toCvShare(right_img, image_encoding_);

  image_buffer_->setLeft(left_cv_ptr);
  image_buffer_->setMid(mid_cv_ptr);
  image_buffer_->setRight(right_cv_ptr);
}

void ArucoPipeline::timerProcessAruco() {
  if (this->get_current_state().label() != "active") {
    return;
  }

  auto left = image_buffer_->getLeft();
  auto mid = image_buffer_->getMid();
  auto right = image_buffer_->getRight();

  thread_pool_->enqueue([this, img = left]() {
    if (img && !img->image.empty()) {
      this->processAruco(img->image, "left", pub_left_);
    }
  });
  thread_pool_->enqueue([this, img = mid]() {
    if (img && !img->image.empty()) {
      this->processAruco(img->image, "mid", pub_mid_);
    }
  });
  thread_pool_->enqueue([this, img = right]() {
    if (img && !img->image.empty()) {
      this->processAruco(img->image, "right", pub_right_);
    }
  });
}

void ArucoPipeline::processAruco(const cv::Mat &image, const std::string &cam_name, 
                                 image_transport::Publisher &pub) 
{
  if (pub.getTopic().empty()) {
    RCLCPP_WARN(get_logger(), "Publisher for %s is invalid, skipping publish", cam_name.c_str());
    return;
  }
  
  if (!camera_info_->hasAllCameras()) {
    RCLCPP_WARN(this->get_logger(), "Not all camera infos ready yet");
    return;
  }

  auto it = transformer_.find(cam_name);
  if (it == transformer_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Transformer not found for camera: %s", cam_name.c_str());
    return;
  }

  detector_->detectMarkers(image);
  it->second->estimatePose(detector_->getCorners());

  if (gui_ && pub) {
    cv::Mat image_copy = image.clone();
    it->second->drawMarkers(image_copy, detector_->getIds(), detector_->getCorners());
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), image_encoding_, image_copy).toImageMsg();
    pub.publish(*msg);
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", cam_name, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not lookup transform from %s to map: %s", cam_name.c_str(), ex.what());
    return;
  }
  tf2::Transform tf_cam_to_map;
  tf2::fromMsg(transform.transform, tf_cam_to_map);
  auto ids = detector_->getIds();
  auto &rvecs = it->second->getRvecs();
  auto &tvecs = it->second->getTvecs();
  for (size_t i = 0; i < ids.size(); ++i) {
    int id = ids[i];
    const auto &rvec = rvecs[i];
    const auto &tvec = tvecs[i];
    tf2::Transform tf_marker_in_map = it->second->getMarkerInMapTf(rvec, tvec, tf_cam_to_map);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    tf2::toMsg(tf_marker_in_map, pose_msg.pose);

    if (id >= 1 && id <= 5) {
      pub_robot_pose_->publish(pose_msg);
    } else if (id >= 6 && id <= 10) {
      pub_rival_pose_->publish(pose_msg);
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArucoPipeline>("aruco_pipeline", 3);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();
  rclcpp::shutdown();
  return 0;
}