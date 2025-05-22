#include "aruco_pipeline/aruco_pipeline.hpp"
#include <rclcpp/rclcpp.hpp>

ArucoPipeline::ArucoPipeline(const std::string &node_name) : rclcpp::Node(node_name) {
  RCLCPP_INFO(get_logger(), "ArucoPipeline Constructor");

  // Declare parameters
  RCLCPP_INFO(get_logger(), "Declaring parameters...");
  num_threads_ = this->declare_parameter<int>("num_threads", 3);
  marker_length_ = this->declare_parameter<double>("marker_length", 0.07);
  gui_ = this->declare_parameter<bool>("gui", false);

  cam_info_topics_["left"] = this->declare_parameter<std::string>("camera_info.left", "/camera_info_left");
  cam_info_topics_["mid"] = this->declare_parameter<std::string>("camera_info.mid", "/camera_info_mid");
  cam_info_topics_["right"] = this->declare_parameter<std::string>("camera_info.right", "/camera_info_right");

  image_pub_topics_["left"] = this->declare_parameter<std::string>("image_pub.left", "/image_pub_left");
  image_pub_topics_["mid"] = this->declare_parameter<std::string>("image_pub.mid", "/image_pub_mid");
  image_pub_topics_["right"] = this->declare_parameter<std::string>("image_pub.right", "/image_pub_right");

  image_sub_topics_["left"] = this->declare_parameter<std::string>("image_sub.left", "/image_sub_left");
  image_sub_topics_["mid"] = this->declare_parameter<std::string>("image_sub.mid", "/image_sub_mid");
  image_sub_topics_["right"] = this->declare_parameter<std::string>("image_sub.right", "/image_sub_right");

  tf_frame_id_ ["left"] = this->declare_parameter<std::string>("tf_frame_id.left", "cam_left");
  tf_frame_id_ ["mid"] = this->declare_parameter<std::string>("tf_frame_id.mid", "cam_mid");
  tf_frame_id_ ["right"] = this->declare_parameter<std::string>("tf_frame_id.right", "cam_right");

  tf_parent_frame_id_ = this->declare_parameter<std::string>("tf_frame_id.parent", "map");

  sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 10);
  image_encoding_ = this->declare_parameter<std::string>("image_encoding", "bgr8");

  image_buffer_ = std::make_shared<ImageBuffer>();
  camera_info_ = std::make_shared<CameraInfoHandler>(std::set<std::string>{"left", "mid", "right"});
  detector_ = std::make_shared<ArucoDetector>();
  thread_pool_ = std::make_shared<ThreadPool>(num_threads_);

  pub_robot_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/robot_pose", 10);
  pub_rival_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/rival_pose", 10);
  RCLCPP_INFO(get_logger(),
    "Pose Publishers:\n"
    "                                                       %s\n"
    "                                                       %s",
    pub_robot_pose_->get_topic_name(),
    pub_rival_pose_->get_topic_name());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  RCLCPP_INFO(get_logger(), "TF2 Listener initialized");
}

void ArucoPipeline::setupImageTransport() {
  if (gui_) {
    RCLCPP_INFO(get_logger(), "GUI is enabled. Setting up image transport...");
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
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

void ArucoPipeline::setupCameraInfoSubscriptions(const std::map<std::string, std::string> &cam_topics) {
  RCLCPP_INFO(get_logger(), "Setting up CameraInfo Subscriptions...");
  cam_info_subs_.clear();

  for (const auto &[name, topic] : cam_topics) {
    auto sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic, 10, [this, name](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      try {
        if (camera_info_->hasAllCameras()) {
          return;
        }

        camera_info_->setCameraInfo(name, *msg);
        RCLCPP_INFO(this->get_logger(), "Received CameraInfo for %s", name.c_str());

        if (camera_info_->hasAllCameras()) {
          RCLCPP_INFO(this->get_logger(), "All CameraInfos received.");
          cam_info_subs_.clear();
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in CameraInfo callback [%s]: %s", name.c_str(), e.what());
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in CameraInfo callback [%s]", name.c_str());
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

void ArucoPipeline::setupImageSubscriptions() {
  sub_left_ = std::make_shared<message_filters::Subscriber<Image>>(
    this->shared_from_this(),
    image_sub_topics_["left"],
    rmw_qos_profile_sensor_data);

  sub_mid_ = std::make_shared<message_filters::Subscriber<Image>>(
    this->shared_from_this(),
    image_sub_topics_["mid"],
    rmw_qos_profile_sensor_data);

  sub_right_ = std::make_shared<message_filters::Subscriber<Image>>(
    this->shared_from_this(),
    image_sub_topics_["right"],
    rmw_qos_profile_sensor_data);

  SyncPolicy policy(sync_queue_size_);
  policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(policy);
  sync_->connectInput(*sub_left_, *sub_mid_, *sub_right_);

  sync_->registerCallback(std::bind(&ArucoPipeline::imageCallback, this,
                                   std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void ArucoPipeline::imageCallback(const ImageConstPtr &left_img, const ImageConstPtr &mid_img, const ImageConstPtr &right_img) {
  // RCLCPP_INFO(get_logger(), "Received images: left, mid, right");
  if (!left_img || !mid_img || !right_img) {
    RCLCPP_WARN(get_logger(), "Received null image(s), skipping processing");
    return;
  }
  try {
    auto left_cv_ptr = cv_bridge::toCvShare(left_img, image_encoding_);
    auto mid_cv_ptr = cv_bridge::toCvShare(mid_img, image_encoding_);
    auto right_cv_ptr = cv_bridge::toCvShare(right_img, image_encoding_);
    
    // Buffer to load images
    image_buffer_->setLeft(left_cv_ptr);
    image_buffer_->setMid(mid_cv_ptr);
    image_buffer_->setRight(right_cv_ptr);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void ArucoPipeline::timerProcessAruco() {
  auto left = image_buffer_->getLeft();
  auto mid = image_buffer_->getMid();
  auto right = image_buffer_->getRight();

  if (!left || !mid || !right || left->image.empty() || mid->image.empty() || right->image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Image buffer is empty, skipping processing");
    return;
  }

  thread_pool_->enqueue([this, img = left]() {
    this->processAruco(img->image, "left", pub_left_, tf_frame_id_["left"]);
  });
  thread_pool_->enqueue([this, img = mid]() {
    this->processAruco(img->image, "mid", pub_mid_, tf_frame_id_["mid"]);
  });
  thread_pool_->enqueue([this, img = right]() {
    this->processAruco(img->image, "right", pub_right_, tf_frame_id_["right"]);
  });
}

bool ArucoPipeline::initialize() {
  setupImageTransport();
  setupCameraInfoSubscriptions(cam_info_topics_);

  auto start = std::chrono::steady_clock::now();
  const int timeout_sec = 5; 

  while (rclcpp::ok() && !camera_info_->hasAllCameras()) {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(33));

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
    if (elapsed > timeout_sec) {
      RCLCPP_ERROR(this->get_logger(), "Timeout: Not all CameraInfo topics received after %d seconds!", timeout_sec);
      break;
    }
  }

  if (!camera_info_->hasAllCameras()) {
    RCLCPP_ERROR(this->get_logger(), "Initialization failed: CameraInfo not complete.");
    return false;
  }

  initTransformers();
  RCLCPP_INFO(this->get_logger(), "Transformers initialized.");
  setupImageSubscriptions();
  RCLCPP_INFO(this->get_logger(), "Image subscriptions set up.");
  
  aruco_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(33),
    std::bind(&ArucoPipeline::timerProcessAruco, this));
  
  return true;
}

void ArucoPipeline::processAruco(const cv::Mat &image, const std::string &cam_name,
                                 image_transport::Publisher &pub, const std::string &tf_frame_id)
{
  thread_local static auto local_detector = detector_->clone();
  
  auto it = transformer_.find(cam_name);
  
  local_detector->detectMarkers(image);
  const auto &ids = local_detector->getIds();
  if (ids.empty()) { return; }
  it->second->estimatePose(local_detector->getCorners());

  if (gui_ && pub) {
    cv::Mat display_img = image.clone();
    it->second->drawMarkers(display_img, ids, local_detector->getCorners());
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), image_encoding_, display_img).toImageMsg();
    msg->header.stamp = this->now();
    pub.publish(*msg);
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform(tf_parent_frame_id_, tf_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not lookup transform from %s to map: %s", tf_frame_id.c_str(), ex.what());
    return;
  }

  tf2::Transform tf_cam_to_map;
  tf2::fromMsg(transform.transform, tf_cam_to_map);
  
  auto &rvecs = it->second->getRvecs();
  auto &tvecs = it->second->getTvecs();
  for (size_t i = 0; i < ids.size(); ++i) {
    int id = ids[i];
    const auto &rvec = rvecs[i];
    const auto &tvec = tvecs[i];
    tf2::Transform tf_marker_in_map = it->second->getMarkerInMapTf(rvec, tvec, tf_cam_to_map);
    if (!validateMarkerTf(tf_marker_in_map)) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = tf_parent_frame_id_;
    tf2::toMsg(tf_marker_in_map, pose_msg.pose);

    if (id >= 1 && id <= 5) {
      pub_robot_pose_->publish(pose_msg);
    } else if (id >= 6 && id <= 10) {
      pub_rival_pose_->publish(pose_msg);
    }
  }
}

bool ArucoPipeline::validateMarkerTf(tf2::Transform &tf_marker_in_map) const {
  tf2::Vector3 marker_z_axis = tf_marker_in_map.getBasis().getColumn(2);
  double z_alignment = marker_z_axis.dot(tf2::Vector3(0, 0, 1));
  if (z_alignment <= 0.8) {
    return false;
  }

  double roll, pitch, yaw;
  tf_marker_in_map.getBasis().getRPY(roll, pitch, yaw);

  tf2::Quaternion q_flat;
  q_flat.setRPY(0, 0, yaw);
  q_flat.normalize();
  tf_marker_in_map.setRotation(q_flat);

  tf2::Vector3 pos = tf_marker_in_map.getOrigin();
  pos.setZ(0.0);
  tf_marker_in_map.setOrigin(pos);

  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArucoPipeline>("aruco_pipeline");

  if (!node->initialize()) {
    RCLCPP_FATAL(node->get_logger(), "ArucoPipeline failed to initialize. Exiting.");
    rclcpp::shutdown();
    return 1;
  } else {
    RCLCPP_INFO(node->get_logger(), "ArucoPipeline initialized successfully.");
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}