#include "aruco_pipeline/aruco_pipeline.hpp"
#include <rclcpp/rclcpp.hpp>

ArucoPipeline::ArucoPipeline(const std::string &node_name) : rclcpp::Node(node_name) {
  RCLCPP_INFO(get_logger(), "ArucoPipeline Constructor");

  // Declare parameters
  RCLCPP_INFO(get_logger(), "Declaring parameters...");
  num_threads_ = this->declare_parameter<int>("num_threads", 3);
  marker_length_ = this->declare_parameter<double>("marker_length", 0.07);
  gui_ = this->declare_parameter<bool>("gui", false);

  image_encoding_ = this->declare_parameter<std::string>("image_encoding", "bgr8");
  tf_parent_frame_id_ = this->declare_parameter<std::string>("tf_frame_id.parent", "map");

  camera_lists_ = this->declare_parameter<std::vector<std::string>>("camera_lists", std::vector<std::string>{"left", "mid", "right"});

  if (camera_lists_.empty()) {
    RCLCPP_ERROR(get_logger(), "No cameras specified in camera_lists parameter!");
    throw std::runtime_error("No cameras specified");
  }

  RCLCPP_INFO(get_logger(), "Configured cameras:");
  for (const auto& name : camera_lists_) {
    RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
  }

  // Camera Lists setup 
  for (const auto& cam_name : camera_lists_) {
    cam_info_topics_[cam_name] = this->declare_parameter<std::string>(
      "camera_info." + cam_name, "/camera_info_" + cam_name);
    
    image_pub_topics_[cam_name] = this->declare_parameter<std::string>(
      "image_pub." + cam_name, "/image_pub_" + cam_name);
    
    image_sub_topics_[cam_name] = this->declare_parameter<std::string>(
      "image_sub." + cam_name, "/image_sub_" + cam_name);
    
    tf_frame_id_[cam_name] = this->declare_parameter<std::string>(
      "tf_frame_id." + cam_name, "cam_" + cam_name);
  }

  // Make Shared
  image_buffer_ = std::make_shared<ImageBuffer>();
  camera_info_ = std::make_shared<CameraInfoHandler>(std::set<std::string>(camera_lists_.begin(), camera_lists_.end()));
  detector_ = std::make_shared<ArucoDetector>();
  thread_pool_ = std::make_shared<ThreadPool>(num_threads_);

  // Pose Publisher
  pub_blue_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/blue_pose", 10);
  pub_yellow_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/yellow_pose", 10);
  RCLCPP_INFO(get_logger(), "Pose Publishers:");
  RCLCPP_INFO(get_logger(), "  - %s", pub_blue_pose_->get_topic_name());
  RCLCPP_INFO(get_logger(), "  - %s", pub_yellow_pose_->get_topic_name());

  // TF initialize
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  RCLCPP_INFO(get_logger(), "TF2 Listener initialized");

  // ArucoDetector Params
  declare_parameter<std::string>("detector_config", "");
  auto path = get_parameter("detector_config").as_string();
  if (!path.empty()) {
      detector_->setParametersFromYaml(path);
  } else {
      RCLCPP_WARN(this->get_logger(), "detector_config parameter is empty, using default parameters.");
  }
}

// Image Publisher setup
void ArucoPipeline::setupImageTransport() {
  if (gui_) {
    RCLCPP_INFO(get_logger(), "GUI is enabled. Setting up image transport...");
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    for (const auto& cam_name : camera_lists_) {
      image_publishers_[cam_name] = it_->advertise(image_pub_topics_[cam_name], 10);
    }
  } else {
    RCLCPP_INFO(get_logger(), "GUI is disabled. No image transport setup.");
    it_ = nullptr;
    image_publishers_.clear();
  }
}

void ArucoPipeline::setupCameraInfoSubscriptions() {
  RCLCPP_INFO(get_logger(), "Setting up CameraInfo Subscriptions...");
  cam_info_subs_.clear();
  received_cam_info_.clear();

  for (const auto& cam_name : camera_lists_) {
    const std::string& topic = cam_info_topics_[cam_name];
    
    auto sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic, 10, [this, cam_name](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        try {
          if (camera_info_->hasAllCameras()) {
            return;
          }

          if (received_cam_info_.count(cam_name) == 0) {
            RCLCPP_INFO(this->get_logger(), "Received CameraInfo for %s", cam_name.c_str());
            received_cam_info_.insert(cam_name);
          }

          camera_info_->setCameraInfo(cam_name, *msg);

          if (camera_info_->hasAllCameras()) {
            RCLCPP_INFO(this->get_logger(), "All CameraInfos received.");
            cam_info_subs_.clear();
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Exception in CameraInfo callback [%s]: %s", cam_name.c_str(), e.what());
        } catch (...) {
          RCLCPP_ERROR(this->get_logger(), "Unknown exception in CameraInfo callback [%s]", cam_name.c_str());
        }
      });
    cam_info_subs_.push_back(sub);
  }
}

void ArucoPipeline::initTransformers() {
  transformer_.clear();
  for (const auto& cam_name : camera_lists_) {
    transformer_[cam_name] = createTransformer(cam_name);
  }
}

std::shared_ptr<ArucoTransformer> ArucoPipeline::createTransformer(const std::string &cam_name) {
  cv::Mat K = camera_info_->getCameraMatrix(cam_name);
  cv::Mat D = camera_info_->getDistCoeffs(cam_name);
  return std::make_shared<ArucoTransformer>(K, D, marker_length_);
}

void ArucoPipeline::setupImageSubscriptions() {
  RCLCPP_INFO(get_logger(), "Setting up Image Subscriptions...");
  image_subs_.clear();

  for (const auto& cam_name : camera_lists_) {
    const std::string& topic = image_sub_topics_[cam_name];
    
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
      topic, 
      rclcpp::SensorDataQoS(),
      [this, cam_name](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->imageCallback(msg, cam_name);
      });
    
    image_subs_[cam_name] = sub;
    RCLCPP_INFO(get_logger(), "Subscribed to Image: %s → %s", cam_name.c_str(), topic.c_str());
  }
}

void ArucoPipeline::imageCallback(const sensor_msgs::msg::Image::SharedPtr img, const std::string& cam_name) {
  if (!img) {
    RCLCPP_WARN(get_logger(), "Received null image from %s, skipping processing", cam_name.c_str());
    return;
  }

  try {
    auto cv_ptr = cv_bridge::toCvShare(img, image_encoding_);
    image_buffer_->setImage(cam_name, cv_ptr);
  } catch(const std::exception& e){
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void ArucoPipeline::timerProcessAruco() {
  for (const auto& cam_name : camera_lists_) {
    if (!image_buffer_->hasCamera(cam_name)) {
      continue;
    }
    auto img = image_buffer_->getImage(cam_name);
    thread_pool_->enqueue(
      [this, img, cam_name]() {
        auto it = transformer_.find(cam_name);
        if (it != transformer_.end()) {
          processAruco(img->image, cam_name, &image_publishers_[cam_name], tf_frame_id_[cam_name]);
        } else {
          RCLCPP_ERROR(get_logger(), "Transformer not found for camera: %s", cam_name.c_str());
        }
      });
  }
}

bool ArucoPipeline::initialize() {
  setupImageTransport();
  setupCameraInfoSubscriptions();

  auto start = std::chrono::steady_clock::now();
  const int timeout_sec = 10;

  RCLCPP_INFO(get_logger(), "Waiting for CameraInfo from %zu cameras...", camera_lists_.size());
  
  while (rclcpp::ok() && !camera_info_->hasAllCameras()) {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

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
  
  RCLCPP_INFO(this->get_logger(), "ArucoPipeline initialization completed successfully!");
  return true;
}

void ArucoPipeline::processAruco(const cv::Mat &image, const std::string &cam_name,
                                 image_transport::Publisher* pub, const std::string &tf_frame_id)
{
  thread_local static auto local_detector = detector_->clone();
  
  auto it = transformer_.find(cam_name);
  
  local_detector->detectMarkers(image);
  const auto &ids = local_detector->getIds();
  if (!ids.empty()){
    it->second->estimatePose(local_detector->getCorners());
  }

  // GUI display
  if (gui_ && pub) {
    cv::Mat display_img = image.clone();
    if (!ids.empty()){
      it->second->drawMarkers(display_img, ids, local_detector->getCorners());
    }
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), image_encoding_, display_img).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = tf_frame_id;
    pub->publish(*msg);
  }

  if (ids.empty()) { return; }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(tf_parent_frame_id_, tf_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                         "Could not lookup transform from %s to %s: %s", 
                         tf_frame_id.c_str(), tf_parent_frame_id_.c_str(), ex.what());
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
      pub_blue_pose_->publish(pose_msg);
    } else if (id >= 6 && id <= 10) {
      pub_yellow_pose_->publish(pose_msg);
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
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}