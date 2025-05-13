#include "aruco_init/init_trans.hpp"
#include "aruco_init/aruco_detector.hpp"
#include "aruco_init/aruco_transformer.hpp"

InitTransNode::InitTransNode(const rclcpp::NodeOptions &options)
  : Node("init_trans", options), 
    aruco_detector_()
{
  // Pose publisher
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose", 10);
  // Subscribers
  sub_left_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/vision/cam_left/color/image_raw");
  sub_mid_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/vision/cam_mid/color/image_raw");
  sub_right_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/vision/cam_right/color/image_raw");

  // Message filters Synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(10);
  sync_->connectInput(*sub_left_, *sub_mid_, *sub_right_);
  
  sync_->registerCallback(
    std::bind(&InitTransNode::imageCallback, this,
              std::placeholders::_1, 
              std::placeholders::_2, 
              std::placeholders::_3));
  
  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // ArucoDetector timer & thread pool
  aruco_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&InitTransNode::processImages, this));

  error_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&InitTransNode::MarkerChecker, this));
  
  thread_pool_ = std::make_unique<ThreadPool>(3);

  RCLCPP_INFO(this->get_logger(), "InitTransNode initialized");
}

void InitTransNode::initialize() {
  // Publishers (include CompressedImage)
  image_transport::ImageTransport it_(shared_from_this());
  publisher_left_ = it_.advertise("/vision/processed_left_image", 10);
  publisher_mid_ = it_.advertise("/vision/processed_mid_image", 10);
  publisher_right_ = it_.advertise("/vision/processed_right_image", 10);
  // Camera parameters
  this->getCameraInfo();
}

void InitTransNode::getCameraInfo() {
  cam_info_sub_left_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/vision/cam_left/color/camera_info", 10,
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      camK_left_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
      dist_left_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
      cam_left_ready_ = true;
      RCLCPP_INFO(this->get_logger(), "Left camera info received.");
    });
  
  cam_info_sub_mid_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/vision/cam_mid/color/camera_info", 10,
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      camK_mid_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
      dist_mid_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
      cam_mid_ready_ = true;
      RCLCPP_INFO(this->get_logger(), "Mid camera info received.");
    });
  
  cam_info_sub_right_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/vision/cam_right/color/camera_info", 10,
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      camK_right_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
      dist_right_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
      cam_right_ready_ = true;
      RCLCPP_INFO(this->get_logger(), "Right camera info received.");
    });

    rclcpp::Rate rate(10); 
    int retry = 100;
    while (rclcpp::ok() && retry-- > 0) {
      if (cam_left_ready_ && cam_mid_ready_ && cam_right_ready_) {
        RCLCPP_INFO(this->get_logger(), "All camera infos received!");
        break;
      }
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
  
    if (!cam_left_ready_ || !cam_mid_ready_ || !cam_right_ready_) {
      RCLCPP_WARN(this->get_logger(), "Timeout: not all camera info received.");
    }
  
    cam_info_sub_left_.reset();
    cam_info_sub_mid_.reset();
    cam_info_sub_right_.reset();
}

void InitTransNode::imageCallback(
  const ImageConstPtr &left_msg, 
  const ImageConstPtr &mid_msg, 
  const ImageConstPtr &right_msg) 
{
  try {
    cv::Mat left_cv_img = cv_bridge::toCvShare(left_msg, "bgr8")->image;
    cv::Mat mid_cv_img = cv_bridge::toCvShare(mid_msg, "bgr8")->image;
    cv::Mat right_cv_img = cv_bridge::toCvShare(right_msg, "bgr8")->image;

    // Lock the mutex to update the image data
    {
      std::scoped_lock lock(mutex_left_, mutex_mid_, mutex_right_);
      left_image_ = left_cv_img.clone();
      mid_image_ = mid_cv_img.clone();
      right_image_ = right_cv_img.clone();
    }
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

std::tuple<cv::Mat&, cv::Mat&, cv::Mat&> InitTransNode::getCvImages() {
  std::scoped_lock lock(mutex_left_, mutex_mid_, mutex_right_);
  return {left_image_, mid_image_, right_image_};
}

void InitTransNode::processOneImage(
  const cv::Mat &image,
  const cv::Mat &camera_matrix,
  const cv::Mat &dist_coeffs,
  image_transport::Publisher pub,
  const std::string &camera_frame,
  const std::string &label)
{
  if (image.empty()) return;

  static thread_local ArucoDetector detector;
  static thread_local ArucoTransformer transformer(camera_matrix, dist_coeffs, 0.07f);

  cv::Mat img_copy = image.clone();

  detector.detectMarkers(img_copy);

  const auto &ids = detector.getIds();
  const auto &corners = detector.getCorners();

  transformer.estimatePose(corners);
  detector.drawDetectedMarkers(
    img_copy,
    camera_matrix,
    dist_coeffs,
    0.07f,
    transformer.getRvecs(),
    transformer.getTvecs()
  );
  cv::Mat debug = detector.drawDebugImg(img_copy);
  getSpecificIdPose(ids, transformer.getRvecs(), transformer.getTvecs(), camera_frame, label);

  RCLCPP_DEBUG(this->get_logger(), "[%s] Detected %lu markers", label.c_str(), ids.size());
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", debug).toImageMsg();
  pub.publish(*msg);
}

void InitTransNode::processImages() {
  if (!cam_left_ready_ || !cam_mid_ready_ || !cam_right_ready_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for all camera_info...");
    return;
  }

  auto [left, mid, right] = getCvImages();

  if (left.empty() || mid.empty() || right.empty()) return;

  thread_pool_->enqueue([=]() {
    this->processOneImage(left, camK_left_, dist_left_, publisher_left_, "cam_left_color_optical_frame", "LEFT");
  });
  thread_pool_->enqueue([=]() {
    this->processOneImage(mid, camK_mid_, dist_mid_, publisher_mid_, "cam_mid_color_optical_frame", "MID");
  });
  thread_pool_->enqueue([=]() {
    this->processOneImage(right, camK_right_, dist_right_, publisher_right_, "cam_right_color_optical_frame", "RIGHT");
  });
}

void InitTransNode::getSpecificIdPose(
  const std::vector<int>& ids, 
  const std::vector<cv::Vec<double, 3>>& rvecs, 
  const std::vector<cv::Vec<double, 3>>& tvecs,
  const std::string &camera_frame,
  const std::string &label)
{
  auto it = std::find(ids.begin(), ids.end(), 6);
  if (it == ids.end()) {
    this->MarkerChecker();
    return;
  }

  size_t index = std::distance(ids.begin(), it);  
  const auto& rvec = rvecs[index];
  const auto& tvec = tvecs[index];

  detected_time_ = rclcpp::Clock().now();

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", camera_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "[%s] Could not lookup transform from %s to map: %s",
                label.c_str(), camera_frame.c_str(), ex.what());
    return;
  }

  // --- marker in camera ---
  tf2::Transform tf_marker_in_cam;
  tf2::Vector3 marker_translation(tvec[0], tvec[1], tvec[2]);

  tf2::Matrix3x3 tf_rot;
  cv::Mat rot_matrix;
  cv::Rodrigues(rvec, rot_matrix);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      tf_rot[i][j] = rot_matrix.at<double>(i, j);

  tf2::Quaternion q_marker;
  tf_rot.getRotation(q_marker);
  tf_marker_in_cam.setOrigin(marker_translation);
  tf_marker_in_cam.setRotation(q_marker);

  // --- camera to map ---
  tf2::Transform tf_cam_to_map;
  tf2::fromMsg(transform.transform, tf_cam_to_map);
  tf2::Transform tf_marker_in_map = tf_cam_to_map * tf_marker_in_cam;

  // --- Check if marker is upright ---
  tf2::Vector3 marker_z_axis = tf_marker_in_map.getBasis().getColumn(2);
  double z_alignment = marker_z_axis.dot(tf2::Vector3(0, 0, 1));

  if (z_alignment <= 0.8) {
    // RCLCPP_DEBUG(this->get_logger(), "[%s] Marker Z axis not upright enough: %.3f", label.c_str(), z_alignment);
    return;
  }

  // --- Set xy plane to 0 ---
  tf2::Quaternion q_flat;
  double roll, pitch, yaw;
  tf_marker_in_map.getBasis().getRPY(roll, pitch, yaw);
  q_flat.setRPY(0, 0, yaw);
  q_flat.normalize();
  tf_marker_in_map.setRotation(q_flat);

  tf2::Vector3 pos = tf_marker_in_map.getOrigin();
  pos.setZ(0.0); // Set Z to 0
  tf_marker_in_map.setOrigin(pos);

  // --- Publish PoseStamped ---
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = rclcpp::Clock().now();
  pose_stamped.header.frame_id = "map";
  tf2::toMsg(tf_marker_in_map, pose_stamped.pose); 
  pose_publisher_->publish(pose_stamped);
}

void InitTransNode::MarkerChecker() {
  rclcpp::Time now = rclcpp::Clock().now();
  if ((now - detected_time_).seconds() > 3.0) {
    // Marker ID 6 not detected for 3 seconds
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "Marker ID 6 not detected for 3 seconds");
  }
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitTransNode>(rclcpp::NodeOptions());
  node->initialize();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
