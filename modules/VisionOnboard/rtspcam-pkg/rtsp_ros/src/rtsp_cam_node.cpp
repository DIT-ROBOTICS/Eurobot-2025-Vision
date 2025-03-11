#include "rtsp_ros/rtsp_cam_node.hpp"

namespace rtsp_ros
{

// ===== NodeErrorHandle : Report ERROR Log to terminal =====
void NodeErrorHandle::log_error(const rclcpp::Logger &logger, const std::string &message)
{
    RCLCPP_ERROR(logger, "%s", message.c_str());
}

// ===== GStreamerRTSPsubs : Subscribe RTSP video stream using GStream =====
GStreamerRTSPsubs::GStreamerRTSPsubs(const std::string &rtsp_url, const rclcpp::Logger &logger)
: rtsp_url_(rtsp_url), logger_(logger)
{
    std::string pipeline = "rtspsrc location=" + rtsp_url_ + " latency=0 do-timestamp=true ! "
                           "rtpjpegdepay ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink sync=true";

    cap_.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
        NodeErrorHandle::log_error(logger_, "Failed to open RTSP stream: " + rtsp_url_);
    }
}

bool GStreamerRTSPsubs::get_frame(cv::Mat &frame)
{
    return cap_.read(frame);
}

GStreamerRTSPsubs::~GStreamerRTSPsubs()
{
    cap_.release();
}


// ===== CamNodePublisher : ROS Camera Publish Node =====
CamNodePublisher::CamNodePublisher(const rclcpp::NodeOptions & options)
: Node("rtsp_cam_node", options),
  cinfo_(this, "ceiling_cam")
{
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    // Declare parameters
    this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.50.60:8554/camera");
    this->declare_parameter<std::string>("camera_name", "ceiling_cam");
    this->declare_parameter<std::string>("camera_info_file", "package://rtsp_ros/config/ceiling_cam_calibration.yaml");

    this->get_parameter("rtsp_url", rtsp_url_);
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("camera_info_file", camera_info_file_);

    try {
        // Stream RTSP video using GStreamer
        rtsp_stream_ = std::make_shared<GStreamerRTSPsubs>(rtsp_url_, this->get_logger());
    } catch (const std::exception &e) {
        NodeErrorHandle::log_error(this->get_logger(), e.what());
        return;
    }

    // Load camera calibration info
    if (cinfo_.validateURL(camera_info_file_)) {
        cinfo_.loadCameraInfo(camera_info_file_);
    } else {
        NodeErrorHandle::log_error(this->get_logger(), "Failed to load camera calibration file: " + camera_info_file_);
    }
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / 30),  // 30 FPS
        std::bind(&CamNodePublisher::publish_frame, this));
}

void CamNodePublisher::publish_frame()
{
    cv::Mat frame;
    if (!rtsp_stream_->get_frame(frame)) {
        NodeErrorHandle::log_error(this->get_logger(), "Failed to read frame from RTSP stream");
        return;
    }

    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = camera_name_;

    img_bridge = cv_bridge::CvImage(header, "bgr8", frame);
    img_bridge.toImageMsg(img_msg);
    pub_image_->publish(img_msg);

    if (cinfo_.isCalibrated()) {
        camera_info_ = cinfo_.getCameraInfo();
        camera_info_.header = header;
        pub_camera_info_->publish(camera_info_);
    }
}

CamNodePublisher::~CamNodePublisher()
{
    RCLCPP_WARN(this->get_logger(), "Shutting down RTSP Node ...");
}

}  // namespace rtsp_ros


// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rtsp_ros::CamNodePublisher)
