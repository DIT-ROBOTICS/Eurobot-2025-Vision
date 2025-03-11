#ifndef RTSP_ROS__RTSP_CAM_NODE_HPP_
#define RTSP_ROS__RTSP_CAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace rtsp_ros
{
class GStreamerRTSPsubs
{
public:
    explicit GStreamerRTSPsubs(const std::string &rtsp_url, const rclcpp::Logger &logger);
    ~GStreamerRTSPsubs();
    bool get_frame(cv::Mat &frame);

private:
    cv::VideoCapture cap_;
    std::string rtsp_url_;
    rclcpp::Logger logger_;
};

class CamNodePublisher : public rclcpp::Node
{
public:
    explicit CamNodePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~CamNodePublisher();

private:
    void publish_frame();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::CameraInfo camera_info_;
    camera_info_manager::CameraInfoManager cinfo_;
    
    std::string rtsp_url_;
    std::string camera_name_;
    std::string camera_info_file_;
    
    std::shared_ptr<GStreamerRTSPsubs> rtsp_stream_;
};

class NodeErrorHandle
{
public:
    static void log_error(const rclcpp::Logger &logger, const std::string &message);
};

} // namespace rtsp_ros

#endif // RTSP_ROS__RTSP_CAM_NODE_HPP_