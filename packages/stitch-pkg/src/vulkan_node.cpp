#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "stitch_vulkan/vulkan_processor.hpp"

class VulkanNode : public rclcpp::Node {
public:
    VulkanNode() : Node("vulkan_node") {
        if (!processor_.initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Vulkan failed to initialize.");
            throw std::runtime_error("Failed to initialize VulkanProcessor");
        }

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/vision/cam_left/color/image_raw", 10,
            std::bind(&VulkanNode::imageCallback, this, std::placeholders::_1)
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            processor_.processImage(image);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    VulkanProcessor processor_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<VulkanNode>());
    } catch (const std::exception& e) {
        std::cerr << "Exception during VulkanNode spin: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
