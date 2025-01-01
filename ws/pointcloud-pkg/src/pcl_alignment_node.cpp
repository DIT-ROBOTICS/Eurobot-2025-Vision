#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_preprocess.hpp"
#include "icp_registration.hpp"
#include "pcl_publisher.hpp"

class PCLAlignmentNode : public rclcpp::Node {
public:
    PCLAlignmentNode() : Node("pcl_alignment") {
        using namespace std::placeholders;

        _preprocess = std::make_shared<PCLPreprocess>();
        _align = std::make_shared<ICPRegistration>();
        _pub = std::make_shared<PCLPublisher>(this);

        _sub_cam1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/realsense1/cam_left/depth/color/points", 
            10, std::bind(&PCLAlignmentNode::callback1, this, _1));

        _sub_cam2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/realsense2/cam_mid/depth/color/points", 
            10, std::bind(&PCLAlignmentNode::callback2, this, _1)); 

        _sub_cam3 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/realsense3/cam_right/depth/color/points", 
            10, std::bind(&PCLAlignmentNode::callback3, this, _1));
    }

private:
    void callback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud1);
    }
    void callback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud2);
    }
    void callback3(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud3);
    }

    // for the first callback, downsample & calculate alignemet matrix
    _preprocess->Downsample(cloud1);
    _preprocess->downsample(cloud2);
    _preprocess->downsample(cloud3);

    // align cloud1 & cloud3 to cloud2
    _align->align(cloud1, cloud2);
    _align->align(cloud3, cloud2);

    std::shared_ptr<PCLPreprocess> _preprocess;
    std::shared_ptr<ICPRegistration> _align;
    std::shared_ptr<PCLPublisher> _pub;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_cam1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_cam2;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_cam3;
}
