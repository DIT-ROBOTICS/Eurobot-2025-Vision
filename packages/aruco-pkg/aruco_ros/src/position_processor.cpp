#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class PositionProcessor : public rclcpp::Node
{
public:
    PositionProcessor() : Node("position_processor")
    {
        // 訂閱左邊、中間、右邊的話題
        subscription_left_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/left/single/pose", 10,
            std::bind(&PositionProcessor::left_callback, this, std::placeholders::_1));

        subscription_mid_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mid/single/pose", 10,
            std::bind(&PositionProcessor::mid_callback, this, std::placeholders::_1));

        subscription_right_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/right/single/pose", 10,
            std::bind(&PositionProcessor::right_callback, this, std::placeholders::_1));

        // 建立發布者
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/average_pose", 10);

        // 設定定時器，每 100 ms 執行一次平均計算
        timer_ = this->create_wall_timer(10ms, std::bind(&PositionProcessor::calculate_and_publish_average, this));
    }

private:
    void left_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_left_pose_ = *msg;
        left_received_ = true;
        last_left_stamp_ = this->now();
    }

    void mid_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_mid_pose_ = *msg;
        mid_received_ = true;
        last_mid_stamp_ = this->now();
    }

    void right_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_right_pose_ = *msg;
        right_received_ = true;
        last_right_stamp_ = this->now();
    }

    void calculate_and_publish_average()
    {
        int valid_count = 0;
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        geometry_msgs::msg::PoseStamped average_pose;
        average_pose.header.frame_id = "map";
        average_pose.header.stamp = this->now();

        // 計算資料是否過期 (1 秒內有效)
        auto now = this->now();
        bool left_valid = left_received_ && (now - last_left_stamp_).seconds() < 1.0;
        bool mid_valid = mid_received_ && (now - last_mid_stamp_).seconds() < 1.0;
        bool right_valid = right_received_ && (now - last_right_stamp_).seconds() < 1.0;

        // 根據有效的資料進行計算
        if (left_valid)
        {
            sum_x += last_left_pose_.pose.position.x;
            sum_y += last_left_pose_.pose.position.y;
            sum_z += last_left_pose_.pose.position.z;
            valid_count++;
        }

        if (mid_valid)
        {
            sum_x += last_mid_pose_.pose.position.x;
            sum_y += last_mid_pose_.pose.position.y;
            sum_z += last_mid_pose_.pose.position.z;
            valid_count++;
        }

        if (right_valid)
        {
            sum_x += last_right_pose_.pose.position.x;
            sum_y += last_right_pose_.pose.position.y;
            sum_z += last_right_pose_.pose.position.z;
            valid_count++;
        }

        // 輸出 valid_count
        //RCLCPP_INFO(this->get_logger(), "Valid count: %d", valid_count);

        // 如果有有效數據才發布平均值
        if (valid_count > 0)
        {
            average_pose.pose.position.x = sum_x / valid_count;
            average_pose.pose.position.y = sum_y / valid_count;
            average_pose.pose.position.z = sum_z / valid_count;
            publisher_->publish(average_pose);
        }
    }

    // 訂閱者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_left_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_mid_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_right_;

    // 發布者
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

    // 最新接收到的 Pose
    geometry_msgs::msg::PoseStamped last_left_pose_;
    geometry_msgs::msg::PoseStamped last_mid_pose_;
    geometry_msgs::msg::PoseStamped last_right_pose_;

    // 是否收到資料標誌
    bool left_received_ = false;
    bool mid_received_ = false;
    bool right_received_ = false;

    // 記錄最後接收時間
    rclcpp::Time last_left_stamp_;
    rclcpp::Time last_mid_stamp_;
    rclcpp::Time last_right_stamp_;

    // 定時器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionProcessor>());
    rclcpp::shutdown();
    return 0;
}
