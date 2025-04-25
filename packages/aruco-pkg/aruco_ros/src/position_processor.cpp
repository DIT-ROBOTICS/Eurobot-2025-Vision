#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

/*
left
[[ 0.36160558 -0.01131429  0.83929644]
 [-0.11872641  0.64968597  0.43832529]
 [-0.11728125 -0.00392042  1.        ]]
mid
[[ 0.65444399 -0.08235688  0.30265156]
 [-0.03019623  0.69653408  0.18560042]
 [-0.08906205 -0.0530397   1.        ]]
right
[[ 1.13842936  0.04913058  0.0206142 ]
 [-0.00560925  1.01756789  0.25000881]
 [ 0.08323399  0.05802631  1.        ]]
*/
const double matrix_left[3][3] = {
    { 0.36160558, -0.01131429 ,0.83929644},
    {-0.11872641, 0.64968597 ,0.43832529},
    {-0.11728125, -0.00392042 ,1.        }
};
const double matrix_mid[3][3] = {
    { 0.65444399, -0.08235688 ,0.30265156},
    {-0.03019623, 0.69653408 ,0.18560042},
    {-0.08906205, -0.0530397 ,1.        }
};
const double matrix_right[3][3] = {
    { 1.13842936, 0.04913058 ,0.0206142 },
    {-0.00560925, 1.01756789 ,0.25000881},
    { 0.08323399, 0.05802631 ,1.        }
};

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
        double sum_orientation_x = 0.0, sum_orientation_y = 0.0, sum_orientation_z = 0.0, sum_orientation_w = 0.0;
        geometry_msgs::msg::PoseStamped average_pose;
        average_pose.header.frame_id = "map";
        average_pose.header.stamp = this->now();

        // 計算資料是否過期 (1 秒內有效)
        auto now = this->now();
        bool left_valid = left_received_ && (now - last_left_stamp_).seconds() < 1.0;
        bool mid_valid = mid_received_ && (now - last_mid_stamp_).seconds() < 1.0;
        bool right_valid = right_received_ && (now - last_right_stamp_).seconds() < 1.0;

        /*double normalize_p = (tf_mat[2][0]*stampedTransform.transform.translation.x + tf_mat[2][1]*stampedTransform.transform.translation.y + tf_mat[2][2]*1.0);
            poseMsg.pose.position.x = (tf_mat[0][0]*stampedTransform.transform.translation.x + tf_mat[0][1]*stampedTransform.transform.translation.y + tf_mat[0][2]*1.0) / normalize_p;
            poseMsg.pose.position.y = (tf_mat[1][0]*stampedTransform.transform.translation.x + tf_mat[1][1]*stampedTransform.transform.translation.y + tf_mat[1][2]*1.0) / normalize_p;
            poseMsg.pose.position.z = stampedTransform.transform.translation.z;
        */
        // 根據有效的資料進行計算
        if (left_valid)
        {
            /*double left_normalize_p = (matrix_left[2][0] * last_left_pose_.pose.position.x + matrix_left[2][1] * last_left_pose_.pose.position.y + matrix_left[2][2] * 1.0);
            sum_x += matrix_left[0][0]*last_left_pose_.pose.position.x + matrix_left[0][1]*last_left_pose_.pose.position.y + matrix_left[0][2]*1.0 / left_normalize_p;
            sum_y += matrix_left[1][0]*last_left_pose_.pose.position.x + matrix_left[1][1]*last_left_pose_.pose.position.y + matrix_left[1][2]*1.0 / left_normalize_p;
            sum_z += last_left_pose_.pose.position.z;*/
            sum_x += last_left_pose_.pose.position.x;
            sum_y += last_left_pose_.pose.position.y+0.1;
            sum_z += last_left_pose_.pose.position.z;
            sum_orientation_x += last_left_pose_.pose.orientation.x;
            sum_orientation_y += last_left_pose_.pose.orientation.y;
            sum_orientation_z += last_left_pose_.pose.orientation.z;
            sum_orientation_w += last_left_pose_.pose.orientation.w;
            valid_count++;
        }

        if (mid_valid)
        {
            /*double mid_normalize_p = (matrix_mid[2][0] * last_mid_pose_.pose.position.x + matrix_mid[2][1] * last_mid_pose_.pose.position.y + matrix_mid[2][2] * 1.0);
            sum_x += matrix_mid[0][0]*last_mid_pose_.pose.position.x + matrix_mid[0][1]*last_mid_pose_.pose.position.y + matrix_mid[0][2]*1.0 / mid_normalize_p;
            sum_y += matrix_mid[1][0]*last_mid_pose_.pose.position.x + matrix_mid[1][1]*last_mid_pose_.pose.position.y + matrix_mid[1][2]*1.0 / mid_normalize_p;
            sum_z += last_mid_pose_.pose.position.z;*/
            sum_x += last_mid_pose_.pose.position.x;
            sum_y += last_mid_pose_.pose.position.y+0.1;
            sum_z += last_mid_pose_.pose.position.z;
            sum_orientation_x += last_mid_pose_.pose.orientation.x;
            sum_orientation_y += last_mid_pose_.pose.orientation.y;
            sum_orientation_z += last_mid_pose_.pose.orientation.z;
            sum_orientation_w += last_mid_pose_.pose.orientation.w;
            valid_count++;
        }

        if (right_valid)
        {
            double right_normalize_p = (matrix_right[2][0] * last_right_pose_.pose.position.x + matrix_right[2][1] * last_right_pose_.pose.position.y + matrix_right[2][2] * 1.0);
            sum_x += matrix_right[0][0]*last_right_pose_.pose.position.x + matrix_right[0][1]*last_right_pose_.pose.position.y + matrix_right[0][2]*1.0 / right_normalize_p;
            sum_y += matrix_right[1][0]*last_right_pose_.pose.position.x + matrix_right[1][1]*last_right_pose_.pose.position.y + matrix_right[1][2]*1.0 / right_normalize_p;
            sum_z += last_right_pose_.pose.position.z;
            sum_orientation_x += last_right_pose_.pose.orientation.x;
            sum_orientation_y += last_right_pose_.pose.orientation.y;
            sum_orientation_z += last_right_pose_.pose.orientation.z;
            sum_orientation_w += last_right_pose_.pose.orientation.w;
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
            average_pose.pose.orientation.x = sum_orientation_x / valid_count;
            average_pose.pose.orientation.y = sum_orientation_y / valid_count;
            average_pose.pose.orientation.z = sum_orientation_z / valid_count;
            average_pose.pose.orientation.w = sum_orientation_w / valid_count;
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
