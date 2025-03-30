#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class SimaPoseArray : public rclcpp::Node
{
public:
    SimaPoseArray() : Node("sima_pose_array")
    {
        subscription_s1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/sima1/average_pose", 10,
            std::bind(&SimaPoseArray::s1_callback, this, std::placeholders::_1));

        subscription_s2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/sima2/average_pose", 10,
            std::bind(&SimaPoseArray::s2_callback, this, std::placeholders::_1));

        subscription_s3_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/sima3/average_pose", 10,
            std::bind(&SimaPoseArray::s3_callback, this, std::placeholders::_1));

        subscription_s4_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/sima4/average_pose", 10,
            std::bind(&SimaPoseArray::s4_callback, this, std::placeholders::_1));
        // 建立發布者
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/sima_pose", 10);

        // 設定定時器，每 100 ms 執行一次平均計算
        timer_ = this->create_wall_timer(10ms, std::bind(&SimaPoseArray::publish_pose, this));
    }

private:
    void s1_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_pose_[0] = *msg;
        sima_received_[0] = true;
        //last_s1_stamp_ = this->now();
    }

    void s2_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_pose_[1] = *msg;
        sima_received_[1] = true;
        //last_s2_stamp_ = this->now();
    }

    void s3_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_pose_[2] = *msg;
        sima_received_[2] = true;
        //last_s3_stamp_ = this->now();
    }

    void s4_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_pose_[3] = *msg;
        sima_received_[3] = true;
        //last_s4_stamp_ = this->now();
    }

    void publish_pose(){
        pose_array_.poses.clear();
        for(int i = 0; i < 4; i++){
            if(sima_received_[i] == true){
                pose_array_.poses.push_back(last_pose_[i].pose);
            }
        }
        pose_array_.header.stamp = this->now();
        publisher_->publish(pose_array_);
        sima_received_[0] = false;
        sima_received_[1] = false;
        sima_received_[2] = false;
        sima_received_[3] = false;
    }

    // 訂閱者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_s1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_s2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_s3_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_s4_;

    // 發布者
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;

    // 最新接收到的 Pose
    geometry_msgs::msg::PoseStamped last_pose_[4];

    // 是否收到資料標誌
    bool sima_received_[4] = {false,false,false,false};

    geometry_msgs::msg::PoseArray pose_array_;

    // 記錄最後接收時間
    // rclcpp::Time last_s1_stamp_;
    // rclcpp::Time last_s2_stamp_;
    // rclcpp::Time last_s3_stamp_;
    // rclcpp::Time last_s4_stamp_;

    // 定時器
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimaPoseArray>());
    rclcpp::shutdown();
    return 0;
}
