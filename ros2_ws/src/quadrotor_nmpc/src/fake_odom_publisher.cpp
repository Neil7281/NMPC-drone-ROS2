#include "rclcpp/rclcpp.hpp"
#include"nav_msgs/msg/odometry.hpp"


class FakeOdomPublisher: public rclcpp::Node{
    public:
        FakeOdomPublisher():Node("fake_odom_publisher"){
            rate_hz_ = this->declare_parameter<double>("rate_hz", 50.00);
            pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            auto period = std::chrono::duration<double>(1/rate_hz_);
            timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),std::bind(&FakeOdomPublisher::tick, this));
        }


    private:
        void tick(){
            nav_msgs::msg::Odometry msg;

            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "word";
            msg.child_frame_id = "base_link";

            msg.pose.pose.position.z = z_;
            z_+=0.01;
            pub_->publish(msg);
        }
        
        double rate_hz_{50.00};
        double z_{0.00};
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};




int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdomPublisher>());
    rclcpp::shutdown();
}