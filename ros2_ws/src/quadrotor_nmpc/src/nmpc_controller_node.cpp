#include"rclcpp/rclcpp.hpp"
#include"nav_msgs/msg/odometry.hpp"
#include<optional>

class NmpcControllerNode : public rclcpp::Node{
    public:
        NmpcControllerNode():Node("nmpc_controller")
        {
            control_rate_hz = this->declare_parameter<double>("control_rate_hz", 100.0);
            auto period = std::chrono::duration<double>(1/control_rate_hz);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&NmpcControllerNode::control_loop, this));
            
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(),
                    std::bind(&NmpcControllerNode::on_odem, this, std::placeholders::_1));
        }



    private:
        void control_loop(){

            if (!last_odem_.has_value()){
                return;                    //do not run until mpc have state values
            }

            static int tick = 0;
            if(++tick % 100 == 0){
                RCLCPP_INFO(this->get_logger(),"controller is alive");
            }
        }


        void on_odem(const nav_msgs::msg::Odometry & msg){
            last_odem_ = msg;
            static int count = 0;
            if (++count % 50 == 0){
                RCLCPP_INFO(this->get_logger(), "receiving odem. z = %.2f", msg.pose.pose.position.z);
            }
        }


        double control_rate_hz{100.00};
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        std::optional<nav_msgs::msg::Odometry> last_odem_;
};



int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NmpcControllerNode>());
    rclcpp::shutdown();

    return 0;
}