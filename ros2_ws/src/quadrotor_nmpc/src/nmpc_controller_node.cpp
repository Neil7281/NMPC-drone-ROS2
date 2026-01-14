#include"rclcpp/rclcpp.hpp"

class NmpcControllerNode : public rclcpp::Node{
    public:
        NmpcControllerNode():Node("nmpc_controller")
        {
            control_rate_hz = this->declare_parameter<double>("control_rate_hz", 100.0);
            auto period = std::chrono::duration<double>(1/control_rate_hz);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&NmpcControllerNode::control_loop, this));

        }



    private:
        void control_loop(){
            static int tick = 0;
            if(tick++ % 100 == 0){
                RCLCPP_INFO(this->get_logger(),"controller is alive");
            }
        }


        double control_rate_hz{100.00};
        rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NmpcControllerNode>());
    rclcpp::shutdown();

    return 0;
}