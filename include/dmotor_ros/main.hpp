#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <drobo_interfaces/msg/md_lib_msg.hpp>
#include <drobo_interfaces/msg/sd_lib_msg.hpp>

class DMotorRos : public rclcpp::Node{
    private:
        rclcpp::Subscription<drobo_interfaces::msg::MdLibMsg>::SharedPtr _subscription_md;
        rclcpp::Subscription<drobo_interfaces::msg::SdLibMsg>::SharedPtr _subscription_sd;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_sr;
    public:
    public:
        DMotorRos(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
        DMotorRos(
            const std::string& name_space,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
};
