#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <drobo_interfaces/msg/md_lib_msg.hpp>
#include <drobo_interfaces/msg/sd_lib_msg.hpp>
#include <motor_lib/motor_lib.hpp>

#include "dmotor_ros/main.hpp"

DMotorRos::DMotorRos(
    const rclcpp::NodeOptions& options
): DMotorRos("", options){}

DMotorRos::DMotorRos(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("d_motor_ros", name_space, options){
    _subscription_md = this->create_subscription<drobo_interfaces::msg::MdLibMsg>(
        "md_driver_topic",
        rclcpp::QoS(10),
        [this](const drobo_interfaces::msg::MdLibMsg::SharedPtr msg){
            switch(msg->mode){
                case MotorLib::Md::PWM:
                    RCLCPP_INFO(this->get_logger(), "%d,%d\n", msg->address, msg->power);
                    MotorLib::md.sendPwm(msg->address, msg->phase, msg->power, 5000);
                    break;
                case MotorLib::Md::SPEED:
                    MotorLib::md.sendSpeed(msg->address, msg->phase, msg->power, 0, 1000, 5000);
                    break;
            }
        }
    );
    _subscription_sd = this->create_subscription<drobo_interfaces::msg::SdLibMsg>(
        "sd_driver_topic",
        rclcpp::QoS(10),
        [this](const drobo_interfaces::msg::SdLibMsg::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "address: %d, port: %d, power: %d", msg->address, msg->port, msg->power1);
            auto port = msg->port ? MotorLib::Sd::LimPort::PORT1 : MotorLib::Sd::LimPort::PORT0;
            MotorLib::sd.sendPower(msg->address, port, msg->power1, 5000);
        }
    );
}

int main(int argc, char* argv[]){
    MotorLib::usb_config.vendor_id = 0x483;
    MotorLib::usb_config.product_id = 0x5740;
    MotorLib::usb_config.b_interface_number = 0;

    MotorLib::usb.setUsb(&MotorLib::usb_config);
    MotorLib::usb.openUsb();

    // ソレノイド初期設定
    for(int i = 0x00; i <= 0x01; i++){
        MotorLib::sd.sendPowers(i, 0, 0, 5000);
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DMotorRos>());
    rclcpp::shutdown();
    return 0;
}
