//
// Created by tlab-uav on 24-9-13.
//
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>


typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

class JoystickInput final : public rclcpp::Node {
public:
    JoystickInput();

    ~JoystickInput() override = default;

private:
    void remoteWirelessHandle(const void *messages);

    std::string network_interface_ = "lo";
    int domain_ = 0;

    unitree_go::msg::dds_::WirelessController_ wireless_controller_{};
    xKeySwitchUnion xKeySwitchUnion_{};

    control_input_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> subscriber_;
};
