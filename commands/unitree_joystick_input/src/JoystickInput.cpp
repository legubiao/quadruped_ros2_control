//
// Created by tlab-uav on 24-9-13.
//

#include "unitree_joystick_input/JoystickInput.h"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using std::placeholders::_1;

using namespace unitree::robot;

JoystickInput::JoystickInput() : Node("unitree_joysick_node")
{
    publisher_ = create_publisher<control_input_msgs::msg::Inputs>("control_input", 10);

    declare_parameter("network_interface", network_interface_);
    declare_parameter("domain", domain_);

    network_interface_ = get_parameter("network_interface").as_string();
    domain_ = get_parameter("domain").as_int();
    RCLCPP_INFO(get_logger(), " network_interface: %s, domain: %d", network_interface_.c_str(), domain_);
    ChannelFactory::Instance()->Init(domain_, network_interface_);

    subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(
        TOPIC_JOYSTICK);
    subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            remoteWirelessHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);
}

void JoystickInput::remoteWirelessHandle(const void* messages)
{
    wireless_controller_ = *static_cast<const unitree_go::msg::dds_::WirelessController_*>(messages);
    xKeySwitchUnion_.value = wireless_controller_.keys();

    if (xKeySwitchUnion_.components.select)
    {
        inputs_.command = 1;
    }
    else if (xKeySwitchUnion_.components.start)
    {
        inputs_.command = 2;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.B)
    {
        inputs_.command = 3;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.A)
    {
        inputs_.command = 4;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.X)
    {
        inputs_.command = 5;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.Y)
    {
        inputs_.command = 6;
    }
    else if (xKeySwitchUnion_.components.left and xKeySwitchUnion_.components.B)
    {
        inputs_.command = 7;
    }
    else if (xKeySwitchUnion_.components.left and xKeySwitchUnion_.components.A)
    {
        inputs_.command = 8;
    }
    else if (xKeySwitchUnion_.components.left and xKeySwitchUnion_.components.X)
    {
        inputs_.command = 9;
    }
    else if (xKeySwitchUnion_.components.left and xKeySwitchUnion_.components.Y)
    {
        inputs_.command = 10;
    }
    else
    {
        inputs_.command = 0;
        inputs_.lx = wireless_controller_.lx();
        inputs_.ly = wireless_controller_.ly();
        inputs_.rx = wireless_controller_.rx();
        inputs_.ry = wireless_controller_.ry();
    }
    publisher_->publish(inputs_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
