//
// Created by tlab-uav on 24-9-13.
//

#ifndef JOYSTICKINPUT_H
#define JOYSTICKINPUT_H
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <control_input_msgs/msg/inputs.hpp>


class JoystickInput final : public rclcpp::Node {
public:
    JoystickInput();

    ~JoystickInput() override = default;

private:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

    control_input_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};


#endif //JOYSTICKINPUT_H
