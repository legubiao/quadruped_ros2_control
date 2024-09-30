//
// Created by biao on 24-9-11.
//


#ifndef KEYBOARDINPUT_H
#define KEYBOARDINPUT_H
#include <rclcpp/rclcpp.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <termios.h>


template <typename T1, typename T2>
T1 max(const T1 a, const T2 b) {
    return (a > b ? a : b);
}

template <typename T1, typename T2>
T1 min(const T1 a, const T2 b) {
    return (a < b ? a : b);
}


class KeyboardInput final : public rclcpp::Node {
public:
    KeyboardInput();

    ~KeyboardInput() override {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

private:
    void timer_callback();

    void check_command(char key);
    void check_value(char key);

    static bool kbhit();

    control_input_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool just_published_ = false;
    int reset_count_ = 0;

    float sensitivity_left_ = 0.05;
    float sensitivity_right_ = 0.05;
    termios old_tio_{}, new_tio_{};
};


#endif //KEYBOARDINPUT_H
