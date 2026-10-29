#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node {
public:
    KeyboardTeleop() : Node("keyboard_teleop") {
        pwm_step_ = declare_parameter<int>("pwm_step", 10);
        steer_step_ = declare_parameter<int>("steer_step", 5);
        pwm_max_ = declare_parameter<int>("pwm_max", 255);

        if (tcgetattr(STDIN_FILENO, &old_term_) == 0) {
            termios raw = old_term_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            term_saved_ = true;
        } else {
            RCLCPP_ERROR(get_logger(), "stdin bukan terminal, keyboard input gabisa");
        }

        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("cmd_drive", 10);
        timer_ = create_wall_timer(50ms, [this] { tick(); });

        std::printf(
            "\nW/S: pwm +/-%d   A/D: steer -/+%d   SPACE: stop & center   Q: quit\n",
            pwm_step_, steer_step_);
    }

    ~KeyboardTeleop() {
        if (term_saved_) tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
        std::printf("\n");
    }

private:
    bool key_available() {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv{0, 0};
        return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0;
    }

    void tick() {
        while (term_saved_ && key_available()) {
            char c;
            if (read(STDIN_FILENO, &c, 1) != 1) {
                break;
            }
            switch (c) {
                case 'w': case 'W': pwm_ += pwm_step_; break;
                case 's': case 'S': pwm_ -= pwm_step_; break;
                case 'a': case 'A': steer_ -= steer_step_; break;
                case 'd': case 'D': steer_ += steer_step_; break;
                case ' ': pwm_ = 0; steer_ = 0; break;
                case 'q': case 'Q':
                    rclcpp::shutdown();
                    return;
                default: break;
            }
        }

        pwm_ = std::clamp(pwm_, -pwm_max_, pwm_max_);
        steer_ = std::clamp(steer_, -90, 90);

        std_msgs::msg::Int32MultiArray msg;
        msg.data = {pwm_, steer_};
        pub_->publish(msg);

        std::printf("\rpwm=%4d  steer=%4d   ", pwm_, steer_);
        std::fflush(stdout);
    }

    int pwm_step_, steer_step_, pwm_max_;
    int pwm_ = 0, steer_ = 0;
    termios old_term_{};
    bool term_saved_ = false;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}