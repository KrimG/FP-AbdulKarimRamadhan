#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <bits/stdc++.h>

#include "communication/udp_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class Sender : public rclcpp::Node {
    public:
    Sender() : Node("sender") {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(get_logger(), "Gagal buat soket");
            return;
        }

        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(udp_header::kCmdPort);
        inet_pton(AF_INET, "127.0.0.1", &addr_.sin_addr);

        sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "cmd_drive", 10,
        [this](std_msgs::msg::Int32MultiArray::SharedPtr m) {
            if (m->data.size() < 2) {
                return;
            }

            else {
                pwm_ = m->data[0];
                steer_ = m->data[1];
                last_ = now();
            }
        });

        timer_ = create_wall_timer(50ms, [this] { send(); });
        RCLCPP_INFO(get_logger(), "Sender Ready -> 127.0.0.1:%d", udp_header::kCmdPort);
    }

    ~Sender() {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

    private:
    void send() {
        int32_t pwm = (now() - last_).seconds() > 0.5 ? 0 : pwm_;

        uint8_t buf[udp_header::kCmdSize];
        udp_header::build_cmd(buf, pwm, steer_);
        if (sendto(sock_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_)) < 0) {
            RCLCPP_WARN(get_logger(), "sendto gagal");
        }
    }

    int sock_ = -1;
    sockaddr_in addr_{};
    int32_t pwm_ = 0, steer_ = 0;
    rclcpp::Time last_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sender>());
    rclcpp::shutdown();
    return 0;
}