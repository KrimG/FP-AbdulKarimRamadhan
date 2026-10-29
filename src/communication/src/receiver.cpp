#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <bits/stdc++.h>

#include "communication/udp_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class Receiver : public rclcpp::Node {
public:
  Receiver() : Node("receiver") {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      RCLCPP_ERROR(get_logger(), "Gagal buat socket");
      return;
    }

    int opt = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(udp_header::kFeedbackPort);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind port %d gagal (sudah dipake?)", udp_header::kFeedbackPort);
      close(sock_);
      sock_ = -1;
      return;
    }

    pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("feedback", 10);
    timer_ = create_wall_timer(5ms, [this] { poll(); });
    RCLCPP_INFO(get_logger(), "Receiver siap di port %d", udp_header::kFeedbackPort);
  }

  ~Receiver() {
    if (sock_ >= 0) {
        close(sock_);
    }
  }

private:
  void poll() {
    uint8_t buf[64];
    ssize_t n;
    while ((n = recvfrom(sock_, buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr)) > 0) {
      udp_header::Feedback fb;
      if (!udp_header::parse_feedback(buf, static_cast<size_t>(n), fb)) {
        RCLCPP_WARN(get_logger(), "Paket ga valid (%zd byte)", n);
        continue;
      }

      std_msgs::msg::Int32MultiArray msg;
      msg.data = {fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms};
      pub_->publish(msg);
    }
  }

  int sock_ = -1;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Receiver>());
  rclcpp::shutdown();
  return 0;
}