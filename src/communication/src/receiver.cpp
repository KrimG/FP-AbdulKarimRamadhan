#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <bits/stdc++.h>

#include "communication/udp_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

//-------------------- RECEIVER: UDP dari simulasi -> ROS topic --------------------
// Fungsi: mendengarkan paket feedback 20 byte dari Gazebo (port 12152),
//         lalu mempublish ke topic /feedback = [left_ticks, right_ticks, vel_x, vel_y].
class Receiver : public rclcpp::Node {
public:
  Receiver() : Node("receiver") {
    //-------------------- Setup socket UDP --------------------
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      RCLCPP_ERROR(get_logger(), "Gagal buat socket");
      return;
    }

    // SO_REUSEADDR: port bisa dipakai lagi dengan cepat setelah node di-restart
    int opt = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    //-------------------- Bind ke port feedback --------------------
    // INADDR_ANY = terima dari semua interface. Port 12152 sesuai modul.
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

    //-------------------- Publisher + timer polling --------------------
    // Polling tiap 5 ms (non-blocking), supaya tidak membekukan rclcpp::spin.
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
  //-------------------- Baca semua paket yang menumpuk --------------------
  void poll() {
    uint8_t buf[64];
    ssize_t n;
    // MSG_DONTWAIT = tidak menunggu; loop selesai kalau antrean socket kosong
    while ((n = recvfrom(sock_, buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr)) > 0) {
      // Validasi: header harus "IRIS" dan panjang minimal 20 byte
      udp_header::Feedback fb;
      if (!udp_header::parse_feedback(buf, static_cast<size_t>(n), fb)) {
        RCLCPP_WARN(get_logger(), "Paket ga valid (%zd byte)", n);
        continue;
      }

      // Publish: [encoder kiri, encoder kanan, vel_x (cm/s), vel_y (cm/s)]
      std_msgs::msg::Int32MultiArray msg;
      msg.data = {fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms};
      pub_->publish(msg);
    }
  }

  //-------------------- Variabel --------------------
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