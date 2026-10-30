#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <bits/stdc++.h>

#include "communication/udp_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

//-------------------- SENDER: ROS topic -> UDP ke simulasi --------------------
// Fungsi: menerima perintah [pwm, steer] dari topic /cmd_drive,
//         lalu mengirimkannya sebagai paket UDP 12 byte ke Gazebo (port 12151).
class Sender : public rclcpp::Node {
    public:
    Sender() : Node("sender") {
        //-------------------- Setup socket UDP --------------------
        // SOCK_DGRAM = UDP. Kalau gagal, node tidak bisa mengirim apa-apa.
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(get_logger(), "Gagal buat soket");
            return;
        }

        // Alamat tujuan: simulasi di localhost (127.0.0.1), port 12151
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(udp_header::kCmdPort);
        inet_pton(AF_INET, "127.0.0.1", &addr_.sin_addr);

        //-------------------- Subscribe perintah gerak --------------------
        // data[0] = PWM (-255..255), data[1] = steering (-90..90, positif = kanan)
        // last_ dicatat untuk failsafe (lihat send()).
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

        //-------------------- Timer kirim 20 Hz --------------------
        // Dikirim terus tiap 50 ms, walaupun tidak ada perintah baru,
        // karena simulasi punya timeout stop kalau paket berhenti datang.
        timer_ = create_wall_timer(50ms, [this] { send(); });
        RCLCPP_INFO(get_logger(), "Sender Ready -> 127.0.0.1:%d", udp_header::kCmdPort);
    }

    ~Sender() {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

    private:
    //-------------------- Kirim paket + failsafe --------------------
    void send() {
        // FAILSAFE: kalau tidak ada perintah baru > 0.5 detik, PWM dipaksa 0
        // (mobil berhenti sendiri kalau node pengirim perintah mati).
        int32_t pwm = (now() - last_).seconds() > 0.5 ? 0 : pwm_;

        // Susun paket: "IRIS" + PWM(int32) + steer(int32), little-endian (ada di udp_header.hpp)
        uint8_t buf[udp_header::kCmdSize];
        udp_header::build_cmd(buf, pwm, steer_);
        if (sendto(sock_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_)) < 0) {
            RCLCPP_WARN(get_logger(), "sendto gagal");
        }
    }

    //-------------------- Variabel --------------------
    int sock_ = -1;                      // file descriptor socket
    sockaddr_in addr_{};                 // alamat tujuan (simulasi)
    int32_t pwm_ = 0, steer_ = 0;        // perintah terakhir yang diterima
    rclcpp::Time last_{0, 0, RCL_ROS_TIME};  // waktu perintah terakhir (untuk failsafe)
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sender>());
    rclcpp::shutdown();
    return 0;
}