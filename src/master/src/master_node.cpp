#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class Master : public rclcpp::Node {
public:
    Master() : Node("master") {
        kc_ = declare_parameter<double>("kc", 60.0);
        kp_ = declare_parameter<double>("kp", 60.0);
        kd_ = declare_parameter<double>("kd", 8.0);
        pwm_max_ = declare_parameter<int>("pwm_max", 100);
        w_near_ = declare_parameter<double>("w_near", 0.3);
        enabled_ = declare_parameter<bool>("enabled", true);

        lost_limit_ = declare_parameter<double>("lost_limit", 0.6);
        stuck_time_ = declare_parameter<double>("stuck_time", 1.5);
        stuck_speed_ = declare_parameter<double>("stuck_speed", 3.0);
        reverse_pwm_ = declare_parameter<int>("reverse_pwm", 40);
        reverse_min_ = declare_parameter<double>("reverse_min", 1.0);
        reverse_max_ = declare_parameter<double>("reverse_max", 3.0);
        reverse_steer_gain_ = declare_parameter<double>("reverse_steer_gain", -1.0);

        last_valid_ = now();
        last_fb_ = now();
        drive_start_ = now();
        rev_start_ = now();
        stuck_since_ = now();

        sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "vision/lane", 10,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr m) { on_lane(*m); });
        fb_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "feedback", 10,
            [this](std_msgs::msg::Int32MultiArray::SharedPtr m) { on_feedback(*m); });
        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("cmd_drive", 10);
        timer_ = create_wall_timer(50ms, [this] { tick(); });
    }

private:
    enum class State { DRIVE, REVERSE };
    static constexpr double kDt = 0.05;

    void on_lane(const std_msgs::msg::Float32MultiArray &m) {
        if (m.data.size() < 3 || m.data[2] < 0.5f) {
            return;
        }
        near_ = m.data[0];
        far_ = m.data[1];

        double e = w_near_ * near_ + (1.0 - w_near_) * far_;
        double t = now().seconds();
        double dt = t - last_t_;
        if (dt > 0.001 && dt < 0.5) {
            de_ = (e - last_e_) / dt;
        }
        last_e_ = e;
        last_t_ = t;
        e_ = e;
        last_valid_ = now();
        has_lane_ = true;
    }

    void on_feedback(const std_msgs::msg::Int32MultiArray &m) {
        if (m.data.size() < 4) {
            return;
        }
        speed_ = std::hypot(static_cast<double>(m.data[2]), static_cast<double>(m.data[3]));
        last_fb_ = now();
        has_fb_ = true;
    }

    void start_reverse(const char *reason, const rclcpp::Time &t) {
        RCLCPP_WARN(get_logger(), "MUNDUR: %s", reason);
        state_ = State::REVERSE;
        rev_start_ = t;
        good_ = 0.0;
        stuck_ = false;
    }

    void tick() {
        if (!enabled_) {
            return;
        }
        const rclcpp::Time t = now();
        const double lost = (t - last_valid_).seconds();
        int pwm = 0, steer = 0;

        if (state_ == State::DRIVE) {
            if (!has_lane_) {
                // belum pernah melihat garis (vision belum jalan): diam saja
            } else if (lost < 0.3) {
                double curve = far_ - near_;
                double s = std::clamp(kp_ * e_ + kd_ * de_ + kc_ * curve, -90.0, 90.0);
                steer = static_cast<int>(std::lround(s));

                if (std::abs(steer) >= std::abs(last_steer_) || steer_age_ > 1.0) {
                    last_steer_ = steer;
                    steer_age_ = 0.0;
                } else {
                    steer_age_ += kDt;
                }

                double bend = std::min(1.0, std::abs(curve) / 0.6);
                double off = std::min(1.0, std::abs(e_) / 0.5);
                double turn = std::min(1.0, std::abs(s) / 60.0);
                double k = (1.0 - 0.5 * bend) * (1.0 - 0.3 * off) * (1.0 - 0.5 * turn);
                pwm = std::max(15, static_cast<int>(pwm_max_ * k));

                bool fb_ok = has_fb_ && (t - last_fb_).seconds() < 1.0;
                bool grace = (t - drive_start_).seconds() < 2.0;
                if (fb_ok && !grace && pwm > 10 && speed_ < stuck_speed_) {
                    if (!stuck_) {
                        stuck_ = true;
                        stuck_since_ = t;
                    } else if ((t - stuck_since_).seconds() > stuck_time_) {
                        start_reverse("mobil macet", t);
                    }
                } else {
                    stuck_ = false;
                }
            } else if (lost < lost_limit_) {
                pwm = 0;
                steer = last_steer_;
            } else {
                start_reverse("garis hilang", t);
            }
        }

        if (state_ == State::REVERSE) {
            double el = (t - rev_start_).seconds();
            pwm = -reverse_pwm_;
            steer = static_cast<int>(
                std::clamp(std::lround(last_steer_ * reverse_steer_gain_), -90L, 90L));

            if (lost < 0.15) {
                good_ += kDt;
            } else {
                good_ = 0.0;
            }

            bool found = el > reverse_min_ && good_ > 0.4;
            if (found || el > reverse_max_) {
                RCLCPP_INFO(get_logger(), "Selesai mundur (%s), lanjut maju",
                            found ? "garis ketemu" : "waktu habis");
                state_ = State::DRIVE;
                drive_start_ = t;
                de_ = 0.0;
                stuck_ = false;
                last_steer_ = 0;
                steer_age_ = 0.0;
                pwm = 0;
                steer = 0;
            }
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "%s lost=%.2f e=%.2f spd=%.1f pwm=%d steer=%d",
                             state_ == State::DRIVE ? "DRIVE" : "REVERSE",
                             lost, e_, speed_, pwm, steer);

        std_msgs::msg::Int32MultiArray msg;
        msg.data = {pwm, steer};
        pub_->publish(msg);
    }

    double kp_, kd_, kc_, w_near_;
    int pwm_max_;
    bool enabled_;

    double lost_limit_, stuck_time_, stuck_speed_;
    int reverse_pwm_;
    double reverse_min_, reverse_max_, reverse_steer_gain_;

    State state_ = State::DRIVE;
    bool has_lane_ = false, has_fb_ = false, stuck_ = false;
    int last_steer_ = 0;
    double steer_age_ = 0.0;
    double good_ = 0.0, speed_ = 0.0;
    double e_ = 0, de_ = 0, last_e_ = 0, last_t_ = 0;
    double near_ = 0, far_ = 0;
    rclcpp::Time last_valid_, last_fb_, drive_start_, rev_start_, stuck_since_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fb_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Master>());
    rclcpp::shutdown();
    return 0;
}