#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

        // --- obstacle avoidance ---
        obs_stop_ = declare_parameter<double>("obs_stop", 15.0);              // mulai menghindar (m)
        obs_min_ = declare_parameter<double>("obs_min", 2.0);                // jarak untuk slowdown maksimum (m)
        obs_half_width_ = declare_parameter<double>("obs_half_width", 1.4);  // setengah lebar koridor lidar (m)
        obs_avoid_ = declare_parameter<double>("obs_avoid", 0.25);           // geseran target (lebar lajur)
        car_half_w_ = declare_parameter<double>("car_half_w", 0.22);         // setengah lebar mobil (lebar lajur)
        obs_ramp_ = declare_parameter<double>("obs_ramp", 0.8);              // laju geseran (unit/detik)
        obs_slow_ = declare_parameter<double>("obs_slow", 0.3);              // pengurangan pwm (0..1)
        pass_dist_ = declare_parameter<double>("pass_dist", 7.0);            // jarak tempuh setelah objek hilang (m)
        speed_scale_ = declare_parameter<double>("speed_scale", 1.0);        // speed_ -> m/s
        max_hold_ = declare_parameter<double>("max_hold", 3.0);              // batas waktu hold (s)
        obs_steer_cmd_ = declare_parameter<double>("obs_steer_cmd", 30.0);
        obs_min_steer_ = declare_parameter<double>("obs_min_steer", 20.0);
        obs_emergency_dist_ = declare_parameter<double>("obs_emergency_dist", 1.8);
        obs_creep_pwm_ = declare_parameter<int>("obs_creep_pwm", 35);

        // --- mundur karena objek terlalu dekat ---
        obs_rev_steer_ = declare_parameter<double>("obs_rev_steer", 60.0);          // steer saat mundur
        obs_rev_clear_ = declare_parameter<double>("obs_rev_clear", 3.5);           // berhenti mundur jika jarak >= ini (m)
        obs_too_close_time_ = declare_parameter<double>("obs_too_close_time", 0.4); // lama "terlalu dekat" sebelum mundur (s)
        obs_rev_min_ = declare_parameter<double>("obs_rev_min", 0.5);               // waktu mundur minimum (s)
        obs_rev_max_ = declare_parameter<double>("obs_rev_max", 3.0);               // waktu mundur maksimum (s)

        last_valid_ = now();
        last_fb_ = now();
        last_scan_ = now();
        last_obs_seen_ = now();
        active_since_ = now();
        drive_start_ = now();
        rev_start_ = now();
        stuck_since_ = now();
        too_close_since_ = now();

        sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "vision/lane", 10,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr m) { on_lane(*m); });
        fb_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "feedback", 10,
            [this](std_msgs::msg::Int32MultiArray::SharedPtr m) { on_feedback(*m); });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/catvehicle/front_laser_points", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) { on_scan(*m); });
        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("cmd_drive", 10);
        timer_ = create_wall_timer(50ms, [this] { tick(); });
    }

private:
    enum class State { DRIVE, REVERSE };
    static constexpr double kDt = 0.05;

    // Format vision/lane: data = {e_near, e_far, valid}
    void on_lane(const std_msgs::msg::Float32MultiArray &m) {
        if (m.data.size() < 3) {
            return;
        }
        // valid == 0 -> lajur tidak terdeteksi, jangan perbarui last_valid_
        if (m.data[2] < 0.5f) {
            return;
        }

        const rclcpp::Time t = now();
        const double n = static_cast<double>(m.data[0]);
        const double f = static_cast<double>(m.data[1]);
        const double e = n;  // error kemudi dari band dekat

        const double dt = (t - last_valid_).seconds();
        if (has_lane_ && dt > 1e-3 && dt < 0.5) {
            de_ = (e - last_e_) / dt;
        } else {
            de_ = 0.0;
        }

        e_ = e;
        last_e_ = e;
        near_ = n;
        far_ = f;
        last_valid_ = t;
        has_lane_ = true;
    }

    void on_scan(const sensor_msgs::msg::LaserScan &m) {
        double nearest = 1e9;
        double nearest_y = 0.0;

        // Jarak terdekat pada sisi kiri dan kanan
        double left_clear = 1e9;
        double right_clear = 1e9;

        for (size_t i = 0; i < m.ranges.size(); i++) {
            double r = m.ranges[i];

            if (!std::isfinite(r) || r < m.range_min || r > m.range_max) {
                continue;
            }

            double a = m.angle_min + static_cast<double>(i) * m.angle_increment;
            double x = r * std::cos(a);
            double y = r * std::sin(a);

            if (x <= 0.0) continue;

            // Ukur ruang kosong di sisi kiri dan kanan
            if (y > 0.35 && std::abs(y) <= 2.5) {
                left_clear = std::min(left_clear, x);
            }
            if (y < -0.35 && std::abs(y) <= 2.5) {
                right_clear = std::min(right_clear, x);
            }

            // Deteksi objek di koridor depan
            if (std::abs(y) <= obs_half_width_) {
                if (x < nearest) {
                    nearest = x;
                    nearest_y = y;
                }
            }
        }

        last_scan_ = now();
        obs_dist_ = nearest;  // selalu diperbarui (1e9 jika tidak ada objek)

        if (nearest < obs_stop_) {
            last_obs_seen_ = now();

            if (!obs_active_) {
                // +1 = kanan, -1 = kiri
                if (left_clear > right_clear + 0.2) {
                    avoid_dir_ = 1.0;
                } else if (right_clear > left_clear + 0.2) {
                    avoid_dir_ = -1.0;
                } else {
                    // Sama-sama lapang / tidak ada pembacaan: pilih berdasarkan posisi objek
                    if (nearest_y > 0.15) {
                        avoid_dir_ = 1.0;
                    } else if (nearest_y < -0.15) {
                        avoid_dir_ = -1.0;
                    } else {
                        avoid_dir_ = 1.0;
                    }
                }

                obs_active_ = true;
                travelled_ = 0.0;
                active_since_ = now();

                RCLCPP_WARN(get_logger(),
                            "OBSTACLE AKTIF: jarak=%.2f kiri=%.2f kanan=%.2f arah=%s",
                            nearest, left_clear, right_clear,
                            avoid_dir_ > 0 ? "KANAN" : "KIRI");
            }
        }
    }

    void on_feedback(const std_msgs::msg::Int32MultiArray &m) {
        if (m.data.size() < 4) {
            return;
        }
        speed_ = std::hypot(static_cast<double>(m.data[2]), static_cast<double>(m.data[3]));
        last_fb_ = now();
        has_fb_ = true;
    }

    // Mundur karena garis hilang / macet (logika lama)
    void start_reverse(const char *reason, const rclcpp::Time &t) {
        RCLCPP_WARN(get_logger(), "MUNDUR: %s", reason);
        state_ = State::REVERSE;
        rev_obs_ = false;
        rev_start_ = t;
        good_ = 0.0;
        stuck_ = false;
    }

    // Mundur karena objek terlalu dekat (agar bisa belok)
    void start_obs_reverse(const rclcpp::Time &t) {
        RCLCPP_WARN(get_logger(), "MUNDUR (objek terlalu dekat %.2f m), arah=%s",
                    obs_dist_, avoid_dir_ > 0 ? "KANAN" : "KIRI");
        state_ = State::REVERSE;
        rev_obs_ = true;
        rev_start_ = t;
        too_close_ = false;
        stuck_ = false;
    }

    void tick() {
        if (!enabled_) {
            return;
        }
        const rclcpp::Time t = now();
        const double lost = (t - last_valid_).seconds();
        int pwm = 0, steer = 0;
        const double step = obs_ramp_ * kDt;
        const double lim = std::max(0.05, 0.5 - car_half_w_);

        const bool visible = (t - last_obs_seen_).seconds() < 0.3 &&
                             (t - last_scan_).seconds() < 1.0;

        if (obs_active_) {
            if (visible) {
                travelled_ = 0.0;
            } else {
                travelled_ += speed_ * speed_scale_ * kDt;
            }

            if (travelled_ > pass_dist_) {
                obs_active_ = false;
                bias_ = 0.0;
                RCLCPP_INFO(get_logger(), "Obstacle dilewati, kembali ke lane following");
            }
        }

        if (state_ == State::DRIVE) {
            if (!has_lane_) {
                // belum ada data lane
            } else if (lost < 0.3) {
                double curve = far_ - near_;

                double prox = 0.0;
                double target_bias = 0.0;
                if (obs_active_) {
                    target_bias = avoid_dir_ * std::min(obs_avoid_, lim);

                    double p_dir = avoid_dir_ * (-e_);
                    if (p_dir >= lim) {
                        target_bias = (avoid_dir_ * bias_ > 0.0) ? bias_ : 0.0;
                    }

                    if (visible && obs_dist_ < obs_stop_) {
                        prox = (obs_stop_ - obs_dist_) / std::max(0.1, obs_stop_ - obs_min_);
                        prox = std::clamp(prox, 0.0, 1.0);
                    }
                }
                bias_ += std::clamp(target_bias - bias_, -step, step);
                bias_ = std::clamp(bias_, -lim, lim);

                double e_use = e_ + bias_;
                double s = kp_ * e_use + kd_ * de_ + kc_ * curve;

                if (obs_active_ && visible) {
                    s += avoid_dir_ * obs_steer_cmd_;

                    if (avoid_dir_ > 0.0) {
                        s = std::max(s, obs_min_steer_);
                    } else {
                        s = std::min(s, -obs_min_steer_);
                    }
                }

                s = std::clamp(s, -90.0, 90.0);
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
                k *= (1.0 - obs_slow_ * prox);
                pwm = std::max(15, static_cast<int>(pwm_max_ * k));

                bool close_now = false;
                if (obs_active_ && visible) {
                    if (obs_dist_ < 4.0) {
                        pwm = std::min(pwm, obs_creep_pwm_);
                    }
                    if (obs_dist_ <= obs_emergency_dist_) {
                        close_now = true;
                        pwm = 0;
                        if (!too_close_) {
                            too_close_ = true;
                            too_close_since_ = t;
                        } else if ((t - too_close_since_).seconds() > obs_too_close_time_) {
                            start_obs_reverse(t);
                        }
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                                             "EMERGENCY STOP: obstacle %.2f m", obs_dist_);
                    }
                }
                if (!close_now) {
                    too_close_ = false;
                }

                if (obs_active_) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                         "OBJEK jarak=%.2f vis=%d dir=%+.0f bias=%.2f travelled=%.1f e=%.2f steer=%d",
                                         obs_dist_, visible, avoid_dir_, bias_, travelled_, e_, steer);
                }

                bool fb_ok = has_fb_ && (t - last_fb_).seconds() < 1.0;
                bool grace = (t - drive_start_).seconds() < 2.0;
                if (state_ == State::DRIVE && fb_ok && !grace && pwm > 10 && speed_ < stuck_speed_) {
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
                bias_ += std::clamp(-bias_, -step, step);
                pwm = 0;
                steer = last_steer_;
            } else {
                start_reverse("garis hilang", t);
            }
        }

        if (state_ == State::REVERSE) {
            double el = (t - rev_start_).seconds();
            pwm = -reverse_pwm_;

            if (rev_obs_) {
                // Mundur sambil membelokkan roda ke sisi berlawanan,
                // supaya moncong mobil berayun ke arah avoid_dir_
                steer = static_cast<int>(std::lround(-avoid_dir_ * obs_rev_steer_));

                const bool scan_ok = (t - last_scan_).seconds() < 1.0;
                const bool clear = scan_ok && obs_dist_ >= obs_rev_clear_;

                if ((el > obs_rev_min_ && clear) || el > obs_rev_max_) {
                    RCLCPP_INFO(get_logger(), "Selesai mundur dari objek (%s), coba maju lagi",
                                clear ? "jarak cukup" : "waktu habis");
                    state_ = State::DRIVE;
                    rev_obs_ = false;
                    too_close_ = false;
                    drive_start_ = t;
                    de_ = 0.0;
                    stuck_ = false;
                    steer_age_ = 0.0;
                    last_steer_ = static_cast<int>(std::lround(avoid_dir_ * obs_min_steer_));
                    travelled_ = 0.0;
                    // obs_active_ dan bias_ sengaja DIPERTAHANKAN: objek masih harus dihindari
                    pwm = 0;
                    steer = 0;
                }
            } else {
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
                    obs_active_ = false;
                    bias_ = 0.0;
                    pwm = 0;
                    steer = 0;
                }
            }
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "%s lost=%.2f e=%.2f spd=%.1f obs=%.2f pwm=%d steer=%d",
                             state_ == State::DRIVE ? "DRIVE" : "REVERSE",
                             lost, e_, speed_, obs_dist_, pwm, steer);

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

    // obstacle avoidance
    double obs_stop_, obs_min_, obs_half_width_, obs_avoid_, car_half_w_;
    double obs_ramp_, obs_slow_, pass_dist_, speed_scale_, max_hold_;
    double obs_dist_ = 1e9;
    double avoid_dir_ = 0.0;   // +1 = geser kanan, -1 = geser kiri
    double bias_ = 0.0;
    double travelled_ = 0.0;
    bool obs_active_ = false;
    double obs_steer_cmd_;
    double obs_min_steer_;
    double obs_emergency_dist_;
    int obs_creep_pwm_;

    // mundur karena objek terlalu dekat
    double obs_rev_steer_, obs_rev_clear_, obs_too_close_time_, obs_rev_min_, obs_rev_max_;
    bool rev_obs_ = false, too_close_ = false;
    rclcpp::Time too_close_since_;

    State state_ = State::DRIVE;
    bool has_lane_ = false, has_fb_ = false, stuck_ = false;
    int last_steer_ = 0;
    double steer_age_ = 0.0;
    double good_ = 0.0, speed_ = 0.0;
    double e_ = 0, de_ = 0, last_e_ = 0, last_t_ = 0;
    double near_ = 0, far_ = 0;
    rclcpp::Time last_valid_, last_fb_, last_scan_, last_obs_seen_, active_since_, drive_start_, rev_start_, stuck_since_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Master>());
    rclcpp::shutdown();
    return 0;
}