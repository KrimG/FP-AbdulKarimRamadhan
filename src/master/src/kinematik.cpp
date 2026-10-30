#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

//-------------------- GO TO POINT: Motion-to-Point --------------------
// Fungsi: menggerakkan mobil ke koordinat (x, y) meter dengan
//   - Odometry: dead reckoning dari bicycle model (kecepatan + steering)
//   - Kontrol : pure pursuit
// Frame: (0,0) dan arah 0 = posisi & arah mobil saat node dijalankan. X ke depan, Y ke kiri.
// Target dikirim lewat: ros2 topic pub --once /goal std_msgs/msg/Float32MultiArray "{data: [x, y]}"
class GoToPoint : public rclcpp::Node {
public:
    GoToPoint() : Node("go_to_point") {
        //-------------------- Parameter: Model mobil (perlu dikalibrasi) --------------------
        wheelbase_ = declare_parameter<double>("wheelbase", 2.9);        // jarak sumbu roda depan-belakang (m)
        max_wheel_deg_ = declare_parameter<double>("max_wheel_deg", 30.0);  // sudut roda maksimum untuk steer = 90
        vel_scale_ = declare_parameter<double>("vel_scale", 0.01);       // konversi vel_x feedback (cm/s) -> m/s

        //-------------------- Parameter: Kontrol kecepatan & target --------------------
        pwm_max_ = declare_parameter<int>("pwm_max", 60);       // PWM jauh dari target
        pwm_min_ = declare_parameter<int>("pwm_min", 20);       // PWM saat mendekati target
        slow_dist_ = declare_parameter<double>("slow_dist", 2.0);  // mulai melambat pada jarak ini (m)
        tol_ = declare_parameter<double>("tol", 0.4);           // jarak (m) dianggap sudah sampai

        // Target awal lewat parameter (opsional); biasanya lewat topic /goal
        double tx = declare_parameter<double>("tx", 0.0);
        double ty = declare_parameter<double>("ty", 0.0);
        if (tx != 0.0 || ty != 0.0) {
            set_goal(tx, ty);
        }

        //-------------------- Subscriber & Publisher --------------------
        // /feedback  -> dipakai untuk odometry
        // /goal      -> target baru [x, y]
        // /cmd_drive -> perintah ke sender (JANGAN jalan bersamaan dengan master_node / keyboard)
        fb_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "feedback", 10,
            [this](std_msgs::msg::Int32MultiArray::SharedPtr m) { on_feedback(*m); });
        goal_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "goal", 10,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr m) {
                if (m->data.size() >= 2) {
                    set_goal(m->data[0], m->data[1]);
                }
            });
        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("cmd_drive", 10);
        timer_ = create_wall_timer(50ms, [this] { tick(); });   // loop kontrol 20 Hz
        last_t_ = now();
    }

private:
    //-------------------- Set target baru --------------------
    void set_goal(double x, double y) {
        gx_ = x;
        gy_ = y;
        active_ = true;
        RCLCPP_INFO(get_logger(), "Target baru: (%.2f, %.2f) m", x, y);
    }

    //-------------------- ODOMETRY: dead reckoning (bicycle model) --------------------
    // Dipanggil tiap feedback datang. Posisi dihitung dengan mengintegrasikan kecepatan.
    void on_feedback(const std_msgs::msg::Int32MultiArray &m) {
        if (m.data.size() < 4) {
            return;
        }
        rclcpp::Time t = now();
        double dt = (t - last_t_).seconds();
        last_t_ = t;
        if (dt <= 0.0 || dt > 0.5) {
            return;   // abaikan dt tidak wajar (pertama kali / data terlambat)
        }
        v_ = m.data[2] * vel_scale_;   // kecepatan maju (m/s) dari vel_x

        // Steering (-90..90) -> sudut roda (rad)
        double delta = steer_cmd_ / 90.0 * max_wheel_deg_ * M_PI / 180.0;

        // theta dikurangi karena steering positif = KANAN (theta positif = kiri)
        theta_ -= v_ / wheelbase_ * std::tan(delta) * dt;
        x_ += v_ * std::cos(theta_) * dt;
        y_ += v_ * std::sin(theta_) * dt;
    }

    //-------------------- KONTROL: pure pursuit (20 Hz) --------------------
    void tick() {
        double dx = gx_ - x_, dy = gy_ - y_;
        double dist = std::hypot(dx, dy);   // jarak ke target

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "pos=(%.2f, %.2f) th=%.0fdeg v=%.2f dist=%.2f",
                             x_, y_, theta_ * 180.0 / M_PI, v_, dist);
        if (!active_) {
            return;   // tidak ada target aktif
        }

        int pwm = 0, steer = 0;
        if (dist < tol_) {
            //-------------------- Sampai di target: berhenti --------------------
            active_ = false;
            RCLCPP_INFO(get_logger(), "SAMPAI. pos=(%.2f, %.2f), sisa jarak %.2f m",
                        x_, y_, dist);
        } else {
            //-------------------- Hitung steering (pure pursuit) --------------------
            // alpha = sudut target relatif arah hadap mobil, dinormalisasi ke -pi..pi
            double alpha = std::atan2(dy, dx) - theta_;
            alpha = std::atan2(std::sin(alpha), std::cos(alpha));

            // delta = atan(2 L sin(alpha) / jarak) : sudut roda agar melengkung ke target
            double delta_left = std::atan2(2.0 * wheelbase_ * std::sin(alpha),
                                           std::max(dist, 0.5));
            // Tanda minus: positif di sistem steering mobil = kanan
            double delta_deg = -delta_left * 180.0 / M_PI;
            steer = static_cast<int>(std::lround(
                std::clamp(delta_deg / max_wheel_deg_ * 90.0, -90.0, 90.0)));

            //-------------------- Hitung kecepatan --------------------
            // k = makin dekat target makin pelan; turn = makin besar sudut belok makin pelan
            double k = std::clamp(dist / slow_dist_, 0.0, 1.0);
            double turn = 1.0 - 0.5 * std::min(1.0, std::abs(alpha) / (M_PI / 2));
            pwm = pwm_min_ + static_cast<int>((pwm_max_ - pwm_min_) * k * turn);
        }
        steer_cmd_ = steer;   // disimpan untuk odometry tick berikutnya

        std_msgs::msg::Int32MultiArray msg;
        msg.data = {pwm, steer};
        pub_->publish(msg);
    }

    //-------------------- Variabel --------------------
    double wheelbase_, max_wheel_deg_, vel_scale_, slow_dist_, tol_;
    int pwm_max_, pwm_min_;
    double x_ = 0, y_ = 0, theta_ = 0, v_ = 0;   // estimasi posisi (m), arah (rad), kecepatan (m/s)
    double gx_ = 0, gy_ = 0;                     // target
    int steer_cmd_ = 0;                          // steering terakhir yang dikirim
    bool active_ = false;                        // true = sedang menuju target
    rclcpp::Time last_t_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fb_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr goal_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPoint>());
    rclcpp::shutdown();
    return 0;
}