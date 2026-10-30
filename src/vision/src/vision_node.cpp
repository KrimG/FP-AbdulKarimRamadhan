#include <bits/stdc++.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace {
constexpr int kBW = 400, kBH = 400;
constexpr int kBands = 8;
constexpr int kMargin = 60;
constexpr int kMinPix = 30;
constexpr double kMinHist = 255.0 * 8;
}

class Vision : public rclcpp::Node {
public:
    Vision() : Node("vision") {
        declare_parameter<int>("v_min", 200);
        declare_parameter<int>("s_max", 40);
        declare_parameter<std::vector<double>>(
            "src", std::vector<double>{0.50, 0.58, 0.84, 0.58, 1.00, 0.78, 0.20, 0.78});

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/catvehicle/camera_front/image_raw_front", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });
        pub_bev_ = create_publisher<sensor_msgs::msg::Image>("vision/bev", 10);
        pub_lane_ = create_publisher<std_msgs::msg::Float32MultiArray>("vision/lane", 10);
    }

private:
    using Pts = std::vector<std::optional<double>>;

    Pts trace(const cv::Mat &bev, int x0) const {
        Pts pts(kBands);
        const int bh = bev.rows / kBands;
        double x = x0, dx = 0;
        int gap = 1;
        for (int i = 0; i < kBands; i++) {
            int y0 = bev.rows - (i + 1) * bh;
            int cx = static_cast<int>(x + dx * gap);
            int xa = std::max(0, cx - kMargin);
            int xb = std::min(bev.cols, cx + kMargin);
            if (xb - xa < 1) { 
                gap++; continue; 
            }
            cv::Moments m = cv::moments(bev(cv::Rect(xa, y0, xb - xa, bh)), true);
            if (m.m00 > kMinPix) {
                double nx = xa + m.m10 / m.m00;
                dx = 0.5 * dx + 0.5 * std::clamp((nx - x) / gap, -40.0, 40.0);
                x = nx;
                gap = 1;
                pts[i] = x;
            } else {
                gap++;
            }
        }
        return pts;
    }   

    void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        const float W = static_cast<float>(frame.cols);
        const float H = static_cast<float>(frame.rows);

        const int v_min = get_parameter("v_min").as_int();
        const int s_max = get_parameter("s_max").as_int();
        const auto s = get_parameter("src").as_double_array();
        if (s.size() != 8) return;

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, v_min), cv::Scalar(180, s_max, 255), mask);

        std::vector<cv::Point2f> src, dst = {
            {0.f, 0.f}, {static_cast<float>(kBW), 0.f},
            {static_cast<float>(kBW), static_cast<float>(kBH)}, {0.f, static_cast<float>(kBH)}};
        for (int i = 0; i < 4; i++) {
            src.emplace_back(static_cast<float>(s[2 * i] * W), static_cast<float>(s[2 * i + 1] * H));
        }

        cv::Mat M = cv::getPerspectiveTransform(src, dst);
        cv::Mat bev;
        cv::warpPerspective(mask, bev, M, cv::Size(kBW, kBH), cv::INTER_NEAREST);

        std::vector<cv::Point2f> in{cv::Point2f(W / 2, static_cast<float>(s[5] * H))}, out;
        cv::perspectiveTransform(in, out, M);
        const double car_x = out[0].x;

        cv::Mat hist;
        cv::reduce(bev.rowRange(kBH / 2, kBH), hist, 0, cv::REDUCE_SUM, CV_32S);
        const int split = std::clamp(static_cast<int>(car_x), 1, kBW - 1);
        cv::Point lp, rp;
        double lv = 0, rv = 0;
        cv::minMaxLoc(hist.colRange(0, split), nullptr, &lv, nullptr, &lp);
        cv::minMaxLoc(hist.colRange(split, kBW), nullptr, &rv, nullptr, &rp);
        rp.x += split;

        Pts L(kBands), R(kBands);
        if (lv > kMinHist) L = trace(bev, lp.x);
        if (rv > kMinHist) R = trace(bev, rp.x);

        Pts C(kBands);
        for (int i = 0; i < kBands; i++) {
            if (L[i] && R[i]) {
                C[i] = (*L[i] + *R[i]) / 2.0;
                double w = *R[i] - *L[i];
                if (w > 60 && w < 350) lane_w_ = 0.95 * lane_w_ + 0.05 * w;
            } else if (L[i]) {
                C[i] = *L[i] + lane_w_ / 2.0;
            } else if (R[i]) {
                C[i] = *R[i] - lane_w_ / 2.0;
            }
        }

        auto pick = [&](int a, int b) -> std::optional<double> {
            for (int i = a; i < b; i++) {
                if (C[i]) return (*C[i] - car_x) / lane_w_;
            }
            return std::nullopt;
        };
        auto e_near = pick(0, 3);
        auto e_far = pick(3, 7);

        std_msgs::msg::Float32MultiArray lane;
        float n = e_near ? static_cast<float>(*e_near) : 0.f;
        float f = e_far ? static_cast<float>(*e_far) : n;
        lane.data = {n, f, (e_near || e_far) ? 1.f : 0.f};
        pub_lane_->publish(lane);

        cv::Mat dbg;
        cv::cvtColor(bev, dbg, cv::COLOR_GRAY2BGR);
        const int bh = kBH / kBands;
        for (int i = 0; i < kBands; i++) {
            int y = kBH - i * bh - bh / 2;
            if (L[i]) cv::circle(dbg, {static_cast<int>(*L[i]), y}, 5, {255, 0, 0}, -1);
            if (R[i]) cv::circle(dbg, {static_cast<int>(*R[i]), y}, 5, {0, 0, 255}, -1);
            if (C[i]) cv::circle(dbg, {static_cast<int>(*C[i]), y}, 5, {0, 255, 0}, -1);
        }
        cv::line(dbg, {static_cast<int>(car_x), 0}, {static_cast<int>(car_x), kBH}, {0, 255, 255}, 2);

        cv::Mat vis = frame.clone();
        std::vector<cv::Point> poly;
        for (auto &p : src) poly.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
        cv::polylines(vis, poly, true, {0, 255, 0}, 2);

        cv::imshow("frame", vis);
        cv::imshow("mask", mask);
        cv::imshow("bev", dbg);
        cv::waitKey(1);

        pub_bev_->publish(*cv_bridge::CvImage(msg->header, "mono8", bev).toImageMsg());
    }

    double lane_w_ = 0.7 * kBW;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_bev_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_lane_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vision>());
    rclcpp::shutdown();
    return 0;
}