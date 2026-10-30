#include <bits/stdc++.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

//-------------------- Konstanta BEV & pelacakan garis --------------------
namespace {
constexpr int kBW = 400, kBH = 400;   // ukuran gambar Bird-Eye View (piksel)
constexpr int kBands = 8;             // BEV dibagi 8 pita horizontal (0 = paling dekat mobil)
constexpr int kMargin = 60;           // setengah lebar jendela pencarian garis per pita (piksel)
constexpr int kMinPix = 30;           // minimal piksel putih di jendela agar dianggap ada garis
constexpr double kMinHist = 255.0 * 8;  // minimal total putih di histogram agar garis dianggap ada
}

//-------------------- VISION: kamera -> posisi tengah lajur --------------------
// Fungsi: dari gambar kamera, hitung seberapa jauh mobil dari tengah lajur.
// Output /vision/lane = [offset_dekat, offset_jauh, valid]
//   offset dalam satuan "lebar lajur": 0 = tepat tengah, positif = tengah lajur ada di KANAN mobil
class Vision : public rclcpp::Node {
public:
    Vision() : Node("vision") {
        //-------------------- Parameter (bisa diubah tanpa compile ulang) --------------------
        declare_parameter<int>("v_min", 200);   // batas bawah brightness (V) untuk garis putih
        declare_parameter<int>("s_max", 40);    // batas atas saturasi (S): putih = saturasi rendah
        // 4 titik trapesium jalan di gambar kamera (fraksi lebar/tinggi), urutan: TL, TR, BR, BL
        declare_parameter<std::vector<double>>(
            "src", std::vector<double>{0.50, 0.58, 0.84, 0.58, 1.00, 0.78, 0.20, 0.78});

        //-------------------- Subscriber kamera + publisher output --------------------
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/catvehicle/camera_front/image_raw_front", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });
        pub_bev_ = create_publisher<sensor_msgs::msg::Image>("vision/bev", 10);      // gambar BEV hitam-putih
        pub_lane_ = create_publisher<std_msgs::msg::Float32MultiArray>("vision/lane", 10);  // hasil offset lajur
    }

private:
    using Pts = std::vector<std::optional<double>>;  // posisi x garis per pita (kosong = tidak terdeteksi)

    //-------------------- Telusuri satu garis dari bawah ke atas --------------------
    // x0 = titik awal (dari histogram). Per pita, cari pusat massa piksel putih
    // di jendela sekitar prediksi. dx = perkiraan kemiringan garis, supaya garis
    // putus-putus yang miring tetap bisa diikuti melewati celah.
    Pts trace(const cv::Mat &bev, int x0) const {
        Pts pts(kBands);
        const int bh = bev.rows / kBands;
        double x = x0, dx = 0;
        int gap = 1;   // jumlah pita sejak terakhir garis ditemukan
        for (int i = 0; i < kBands; i++) {
            int y0 = bev.rows - (i + 1) * bh;
            int cx = static_cast<int>(x + dx * gap);   // prediksi posisi garis
            int xa = std::max(0, cx - kMargin);
            int xb = std::min(bev.cols, cx + kMargin);
            if (xb - xa < 1) { 
                gap++; continue; 
            }
            cv::Moments m = cv::moments(bev(cv::Rect(xa, y0, xb - xa, bh)), true);
            if (m.m00 > kMinPix) {
                double nx = xa + m.m10 / m.m00;   // centroid x
                dx = 0.5 * dx + 0.5 * std::clamp((nx - x) / gap, -40.0, 40.0);  // update kemiringan (dihaluskan)
                x = nx;
                gap = 1;
                pts[i] = x;
            } else {
                gap++;   // garis hilang di pita ini (celah garis putus-putus)
            }
        }
        return pts;
    }   

    //-------------------- Callback tiap frame kamera --------------------
    void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        const float W = static_cast<float>(frame.cols);
        const float H = static_cast<float>(frame.rows);

        const int v_min = get_parameter("v_min").as_int();
        const int s_max = get_parameter("s_max").as_int();
        const auto s = get_parameter("src").as_double_array();
        if (s.size() != 8) return;

        //-------------------- 1. Thresholding HSV --------------------
        // Ambil piksel putih/terang (garis jalan): S rendah, V tinggi
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, v_min), cv::Scalar(180, s_max, 255), mask);

        //-------------------- 2. Bird-Eye View (perspective transform) --------------------
        // Trapesium jalan di kamera diubah jadi persegi tampak atas (kBW x kBH)
        std::vector<cv::Point2f> src, dst = {
            {0.f, 0.f}, {static_cast<float>(kBW), 0.f},
            {static_cast<float>(kBW), static_cast<float>(kBH)}, {0.f, static_cast<float>(kBH)}};
        for (int i = 0; i < 4; i++) {
            src.emplace_back(static_cast<float>(s[2 * i] * W), static_cast<float>(s[2 * i + 1] * H));
        }

        cv::Mat M = cv::getPerspectiveTransform(src, dst);
        cv::Mat bev;
        cv::warpPerspective(mask, bev, M, cv::Size(kBW, kBH), cv::INTER_NEAREST);

        //-------------------- 3. Posisi mobil di BEV --------------------
        // Titik tengah bawah gambar kamera ditransformasi ke koordinat BEV (garis kuning)
        std::vector<cv::Point2f> in{cv::Point2f(W / 2, static_cast<float>(s[5] * H))}, out;
        cv::perspectiveTransform(in, out, M);
        const double car_x = out[0].x;

        //-------------------- 4. Cari titik awal garis kiri & kanan --------------------
        // Histogram kolom pada separuh bawah BEV, dibelah di posisi mobil:
        // puncak kiri = garis kiri, puncak kanan = garis kanan
        cv::Mat hist;
        cv::reduce(bev.rowRange(kBH / 2, kBH), hist, 0, cv::REDUCE_SUM, CV_32S);
        const int split = std::clamp(static_cast<int>(car_x), 1, kBW - 1);
        cv::Point lp, rp;
        double lv = 0, rv = 0;
        cv::minMaxLoc(hist.colRange(0, split), nullptr, &lv, nullptr, &lp);
        cv::minMaxLoc(hist.colRange(split, kBW), nullptr, &rv, nullptr, &rp);
        rp.x += split;

        //-------------------- 5. Telusuri garis kiri (L) dan kanan (R) --------------------
        Pts L(kBands), R(kBands);
        if (lv > kMinHist) L = trace(bev, lp.x);
        if (rv > kMinHist) R = trace(bev, rp.x);

        //-------------------- 6. Hitung tengah lajur (C) per pita --------------------
        // Kedua garis ada  -> tengah = rata-rata (sekalian belajar lebar lajur lane_w_)
        // Hanya satu garis -> tengah = garis itu +/- setengah lebar lajur
        Pts C(kBands);
        for (int i = 0; i < kBands; i++) {
            if (L[i] && R[i]) {
                C[i] = (*L[i] + *R[i]) / 2.0;
                double w = *R[i] - *L[i];
                if (w > 60 && w < 350) lane_w_ = 0.95 * lane_w_ + 0.05 * w;  // low-pass lebar lajur
            } else if (L[i]) {
                C[i] = *L[i] + lane_w_ / 2.0;
            } else if (R[i]) {
                C[i] = *R[i] - lane_w_ / 2.0;
            }
        }

        //-------------------- 7. Offset terhadap mobil --------------------
        // pick(a,b): ambil tengah lajur pertama yang terdeteksi di pita a..b-1,
        // lalu ubah ke satuan lebar lajur relatif posisi mobil
        auto pick = [&](int a, int b) -> std::optional<double> {
            for (int i = a; i < b; i++) {
                if (C[i]) return (*C[i] - car_x) / lane_w_;
            }
            return std::nullopt;
        };
        auto e_near = pick(0, 3);   // pita dekat (0-2)
        auto e_far = pick(3, 7);    // pita jauh (3-6), untuk antisipasi tikungan

        //-------------------- 8. Publish /vision/lane = [near, far, valid] --------------------
        std_msgs::msg::Float32MultiArray lane;
        float n = e_near ? static_cast<float>(*e_near) : 0.f;
        float f = e_far ? static_cast<float>(*e_far) : n;   // far kosong -> pakai near
        lane.data = {n, f, (e_near || e_far) ? 1.f : 0.f};
        pub_lane_->publish(lane);

        //-------------------- Debug visual --------------------
        // Biru = garis kiri, Merah = garis kanan, Hijau = tengah lajur, Kuning = posisi mobil
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

        // Gambar trapesium hijau di frame asli (area yang diambil untuk BEV)
        cv::Mat vis = frame.clone();
        std::vector<cv::Point> poly;
        for (auto &p : src) poly.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
        cv::polylines(vis, poly, true, {0, 255, 0}, 2);

        cv::imshow("frame", vis);
        cv::imshow("mask", mask);
        cv::imshow("bev", dbg);
        cv::waitKey(1);   // wajib agar jendela imshow ter-refresh (tutup ini kalau simulasi berat)

        pub_bev_->publish(*cv_bridge::CvImage(msg->header, "mono8", bev).toImageMsg());
    }

    //-------------------- Variabel --------------------
    double lane_w_ = 0.7 * kBW;   // perkiraan lebar lajur di BEV (piksel), dipelajari otomatis
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