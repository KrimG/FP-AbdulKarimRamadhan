#pragma once
#include <endian.h>
#include <algorithm>
#include <cstdint>
#include <cstring>


namespace udp_header {
    constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
    constexpr size_t kCmdSize = 12;
    constexpr size_t kFeedbackSize = 20;
    constexpr int kCmdPort = 12151;
    constexpr int kFeedbackPort = 12152;

    struct Feedback {
        int32_t left_ticks, right_ticks, vel_x_cms, vel_y_cms;
    };

    inline void build_cmd(uint8_t (&buf)[kCmdSize], int32_t pwm, int32_t steer) {
        pwm = std::clamp<int32_t>(pwm, -255, 255);
        steer = std::clamp<int32_t>(steer, -90, 90);
        uint32_t p = htole32(static_cast<uint32_t>(pwm));
        uint32_t s = htole32(static_cast<uint32_t>(steer));
        std::memcpy(buf, kHeader, 4);
        std::memcpy(buf + 4, &p, 4);
        std::memcpy(buf + 8, &s, 4);
    }

    inline int32_t read_i32(const uint8_t *src) {
        uint32_t raw;
        std::memcpy(&raw, src, 4);
        return static_cast<int32_t>(le32toh(raw));
    }

    inline bool parse_feedback(const uint8_t *buf, size_t len, Feedback &out) {
        if (len < kFeedbackSize || std::memcmp(buf, kHeader, 4) !=0) {
            return false;
        }

        else {
        out.left_ticks = read_i32(buf + 4);
        out.right_ticks = read_i32(buf + 8);
        out.vel_x_cms = read_i32(buf + 12);
        out.vel_y_cms = read_i32(buf + 16);
        return true;
        }
    }
}