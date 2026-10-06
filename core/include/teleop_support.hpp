#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace teleop {

// Offline reference delay: 1000 samples at a nominal 1 kHz is one second.
template <typename Sample, std::size_t Capacity>
class SampleDelay {
public:
    explicit SampleDelay(std::size_t samples) : delay_(samples) {
        if (samples >= Capacity) {
            throw std::invalid_argument("Delay exceeds fixed buffer capacity");
        }
    }

    Sample update(const Sample& input) {
        const auto read = (write_ + Capacity - delay_) % Capacity;
        const auto output = delay_ == 0 ? input : samples_[read];
        samples_[write_] = input;
        write_ = (write_ + 1) % Capacity;
        return output;
    }

private:
    std::array<Sample, Capacity> samples_{};
    std::size_t delay_;
    std::size_t write_ = 0;
};

inline std::uint16_t crc16(const std::uint8_t* bytes, std::size_t size) {
    std::uint16_t crc = 0;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(bytes[i]) << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = static_cast<std::uint16_t>(
                (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
        }
    }
    return crc;
}

using Packet = std::array<std::uint8_t, 18>;

class PacketParser {
public:
    bool update(std::uint8_t byte, Packet& packet) {
        if (count_ == 0 && byte != 0x02) {
            return false;
        }
        pending_[count_++] = byte;
        if (count_ == 2 && static_cast<std::size_t>(pending_[1]) != pending_.size()) {
            count_ = byte == 0x02 ? 1 : 0;
            pending_[0] = 0x02;
            ++rejected_;
            return false;
        }
        if (count_ != pending_.size()) {
            return false;
        }
        const auto expected = static_cast<std::uint16_t>(
            pending_[15] | (static_cast<std::uint16_t>(pending_[16]) << 8));
        if (pending_[17] == 0x03 && crc16(pending_.data(), 15) == expected) {
            packet = pending_;
            count_ = 0;
            return true;
        }
        ++rejected_;
        // Retain a possible start within a corrupt/truncated frame.
        std::size_t start = 1;
        while (start < pending_.size() &&
               !(pending_[start] == 0x02 &&
                 (start + 1 == pending_.size() || pending_[start + 1] == 18))) {
            ++start;
        }
        count_ = pending_.size() - start;
        for (std::size_t i = 0; i < count_; ++i) {
            pending_[i] = pending_[start + i];
        }
        return false;
    }

    std::size_t rejected() const { return rejected_; }

private:
    Packet pending_{};
    std::size_t count_ = 0;
    std::size_t rejected_ = 0;
};

class LowPassFilter {
public:
    LowPassFilter(double sample_time, double angular_cutoff)
        : decay_(std::exp(-sample_time * angular_cutoff)) {
        if (!std::isfinite(sample_time) || !std::isfinite(angular_cutoff) ||
            sample_time <= 0 || angular_cutoff <= 0) {
            throw std::invalid_argument("Filter requires positive finite timing and cutoff");
        }
    }

    double update(double value) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("Filter input must be finite");
        }
        return output_ = decay_ * output_ + (1 - decay_) * value;
    }

private:
    double decay_;
    double output_ = 0;
};

inline double passivity_gain(double energy, double flow, double dt,
                             double threshold) {
    if (!std::isfinite(energy) || !std::isfinite(flow) ||
        !std::isfinite(dt) || dt <= 0 || threshold < 0 ||
        !std::isfinite(threshold)) {
        throw std::invalid_argument("Invalid passivity observer input");
    }
    const auto gain = energy < 0 && std::abs(flow) > threshold
        ? -energy / (dt * flow * flow) : 0;
    if (!std::isfinite(gain)) {
        throw std::overflow_error("Passivity gain is not representable");
    }
    return gain;
}

} // namespace teleop
