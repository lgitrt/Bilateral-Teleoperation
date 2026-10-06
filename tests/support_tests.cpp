#include "teleop_support.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {
int checks = 0;
void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
teleop::Packet packet() {
    teleop::Packet result{};
    result[0] = 2;
    result[1] = 18;
    result[2] = 1;
    for (std::size_t i = 3; i < 15; ++i) {
        result[i] = static_cast<std::uint8_t>(i);
    }
    auto crc = teleop::crc16(result.data(), 15);
    result[15] = static_cast<std::uint8_t>(crc);
    result[16] = static_cast<std::uint8_t>(crc >> 8);
    result[17] = 3;
    return result;
}
}

int main() {
    teleop::SampleDelay<int, 8> delay(3);
    check(delay.update(1) == 0, "delay warmup 1");
    check(delay.update(2) == 0, "delay warmup 2");
    check(delay.update(3) == 0, "delay warmup 3");
    for (int i = 4; i < 200; ++i) {
        check(delay.update(i) == i - 3, "delay wraparound");
    }
    teleop::SampleDelay<int, 1> immediate(0);
    check(immediate.update(17) == 17, "zero delay");
    bool invalid = false;
    try { teleop::SampleDelay<int, 8> too_long(8); }
    catch (const std::invalid_argument&) { invalid = true; }
    check(invalid, "reject oversized delay");

    const std::uint8_t known[] = {'1','2','3','4','5','6','7','8','9'};
    check(teleop::crc16(known, 9) == 0x31c3, "CRC-16/XMODEM test vector");
    teleop::PacketParser parser;
    teleop::Packet decoded{};
    const auto good = packet();
    int received = 0;
    for (auto byte : good) { received += parser.update(byte, decoded); }
    check(received == 1 && good == decoded, "valid packet");
    auto bad = good;
    bad[7] ^= 0x40;
    for (auto byte : bad) { check(!parser.update(byte, decoded), "CRC corruption rejected"); }
    for (auto byte : good) { received += parser.update(byte, decoded); }
    check(received == 2 && good == decoded, "recovery after corruption");
    check(parser.rejected() == 1, "rejected packet count");
    teleop::PacketParser truncated;
    for (unsigned i = 0; i < 8; ++i) { truncated.update(good[i], decoded); }
    received = 0;
    for (auto byte : good) { received += truncated.update(byte, decoded); }
    check(received == 1 && decoded == good, "recovery after truncated packet");
    teleop::PacketParser noise;
    check(!noise.update(0x44, decoded), "ignore leading noise");
    check(!noise.update(2, decoded), "partial start");
    check(!noise.update(2, decoded), "overlapping start");
    received = 0;
    for (std::size_t i = 1; i < good.size(); ++i) {
        received += noise.update(good[i], decoded);
    }
    check(received == 1, "resynchronize overlapping start");

    teleop::LowPassFilter filter(.001, 3.141592653589793);
    double previous = 0;
    for (int i = 0; i < 10000; ++i) {
        const auto value = filter.update(1);
        check(value >= previous && value <= 1, "low-pass stable unit step");
        previous = value;
    }
    check(std::abs(previous - 1) < 1e-10, "low-pass DC convergence");
    invalid = false;
    try { filter.update(std::numeric_limits<double>::quiet_NaN()); }
    catch (const std::invalid_argument&) { invalid = true; }
    check(invalid, "reject non-finite filter input");
    invalid = false;
    try { teleop::LowPassFilter bad_filter(0, 1); }
    catch (const std::invalid_argument&) { invalid = true; }
    check(invalid, "reject invalid filter timing");
    check(teleop::passivity_gain(1, 2, .001, .000001) == 0, "no damping for positive energy");
    check(teleop::passivity_gain(-1, 0, .001, .000001) == 0, "no division by zero");
    const auto gain = teleop::passivity_gain(-.02, .1, .001, .000001);
    check(std::abs(gain * .1 * .1 * .001 - .02) < 1e-12, "dissipation in SI joules");
    invalid = false;
    try { teleop::passivity_gain(-1, 1, 0, .000001); }
    catch (const std::invalid_argument&) { invalid = true; }
    check(invalid, "invalid observer dt");
    invalid = false;
    try { teleop::passivity_gain(-1, 1e-200, .001, 0); }
    catch (const std::overflow_error&) { invalid = true; }
    check(invalid, "report nonrepresentable passivity gain");
    std::cout << checks << " checks passed\n";
}
