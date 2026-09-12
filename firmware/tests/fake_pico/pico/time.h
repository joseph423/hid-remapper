#pragma once
#include <cstdint>
extern uint64_t fake_now;
inline uint64_t time_us_64() {
    return fake_now;
}
inline uint32_t time_us_32() {
    return static_cast<uint32_t>(fake_now);
}
