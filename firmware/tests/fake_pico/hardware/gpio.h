#pragma once
#include <cstdint>
constexpr int GPIO_FUNC_I2C = 3;
constexpr int GPIO_IN = 0;
extern bool fake_rdy;
inline void gpio_set_function(uint8_t, int) {}
inline void gpio_set_pulls(uint8_t, bool, bool) {}
inline void gpio_init(uint8_t) {}
inline void gpio_set_dir(uint8_t, int) {}
#ifdef TPS43_CUSTOM_FAKE_GPIO_GET
bool gpio_get(uint8_t pin);
#else
inline bool gpio_get(uint8_t) {
    return fake_rdy;
}
#endif
