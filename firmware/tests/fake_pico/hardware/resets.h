#pragma once
#include <cstdint>
constexpr uint32_t RESET_I2C0 = 3, RESET_I2C1 = 4;
struct FakeResets {
    uint32_t reset_done = ~0u;
};
extern FakeResets* resets_hw;
void reset_block_num(uint32_t);
void unreset_block_num(uint32_t);
