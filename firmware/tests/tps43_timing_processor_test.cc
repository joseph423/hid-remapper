#include <iostream>
#include <stdexcept>

#include "tps43_timing_processor.h"

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

int main() {
    Tps43OnePadBringupProcessor processor;

    DualPadSnapshot moving_right;
    moving_right.right.fresh_sample = true;
    moving_right.right.active = true;
    moving_right.right.finger_count = 1;
    moving_right.right.movement_reported = true;
    moving_right.right.relative_x = 12;
    moving_right.right.relative_y = -7;
    const LogicalActions movement = processor.process(moving_right);
    require(movement.cursor_x == 12 && movement.cursor_y == -7,
        "fresh Right one-finger movement must reach the logical cursor output");

    moving_right.right.fresh_sample = false;
    const LogicalActions retained = processor.process(moving_right);
    require(retained.cursor_x == 0 && retained.cursor_y == 0,
        "retained state must not replay cursor movement");

    moving_right.right.fresh_sample = true;
    moving_right.right.finger_count = 3;
    const LogicalActions three_finger = processor.process(moving_right);
    require(three_finger.cursor_x == 0 && three_finger.cursor_y == 0,
        "three-finger timing input must not become one-finger cursor output");

    std::cout << "PASS tps43_timing_processor\n";
    return 0;
}
