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
    require(movement.scroll_x == 0 && movement.scroll_y == 0 &&
                movement.left_button == ButtonAction::None && movement.right_button == ButtonAction::None,
        "one-finger movement must not create scroll or button output");

    DualPadSnapshot one_finger_tap;
    one_finger_tap.right.fresh_sample = true;
    one_finger_tap.right.single_tap = true;
    const LogicalActions left_click = processor.process(one_finger_tap);
    require(left_click.left_button == ButtonAction::Click,
        "fresh Right one-finger tap must reach the Left-button output");

    DualPadSnapshot two_finger_move;
    two_finger_move.right.fresh_sample = true;
    two_finger_move.right.active = true;
    two_finger_move.right.finger_count = 2;
    two_finger_move.right.movement_reported = true;
    two_finger_move.right.scroll_gesture = true;
    two_finger_move.right.relative_x = -4;
    two_finger_move.right.relative_y = 9;
    const LogicalActions scroll = processor.process(two_finger_move);
    require(scroll.scroll_x == -4 && scroll.scroll_y == 9,
        "fresh Right two-finger movement must reach the scroll output");
    require(scroll.cursor_x == 0 && scroll.cursor_y == 0 &&
                scroll.left_button == ButtonAction::None && scroll.right_button == ButtonAction::None,
        "two-finger movement must not create cursor or button output");

    DualPadSnapshot two_finger_tap;
    two_finger_tap.right.fresh_sample = true;
    two_finger_tap.right.two_finger_tap = true;
    const LogicalActions right_click = processor.process(two_finger_tap);
    require(right_click.right_button == ButtonAction::Click,
        "fresh Right two-finger tap must reach the Right-button output");

    moving_right.right.fresh_sample = false;
    moving_right.right.single_tap = true;
    moving_right.right.two_finger_tap = true;
    const LogicalActions retained = processor.process(moving_right);
    require(retained.cursor_x == 0 && retained.cursor_y == 0 && retained.scroll_x == 0 &&
                retained.scroll_y == 0 && retained.left_button == ButtonAction::None &&
                retained.right_button == ButtonAction::None,
        "retained state must not replay movement or gesture events");

    moving_right.right.fresh_sample = true;
    moving_right.right.finger_count = 3;
    const LogicalActions three_finger = processor.process(moving_right);
    require(three_finger.cursor_x == 0 && three_finger.cursor_y == 0 && three_finger.scroll_x == 0 &&
                three_finger.scroll_y == 0,
        "three-finger timing input must not become one-pad cursor or scroll output");

    std::cout << "PASS tps43_one_pad_processor\n";
    return 0;
}
