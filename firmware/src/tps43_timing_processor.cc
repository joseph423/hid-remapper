#include "tps43_timing_processor.h"

LogicalActions Tps43OnePadBringupProcessor::process(const DualPadSnapshot& snapshot) {
    LogicalActions actions;
    if (!snapshot.right.fresh_sample) {
        return actions;
    }

    if (snapshot.right.single_tap) {
        actions.left_button = ButtonAction::Click;
    }
    if (snapshot.right.two_finger_tap) {
        actions.right_button = ButtonAction::Click;
    }

    if (!snapshot.right.active) {
        return actions;
    }

    if (snapshot.right.finger_count == 1) {
        // Relative deltas are valid movement output independently of the
        // sensor's threshold-based movement status. Freshness, activity, and
        // the exact one-finger contact count remain the cursor-path gates.
        actions.cursor_x = snapshot.right.relative_x;
        actions.cursor_y = snapshot.right.relative_y;
    } else if (snapshot.right.finger_count == 2 && snapshot.right.movement_reported &&
               snapshot.right.scroll_gesture) {
        actions.scroll_x = snapshot.right.relative_x;
        actions.scroll_y = snapshot.right.relative_y;
    }
    return actions;
}
