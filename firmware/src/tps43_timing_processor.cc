#include "tps43_timing_processor.h"

LogicalActions Tps43OnePadBringupProcessor::process(const DualPadSnapshot& snapshot) {
    LogicalActions actions;
    if (snapshot.right.fresh_sample && snapshot.right.active &&
        snapshot.right.finger_count == 1 && snapshot.right.movement_reported) {
        actions.cursor_x = snapshot.right.relative_x;
        actions.cursor_y = snapshot.right.relative_y;
    }
    return actions;
}
