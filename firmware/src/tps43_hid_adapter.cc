#include "tps43_hid_adapter.h"

#include "platform.h"
#include "remapper.h"
#include "tps43_timing_metrics.h"

void Tps43RemapperActionSink::apply(const LogicalActions& actions) {
    apply_button_action(actions.left_button, left_button_held_, left_release_pending_);
    apply_button_action(actions.right_button, right_button_held_, right_release_pending_);

    const auto q8_or_integer = [](int64_t q8, int32_t integer) {
        return q8 != 0 ? q8 : static_cast<int64_t>(integer) * 256;
    };
    const int64_t cursor_x_q8 = q8_or_integer(actions.cursor_x_q8, actions.cursor_x);
    const int64_t cursor_y_q8 = q8_or_integer(actions.cursor_y_q8, actions.cursor_y);
    const int64_t scroll_x_q8 = q8_or_integer(actions.scroll_x_q8, actions.scroll_x);
    const int64_t scroll_y_q8 = q8_or_integer(actions.scroll_y_q8, actions.scroll_y);
    inject_tps43_output_q8(cursor_x_q8, cursor_y_q8, scroll_x_q8, scroll_y_q8,
        left_button_held_, right_button_held_);
    if (tps43_normal_capture_busy()) {
        tps43_normal_capture_note_scroll_action(get_time(), scroll_x_q8, scroll_y_q8);
    }
}

void Tps43RemapperActionSink::reset() {
    left_button_held_ = false;
    right_button_held_ = false;
    left_release_pending_ = false;
    right_release_pending_ = false;
    reset_tps43_fractional_output();
    inject_tps43_output_q8(0, 0, 0, 0, false, false);
}

void Tps43RemapperActionSink::apply_button_action(
    ButtonAction action,
    bool& held,
    bool& release_pending) {
    if (release_pending) {
        held = false;
        release_pending = false;
    }

    switch (action) {
        case ButtonAction::None:
            break;
        case ButtonAction::Press:
            held = true;
            break;
        case ButtonAction::Release:
            held = false;
            break;
        case ButtonAction::Click:
            held = true;
            release_pending = true;
            break;
    }
}
