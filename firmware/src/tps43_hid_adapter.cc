#include "tps43_hid_adapter.h"

#include "remapper.h"

void Tps43RemapperActionSink::apply(const LogicalActions& actions) {
    apply_button_action(actions.left_button, left_button_held_, left_release_pending_);
    apply_button_action(actions.right_button, right_button_held_, right_release_pending_);

    inject_tps43_output(actions.cursor_x, actions.cursor_y, actions.scroll_x, actions.scroll_y,
        left_button_held_, right_button_held_);
}

void Tps43RemapperActionSink::reset() {
    left_button_held_ = false;
    right_button_held_ = false;
    left_release_pending_ = false;
    right_release_pending_ = false;
    inject_tps43_output(0, 0, 0, 0, false, false);
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
