#include "dual_tps43_fsm.h"

#include <cstdint>

DualTps43Fsm::DualTps43Fsm(DualTps43Tuning tuning)
    : tuning_(tuning) {
}

LogicalActions DualTps43Fsm::process(const DualPadSnapshot& snapshot) {
    // Preserve movement history before a new session can reset it. Touch-order
    // decisions need to know whether Right was already moving in the preceding
    // cycle when Left starts or becomes stationary.
    const bool left_was_moving = left_session_.movement_seen;
    const bool right_was_moving = right_session_.movement_seen;

    begin_session_if_needed(snapshot.left, left_session_, snapshot.cycle_timestamp_us);
    begin_session_if_needed(snapshot.right, right_session_, snapshot.cycle_timestamp_us);

    const bool left_was_stationary = previous_left_active_ && !left_was_moving &&
                                     is_stationary(snapshot.left, left_session_, snapshot.cycle_timestamp_us);
    const bool right_was_stationary = previous_right_active_ && !right_was_moving &&
                                      is_stationary(snapshot.right, right_session_, snapshot.cycle_timestamp_us);

    if (snapshot.left.active && snapshot.left.movement_reported) {
        left_session_.movement_seen = true;
    }
    if (snapshot.right.active && snapshot.right.movement_reported) {
        right_session_.movement_seen = true;
    }

    // Dispatch a locked mode before evaluating any Idle transition. This is
    // the mode-priority boundary that prevents ignored input from replacing an
    // active Drag or Left-scroll interaction.
    LogicalActions actions;
    switch (mode_) {
        case Mode::NeutralDualTouch:
            actions = process_neutral(snapshot);
            break;
        case Mode::LeftScroll:
            actions = process_left_scroll(snapshot);
            break;
        case Mode::LeftAssistedDrag:
            actions = process_left_assisted_drag(snapshot);
            break;
        case Mode::RightLatchedDrag:
            actions = process_right_latched_drag(snapshot);
            break;
        case Mode::Idle:
            actions = process_idle(snapshot, left_was_stationary, right_was_stationary,
                previous_right_active_ && right_was_moving);
            break;
    }

    previous_left_active_ = snapshot.left.active;
    previous_right_active_ = snapshot.right.active;
    return actions;
}

void DualTps43Fsm::begin_session_if_needed(const PadState& pad, SessionState& session, uint64_t now_us) {
    if (!pad.touch_started && (pad.session_id == session.id || !pad.active)) {
        return;
    }

    session.id = pad.session_id;
    session.started_us = pad.timestamp_us != 0 ? pad.timestamp_us : now_us;
    session.movement_seen = false;
}

bool DualTps43Fsm::is_stationary(const PadState& pad, const SessionState& session, uint64_t now_us) const {
    return pad.active && !session.movement_seen && now_us >= session.started_us &&
           now_us - session.started_us >= tuning_.stationary_intent_threshold_us;
}

bool DualTps43Fsm::is_eligible_tap(const PadState& pad, const SessionState& session, bool tap_event, bool consumed) const {
    return tap_event && !consumed && session.id != 0 && pad.session_id == session.id &&
           pad.timestamp_us >= session.started_us &&
           pad.timestamp_us - session.started_us <= tuning_.tap_max_duration_us;
}

bool DualTps43Fsm::reached_neutral_threshold(int64_t x, int64_t y) const {
    if (tuning_.neutral_activation_threshold <= 0) {
        return true;
    }

    const uint64_t threshold = static_cast<uint64_t>(tuning_.neutral_activation_threshold);
    // Negating after adding one keeps INT64_MIN representable. The early axis
    // comparison also bounds both operands before the squared-distance sum.
    const uint64_t absolute_x = x < 0 ? static_cast<uint64_t>(-(x + 1)) + 1 : static_cast<uint64_t>(x);
    const uint64_t absolute_y = y < 0 ? static_cast<uint64_t>(-(y + 1)) + 1 : static_cast<uint64_t>(y);
    if (absolute_x >= threshold || absolute_y >= threshold) {
        return true;
    }

    return absolute_x * absolute_x + absolute_y * absolute_y >= threshold * threshold;
}

void DualTps43Fsm::consume_left_session(const PadState& left) {
    consumed_left_session_id_ = left.session_id;
}

void DualTps43Fsm::consume_right_session(const PadState& right) {
    consumed_right_session_id_ = right.session_id;
}

bool DualTps43Fsm::left_session_consumed(const PadState& left) const {
    return left.session_id != 0 && left.session_id == consumed_left_session_id_;
}

bool DualTps43Fsm::right_session_consumed(const PadState& right) const {
    return right.session_id != 0 && right.session_id == consumed_right_session_id_;
}

LogicalActions DualTps43Fsm::process_neutral(const DualPadSnapshot& snapshot) {
    if (!snapshot.left.active || !snapshot.right.active) {
        neutral_left_x_ = 0;
        neutral_left_y_ = 0;
        neutral_right_x_ = 0;
        neutral_right_y_ = 0;
        mode_ = Mode::Idle;
        return {};
    }

    if (snapshot.left.movement_reported) {
        neutral_left_x_ += snapshot.left.relative_x;
        neutral_left_y_ += snapshot.left.relative_y;
    }
    if (snapshot.right.movement_reported) {
        neutral_right_x_ += snapshot.right.relative_x;
        neutral_right_y_ += snapshot.right.relative_y;
    }

    const bool left_reached = reached_neutral_threshold(neutral_left_x_, neutral_left_y_);
    const bool right_reached = reached_neutral_threshold(neutral_right_x_, neutral_right_y_);
    if (!left_reached && !right_reached) {
        return {};
    }

    neutral_left_x_ = 0;
    neutral_left_y_ = 0;
    neutral_right_x_ = 0;
    neutral_right_y_ = 0;

    // Left wins when both flags are true, implementing the same-cycle tie rule
    // without a separate conflict branch.
    if (left_reached) {
        mode_ = Mode::LeftScroll;
        return {};
    }

    mode_ = Mode::LeftAssistedDrag;
    LogicalActions actions;
    actions.left_button = ButtonAction::Press;
    return actions;
}

LogicalActions DualTps43Fsm::process_left_scroll(const DualPadSnapshot& snapshot) {
    consume_left_session(snapshot.left);
    // Determine tap eligibility before this cycle consumes suppressed Right
    // input. A fresh 1-finger tap may click, while a session consumed before
    // entering Left-scroll must remain suppressed on release.
    const bool right_tap = is_eligible_tap(
        snapshot.right, right_session_, snapshot.right.single_tap, right_session_consumed(snapshot.right));
    if ((snapshot.right.active && snapshot.right.finger_count == 2) || snapshot.right.two_finger_tap) {
        consume_right_session(snapshot.right);
    }

    if (!snapshot.left.active) {
        mode_ = Mode::Idle;
        return {};
    }

    LogicalActions actions;
    add_left_scroll(snapshot.left, actions);
    if (snapshot.right.finger_count == 1 && snapshot.right.movement_reported) {
        add_right_cursor(snapshot.right, actions);
    }
    if (right_tap) {
        consume_right_session(snapshot.right);
        actions.left_button = ButtonAction::Click;
    }
    return actions;
}

LogicalActions DualTps43Fsm::process_left_assisted_drag(const DualPadSnapshot& snapshot) {
    consume_left_session(snapshot.left);
    if (snapshot.right.active || snapshot.right.touch_ended || snapshot.right.single_tap || snapshot.right.two_finger_tap) {
        consume_right_session(snapshot.right);
    }

    if (!snapshot.left.active) {
        mode_ = Mode::Idle;
        LogicalActions actions;
        actions.left_button = ButtonAction::Release;
        return actions;
    }

    LogicalActions actions;
    if (snapshot.right.active && snapshot.right.finger_count == 1 && snapshot.right.movement_reported) {
        add_right_cursor(snapshot.right, actions);
    }
    return actions;
}

LogicalActions DualTps43Fsm::process_right_latched_drag(const DualPadSnapshot& snapshot) {
    if (snapshot.left.active || snapshot.left.touch_ended || snapshot.left.single_tap || snapshot.left.two_finger_tap) {
        consume_left_session(snapshot.left);
    }
    consume_right_session(snapshot.right);

    // A fully inactive sample arms eligibility for a later session; changing
    // directly from three fingers to one does not qualify as a distinct drop
    // tap session.
    if (!snapshot.right.active) {
        right_latched_saw_inactive_ = true;
    } else if (snapshot.right.touch_started && right_latched_saw_inactive_ && snapshot.right.finger_count == 1) {
        right_latched_drop_session_id_ = snapshot.right.session_id;
    }

    if (is_eligible_tap(snapshot.right, right_session_, snapshot.right.single_tap, false) &&
        right_latched_drop_session_id_ != 0 &&
        snapshot.right.session_id == right_latched_drop_session_id_ && snapshot.right.session_id == right_session_.id) {
        mode_ = Mode::Idle;
        right_latched_saw_inactive_ = false;
        right_latched_drop_session_id_ = 0;
        LogicalActions actions;
        actions.left_button = ButtonAction::Release;
        return actions;
    }

    LogicalActions actions;
    if (snapshot.right.active && snapshot.right.finger_count == 3 && snapshot.right.movement_reported &&
        snapshot.right.three_finger_delta_valid) {
        actions.cursor_x = snapshot.right.three_finger_delta_x * tuning_.cursor_scale;
        actions.cursor_y = snapshot.right.three_finger_delta_y * tuning_.cursor_scale;
    } else if (snapshot.right.active && snapshot.right.finger_count == 1 && snapshot.right.movement_reported) {
        add_right_cursor(snapshot.right, actions);
    }
    return actions;
}

LogicalActions DualTps43Fsm::process_idle(const DualPadSnapshot& snapshot, bool left_was_stationary, bool right_was_stationary, bool right_was_moving) {
    // The order below is intentional: same-cycle neutral entry, ordered
    // cross-pad taps, mode-entry gestures, and finally ordinary one-pad output.
    if (snapshot.left.touch_started && snapshot.right.touch_started) {
        mode_ = Mode::NeutralDualTouch;
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        return process_neutral(snapshot);
    }

    const bool left_tap =
        is_eligible_tap(snapshot.left, left_session_, snapshot.left.single_tap, left_session_consumed(snapshot.left));
    const bool right_tap =
        is_eligible_tap(snapshot.right, right_session_, snapshot.right.single_tap, right_session_consumed(snapshot.right));
    const bool right_two_finger_tap = is_eligible_tap(
        snapshot.right, right_session_, snapshot.right.two_finger_tap, right_session_consumed(snapshot.right));

    if (left_tap && previous_right_active_) {
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        if (right_was_stationary) {
            LogicalActions actions;
            actions.left_button = ButtonAction::Click;
            return actions;
        }
        return {};
    }

    if (right_tap && previous_left_active_) {
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        if (left_was_stationary) {
            LogicalActions actions;
            actions.right_button = ButtonAction::Click;
            return actions;
        }
        return {};
    }

    const bool left_stationary_now = is_stationary(snapshot.left, left_session_, snapshot.cycle_timestamp_us);

    if (snapshot.left.movement_reported) {
        mode_ = Mode::LeftScroll;
        consume_left_session(snapshot.left);
        if (snapshot.right.active) {
            consume_right_session(snapshot.right);
        }
        LogicalActions actions;
        add_left_scroll(snapshot.left, actions);
        if (snapshot.right.active && snapshot.right.finger_count == 1 && snapshot.right.movement_reported) {
            add_right_cursor(snapshot.right, actions);
        }
        return actions;
    }

    if ((left_was_stationary || left_stationary_now) && snapshot.right.active && snapshot.right.finger_count == 1 &&
        (snapshot.right.movement_reported || right_was_moving)) {
        mode_ = Mode::LeftAssistedDrag;
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        LogicalActions actions;
        if (snapshot.right.movement_reported) {
            add_right_cursor(snapshot.right, actions);
        }
        actions.left_button = ButtonAction::Press;
        return actions;
    }

    if (snapshot.right.active && snapshot.right.finger_count == 3 && snapshot.right.movement_reported &&
        (!snapshot.left.active || left_was_stationary || left_stationary_now)) {
        mode_ = Mode::RightLatchedDrag;
        consume_right_session(snapshot.right);
        if (snapshot.left.active) {
            consume_left_session(snapshot.left);
        }
        right_latched_saw_inactive_ = false;
        right_latched_drop_session_id_ = 0;
        LogicalActions actions;
        actions.left_button = ButtonAction::Press;
        if (snapshot.right.three_finger_delta_valid) {
            actions.cursor_x = snapshot.right.three_finger_delta_x * tuning_.cursor_scale;
            actions.cursor_y = snapshot.right.three_finger_delta_y * tuning_.cursor_scale;
        }
        return actions;
    }

    if (snapshot.left.touch_started && previous_right_active_ && right_was_moving) {
        // The overlap is a suppressed cross-pad interaction even before Left
        // either lifts quickly or remains long enough to enter Drag.
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
    }

    LogicalActions actions;
    if (snapshot.right.active && snapshot.right.finger_count == 1 && snapshot.right.movement_reported) {
        add_right_cursor(snapshot.right, actions);
    } else if (snapshot.right.active && snapshot.right.finger_count == 2 && snapshot.right.movement_reported &&
               snapshot.right.scroll_gesture && (!snapshot.left.active || left_was_stationary || left_stationary_now)) {
        add_right_scroll(snapshot.right, actions);
        consume_right_session(snapshot.right);
    }

    if (right_two_finger_tap &&
        (!snapshot.left.active || left_was_stationary || left_stationary_now)) {
        consume_right_session(snapshot.right);
        actions.right_button = ButtonAction::Click;
    } else if (right_tap && !snapshot.left.active) {
        consume_right_session(snapshot.right);
        actions.left_button = ButtonAction::Click;
    }

    return actions;
}

void DualTps43Fsm::add_right_cursor(const PadState& right, LogicalActions& actions) const {
    actions.cursor_x += right.relative_x * tuning_.cursor_scale;
    actions.cursor_y += right.relative_y * tuning_.cursor_scale;
}

void DualTps43Fsm::add_left_scroll(const PadState& left, LogicalActions& actions) const {
    actions.scroll_x += left.relative_x * tuning_.scroll_scale;
    actions.scroll_y += left.relative_y * tuning_.scroll_scale;
}

void DualTps43Fsm::add_right_scroll(const PadState& right, LogicalActions& actions) const {
    actions.scroll_x += right.relative_x * tuning_.scroll_scale;
    actions.scroll_y += right.relative_y * tuning_.scroll_scale;
}
