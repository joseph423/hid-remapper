#include "dual_tps43_fsm.h"

#include <algorithm>
#include <cstdint>
#include <limits>

DualTps43Fsm::DualTps43Fsm(DualTps43Tuning tuning)
    : tuning_(tuning) {
}

LogicalActions DualTps43Fsm::process(const DualPadSnapshot& snapshot) {
    scroll_source_this_cycle_ = ScrollSource::None;

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

    apply_motion(snapshot, actions);
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
        actions.cursor_x = snapshot.right.three_finger_delta_x;
        actions.cursor_y = snapshot.right.three_finger_delta_y;
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
            actions.cursor_x = snapshot.right.three_finger_delta_x;
            actions.cursor_y = snapshot.right.three_finger_delta_y;
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

void DualTps43Fsm::add_right_cursor(const PadState& right, LogicalActions& actions) {
    actions.cursor_x += right.relative_x;
    actions.cursor_y += right.relative_y;
}

void DualTps43Fsm::add_left_scroll(const PadState& left, LogicalActions& actions) {
    actions.scroll_x += left.relative_x;
    actions.scroll_y += left.relative_y;
    if (left.movement_reported) {
        scroll_source_this_cycle_ = ScrollSource::Left;
    }
}

void DualTps43Fsm::add_right_scroll(const PadState& right, LogicalActions& actions) {
    actions.scroll_x += right.relative_x;
    actions.scroll_y += right.relative_y;
    if (right.movement_reported) {
        scroll_source_this_cycle_ = ScrollSource::Right;
    }
}

void DualTps43Fsm::apply_motion(const DualPadSnapshot& snapshot, LogicalActions& actions) {
    // Momentum belongs to the released touch sequence. A new touch starts a
    // new interaction and must also preserve neutral/locked modes' no-scroll
    // guarantees before that interaction produces any movement.
    const bool new_touch_started = snapshot.left.touch_started || snapshot.right.touch_started;
    if (new_touch_started) {
        stop_scroll_momentum();
    }

    if (actions.cursor_x != 0 || actions.cursor_y != 0) {
        const ScaledDelta cursor = scale_active_delta(
            actions.cursor_x, actions.cursor_y, snapshot.cycle_timestamp_us, tuning_.cursor_gain, cursor_motion_);
        actions.cursor_x = cursor.x;
        actions.cursor_y = cursor.y;
    } else {
        // Velocity history and fractional output must not move the cursor after
        // the normalized input stops.
        stop_cursor_motion(snapshot.cycle_timestamp_us);
    }

    if (scroll_source_this_cycle_ != ScrollSource::None) {
        if (scroll_motion_.source != scroll_source_this_cycle_) {
            const uint64_t last_timestamp_us = scroll_motion_.active_gain.last_timestamp_us;
            scroll_motion_.active_gain = {};
            scroll_motion_.active_gain.last_timestamp_us = last_timestamp_us;
            scroll_motion_.filtered_velocity_x_q8_per_second = 0;
            scroll_motion_.filtered_velocity_y_q8_per_second = 0;
        }

        scroll_motion_.source = scroll_source_this_cycle_;
        stop_scroll_momentum();
        const ScaledDelta scroll = scale_active_delta(
            actions.scroll_x, actions.scroll_y, snapshot.cycle_timestamp_us, tuning_.scroll_gain,
            scroll_motion_.active_gain);
        actions.scroll_x = scroll.x;
        actions.scroll_y = scroll.y;
        update_scroll_release_velocity(scroll);
        return;
    }

    actions.scroll_x = 0;
    actions.scroll_y = 0;
    if (scroll_motion_.source != ScrollSource::None) {
        if (scroll_source_active(snapshot)) {
            // A stationary contact produces no output but contributes a zero
            // velocity sample, preventing stale fast motion from seeding coast.
            const ScaledDelta stationary = scale_active_delta(
                0, 0, snapshot.cycle_timestamp_us, tuning_.scroll_gain, scroll_motion_.active_gain);
            update_scroll_release_velocity(stationary);
            return;
        }

        if (!new_touch_started) {
            start_scroll_momentum(snapshot.cycle_timestamp_us);
        }
        scroll_motion_.source = ScrollSource::None;
        scroll_motion_.active_gain = {};
        return;
    }

    if (scroll_motion_.momentum_active) {
        apply_scroll_momentum(snapshot.cycle_timestamp_us, actions);
    } else {
        // Retain the preceding logical-cycle time before the first scroll
        // sample so its velocity need not use the fallback interval.
        scroll_motion_.active_gain.last_timestamp_us = snapshot.cycle_timestamp_us;
    }
}

DualTps43Fsm::ScaledDelta DualTps43Fsm::scale_active_delta(
    int32_t x,
    int32_t y,
    uint64_t now_us,
    const VelocityGainTuning& tuning,
    VelocityGainState& state) const {
    ScaledDelta result;
    result.sample_interval_us = sample_interval_us(
        now_us, state.last_timestamp_us, tuning.fallback_sample_interval_us);

    const uint64_t magnitude = vector_magnitude(x, y);
    const uint64_t raw_speed = magnitude * 1000000ULL / result.sample_interval_us;
    state.filtered_speed_pad_units_per_second = filter_unsigned(
        state.filtered_speed_pad_units_per_second, raw_speed, tuning.velocity_filter_weight_q8);

    const int32_t gain_q8 = velocity_gain_q8(state.filtered_speed_pad_units_per_second, tuning);
    result.x_q8 = static_cast<int64_t>(x) * gain_q8;
    result.y_q8 = static_cast<int64_t>(y) * gain_q8;
    result.x = scaled_axis(x, gain_q8, state.residual_x_q8);
    result.y = scaled_axis(y, gain_q8, state.residual_y_q8);
    return result;
}

void DualTps43Fsm::stop_cursor_motion(uint64_t now_us) {
    cursor_motion_.last_timestamp_us = now_us;
    cursor_motion_.filtered_speed_pad_units_per_second = 0;
    cursor_motion_.residual_x_q8 = 0;
    cursor_motion_.residual_y_q8 = 0;
}

void DualTps43Fsm::update_scroll_release_velocity(const ScaledDelta& delta) {
    const int64_t instantaneous_x = multiply_divide_saturated(delta.x_q8, 1000000, delta.sample_interval_us);
    const int64_t instantaneous_y = multiply_divide_saturated(delta.y_q8, 1000000, delta.sample_interval_us);
    scroll_motion_.filtered_velocity_x_q8_per_second = filter_signed(
        scroll_motion_.filtered_velocity_x_q8_per_second, instantaneous_x,
        tuning_.scroll_momentum.release_velocity_filter_weight_q8);
    scroll_motion_.filtered_velocity_y_q8_per_second = filter_signed(
        scroll_motion_.filtered_velocity_y_q8_per_second, instantaneous_y,
        tuning_.scroll_momentum.release_velocity_filter_weight_q8);
}

void DualTps43Fsm::start_scroll_momentum(uint64_t now_us) {
    scroll_motion_.momentum_velocity_x_q8_per_second = multiply_divide_saturated(
        scroll_motion_.filtered_velocity_x_q8_per_second, tuning_.scroll_momentum.launch_gain_q8, 256);
    scroll_motion_.momentum_velocity_y_q8_per_second = multiply_divide_saturated(
        scroll_motion_.filtered_velocity_y_q8_per_second, tuning_.scroll_momentum.launch_gain_q8, 256);
    scroll_motion_.momentum_timestamp_us = now_us;
    scroll_motion_.momentum_active = true;

    const int64_t cutoff_q8 = static_cast<int64_t>(tuning_.scroll_momentum.stop_velocity_logical_units_per_second) * 256;
    if (std::max(
            absolute_int64(scroll_motion_.momentum_velocity_x_q8_per_second),
            absolute_int64(scroll_motion_.momentum_velocity_y_q8_per_second)) <= static_cast<uint64_t>(cutoff_q8)) {
        stop_scroll_momentum();
    }
}

void DualTps43Fsm::apply_scroll_momentum(uint64_t now_us, LogicalActions& actions) {
    if (!scroll_motion_.momentum_active) {
        return;
    }

    const uint32_t interval_us = sample_interval_us(
        now_us, scroll_motion_.momentum_timestamp_us, tuning_.scroll_gain.fallback_sample_interval_us);
    const uint16_t decay_q8 = std::min<uint16_t>(tuning_.scroll_momentum.decay_q8, 256);
    scroll_motion_.momentum_velocity_x_q8_per_second = multiply_divide_saturated(
        scroll_motion_.momentum_velocity_x_q8_per_second, decay_q8, 256);
    scroll_motion_.momentum_velocity_y_q8_per_second = multiply_divide_saturated(
        scroll_motion_.momentum_velocity_y_q8_per_second, decay_q8, 256);

    const int64_t cutoff_q8 = static_cast<int64_t>(tuning_.scroll_momentum.stop_velocity_logical_units_per_second) * 256;
    if (std::max(
            absolute_int64(scroll_motion_.momentum_velocity_x_q8_per_second),
            absolute_int64(scroll_motion_.momentum_velocity_y_q8_per_second)) <= static_cast<uint64_t>(cutoff_q8)) {
        stop_scroll_momentum();
        return;
    }

    const int64_t x_q8 = multiply_divide_saturated(
        scroll_motion_.momentum_velocity_x_q8_per_second, interval_us, 1000000);
    const int64_t y_q8 = multiply_divide_saturated(
        scroll_motion_.momentum_velocity_y_q8_per_second, interval_us, 1000000);
    // HID-facing actions are integral, so retain sub-unit displacement until
    // later momentum cycles accumulate enough motion to emit it.
    actions.scroll_x = q8_axis(x_q8, scroll_motion_.momentum_residual_x_q8);
    actions.scroll_y = q8_axis(y_q8, scroll_motion_.momentum_residual_y_q8);
}

void DualTps43Fsm::stop_scroll_momentum() {
    scroll_motion_.momentum_active = false;
    scroll_motion_.momentum_velocity_x_q8_per_second = 0;
    scroll_motion_.momentum_velocity_y_q8_per_second = 0;
    scroll_motion_.momentum_residual_x_q8 = 0;
    scroll_motion_.momentum_residual_y_q8 = 0;
    scroll_motion_.momentum_timestamp_us = 0;
}

bool DualTps43Fsm::scroll_source_active(const DualPadSnapshot& snapshot) const {
    switch (scroll_motion_.source) {
        case ScrollSource::Left:
            return snapshot.left.active;
        case ScrollSource::Right:
            return snapshot.right.active;
        case ScrollSource::None:
            return false;
    }
    return false;
}

uint32_t DualTps43Fsm::sample_interval_us(
    uint64_t now_us,
    uint64_t& last_timestamp_us,
    uint32_t fallback_us) {
    uint64_t interval_us = fallback_us == 0 ? 1 : fallback_us;
    if (last_timestamp_us != 0 && now_us > last_timestamp_us) {
        interval_us = now_us - last_timestamp_us;
    }
    last_timestamp_us = now_us;

    // Very long gaps are irrelevant to velocity estimation. Capping them at
    // the 32-bit unit limit keeps later fixed-point arithmetic bounded.
    return interval_us > std::numeric_limits<uint32_t>::max()
               ? std::numeric_limits<uint32_t>::max()
               : static_cast<uint32_t>(interval_us);
}

uint64_t DualTps43Fsm::vector_magnitude(int32_t x, int32_t y) {
    const uint64_t absolute_x = x < 0 ? static_cast<uint64_t>(-static_cast<int64_t>(x)) : static_cast<uint64_t>(x);
    const uint64_t absolute_y = y < 0 ? static_cast<uint64_t>(-static_cast<int64_t>(y)) : static_cast<uint64_t>(y);
    return integer_square_root(absolute_x * absolute_x + absolute_y * absolute_y);
}

uint64_t DualTps43Fsm::integer_square_root(uint64_t value) {
    uint64_t result = 0;
    uint64_t bit = uint64_t{ 1 } << 62;
    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

uint64_t DualTps43Fsm::filter_unsigned(uint64_t previous, uint64_t current, uint16_t weight_q8) {
    const uint16_t current_weight = std::min<uint16_t>(weight_q8, 256);
    const uint16_t previous_weight = 256 - current_weight;
    return (previous * previous_weight + current * current_weight + 128) / 256;
}

int64_t DualTps43Fsm::filter_signed(int64_t previous, int64_t current, uint16_t weight_q8) {
    const uint16_t current_weight = std::min<uint16_t>(weight_q8, 256);
    const uint16_t previous_weight = 256 - current_weight;
    const int64_t filtered_previous = multiply_divide_saturated(previous, previous_weight, 256);
    const int64_t filtered_current = multiply_divide_saturated(current, current_weight, 256);
    if (filtered_current > 0 && filtered_previous > std::numeric_limits<int64_t>::max() - filtered_current) {
        return std::numeric_limits<int64_t>::max();
    }
    if (filtered_current < 0 && filtered_previous < std::numeric_limits<int64_t>::min() - filtered_current) {
        return std::numeric_limits<int64_t>::min();
    }
    return filtered_previous + filtered_current;
}

uint64_t DualTps43Fsm::absolute_int64(int64_t value) {
    return value < 0 ? static_cast<uint64_t>(-(value + 1)) + 1 : static_cast<uint64_t>(value);
}

int32_t DualTps43Fsm::velocity_gain_q8(
    uint64_t speed_pad_units_per_second,
    const VelocityGainTuning& tuning) {
    const int32_t minimum_gain_q8 = std::max<int32_t>(tuning.minimum_gain_q8, 0);
    const int32_t maximum_gain_q8 = std::max<int32_t>(tuning.maximum_gain_q8, minimum_gain_q8);
    if (tuning.full_gain_velocity_pad_units_per_second == 0 ||
        speed_pad_units_per_second >= tuning.full_gain_velocity_pad_units_per_second) {
        return maximum_gain_q8;
    }

    const int64_t gain_range = static_cast<int64_t>(maximum_gain_q8) - minimum_gain_q8;
    return minimum_gain_q8 + static_cast<int32_t>(
                                 gain_range * speed_pad_units_per_second /
                                 tuning.full_gain_velocity_pad_units_per_second);
}

int32_t DualTps43Fsm::scaled_axis(int32_t input, int32_t gain_q8, int64_t& residual_q8) {
    const int64_t scaled_q8 = static_cast<int64_t>(input) * gain_q8;
    return q8_axis(scaled_q8, residual_q8);
}

int32_t DualTps43Fsm::q8_axis(int64_t delta_q8, int64_t& residual_q8) {
    int64_t total_q8 = delta_q8;
    if (residual_q8 > 0 && total_q8 > std::numeric_limits<int64_t>::max() - residual_q8) {
        total_q8 = std::numeric_limits<int64_t>::max();
    } else if (residual_q8 < 0 && total_q8 < std::numeric_limits<int64_t>::min() - residual_q8) {
        total_q8 = std::numeric_limits<int64_t>::min();
    } else {
        total_q8 += residual_q8;
    }

    const int64_t output = total_q8 / 256;
    residual_q8 = total_q8 - output * 256;
    return saturate_int32(output);
}

int64_t DualTps43Fsm::multiply_divide_saturated(int64_t value, uint32_t multiplier, uint32_t divisor) {
    if (value == 0 || multiplier == 0) {
        return 0;
    }
    if (divisor == 0) {
        return value > 0 ? std::numeric_limits<int64_t>::max() : std::numeric_limits<int64_t>::min();
    }

    const int64_t quotient = value / divisor;
    const int64_t remainder = value % divisor;
    if (quotient > 0 && quotient > std::numeric_limits<int64_t>::max() / multiplier) {
        return std::numeric_limits<int64_t>::max();
    }
    if (quotient < 0 && quotient < std::numeric_limits<int64_t>::min() / multiplier) {
        return std::numeric_limits<int64_t>::min();
    }

    const int64_t scaled_quotient = quotient * multiplier;
    const int64_t scaled_remainder = remainder * multiplier / divisor;
    if (scaled_remainder > 0 && scaled_quotient > std::numeric_limits<int64_t>::max() - scaled_remainder) {
        return std::numeric_limits<int64_t>::max();
    }
    if (scaled_remainder < 0 && scaled_quotient < std::numeric_limits<int64_t>::min() - scaled_remainder) {
        return std::numeric_limits<int64_t>::min();
    }
    return scaled_quotient + scaled_remainder;
}

int32_t DualTps43Fsm::saturate_int32(int64_t value) {
    if (value > std::numeric_limits<int32_t>::max()) {
        return std::numeric_limits<int32_t>::max();
    }
    if (value < std::numeric_limits<int32_t>::min()) {
        return std::numeric_limits<int32_t>::min();
    }
    return static_cast<int32_t>(value);
}
