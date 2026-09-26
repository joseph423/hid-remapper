#include "dual_tps43_fsm.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

constexpr uint32_t kFallbackMotionSampleIntervalUs = 15000;
// Release samples older than 100 ms cannot represent the lift velocity.
constexpr uint64_t kReleaseVelocityExpiryUs = 100000;
// A delayed scheduler tick must not replay a long, unseen coast.
constexpr uint64_t kMomentumGapCancelUs = 250000;
constexpr uint32_t kMomentumStopVelocityQ8PerSecond = 256;
constexpr uint16_t kReleaseVelocityFilterWeightQ8 = 128;
constexpr int64_t kMomentumMaxLaunchVelocityQ8PerSecond = 100000LL * 256;
constexpr double kQ32PerQ8 = 16777216.0;
constexpr double kNaturalLogTwo = 0.6931471805599453;

bool qualifies_left_assisted_drag(const PadState& right, int32_t threshold) {
    if (!right.movement_reported || threshold <= 0) {
        return false;
    }

    const int64_t absolute_x = right.relative_x < 0 ? -static_cast<int64_t>(right.relative_x) : right.relative_x;
    const int64_t absolute_y = right.relative_y < 0 ? -static_cast<int64_t>(right.relative_y) : right.relative_y;
    return absolute_x >= threshold || absolute_y >= threshold;
}

bool has_cursor_input(const PadState& pad) {
    return pad.movement_reported || pad.relative_x != 0 || pad.relative_y != 0;
}

#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
uint64_t absolute_int64_value(int64_t value) {
    return value < 0 ? static_cast<uint64_t>(-(value + 1)) + 1 : static_cast<uint64_t>(value);
}
#endif

}  // namespace

DualTps43Fsm::DualTps43Fsm(DualTps43Tuning tuning)
    : tuning_(tuning) {
}

void DualTps43Fsm::set_tuning(DualTps43Tuning tuning) {
    tuning_ = tuning;
    reset();
}

void DualTps43Fsm::reset() {
    mode_ = Mode::Idle;
    left_session_ = {};
    right_session_ = {};
    previous_left_active_ = false;
    previous_right_active_ = false;
    consumed_left_session_id_ = 0;
    consumed_right_session_id_ = 0;
    neutral_left_x_ = 0;
    neutral_left_y_ = 0;
    neutral_right_x_ = 0;
    neutral_right_y_ = 0;
    right_latched_saw_inactive_ = false;
    right_latched_drop_session_id_ = 0;
    cursor_motion_ = {};
    reset_cursor_temporal_filter();
    pending_cursor_x_ = 0;
    pending_cursor_y_ = 0;
    pending_cursor_since_us_ = 0;
    clear_pending_left_scroll();
    scroll_sample_interval_override_us_ = 0;
    scroll_motion_ = {};
    scroll_source_this_cycle_ = ScrollSource::None;
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    scroll_direction_ = {};
#endif
}

LogicalActions DualTps43Fsm::process(const DualPadSnapshot& snapshot) {
    scroll_source_this_cycle_ = ScrollSource::None;
    scroll_sample_interval_override_us_ = 0;
    if (snapshot.left.touch_started) {
        clear_pending_left_scroll();
    }
    if (snapshot.right.touch_started) {
        reset_cursor_temporal_filter();
    }
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    if (snapshot.left.touch_started) {
        reset_scroll_direction(ScrollSource::Left);
    }
    if (snapshot.right.touch_started) {
        reset_scroll_direction(ScrollSource::Right);
    }
#endif
    if (snapshot.right.touch_started || snapshot.right.touch_ended) {
        pending_cursor_x_ = 0;
        pending_cursor_y_ = 0;
        pending_cursor_since_us_ = 0;
    }

    // Preserve movement history before a new session can reset it. Touch-order
    // decisions need to know whether Right was already moving in the preceding
    // cycle when Left starts or becomes stationary.
    const bool left_was_moving = left_session_.movement_seen;
    const bool right_was_moving = right_session_.movement_seen;

    begin_session_if_needed(snapshot.left, left_session_, snapshot.cycle_timestamp_us,
        previous_right_active_ && snapshot.right.active && !snapshot.right.touch_started ? snapshot.right.session_id : 0);
    begin_session_if_needed(snapshot.right, right_session_, snapshot.cycle_timestamp_us,
        previous_left_active_ && snapshot.left.active && !snapshot.left.touch_started ? snapshot.left.session_id : 0);

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
    if (snapshot.right.active && qualifies_left_assisted_drag(snapshot.right, tuning_.left_assisted_drag_axis_threshold)) {
        right_session_.drag_movement_qualified = true;
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
                previous_right_active_ && right_was_moving, right_session_.drag_movement_qualified);
            break;
    }

#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    flush_pending_scroll_direction(snapshot, actions);
#endif
    apply_motion(snapshot, actions);
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    if (snapshot.left.touch_ended) {
        reset_scroll_direction(ScrollSource::Left);
    }
    if (snapshot.right.touch_ended) {
        reset_scroll_direction(ScrollSource::Right);
    }
#endif
    if (snapshot.left.touch_ended) {
        clear_pending_left_scroll();
    }
    actions.right_touch_active = snapshot.right.active;
    actions.right_touch_started = snapshot.right.touch_started;
    actions.right_touch_ended = snapshot.right.touch_ended;
    actions.right_touch_finger_count = snapshot.right.finger_count;
    actions.right_touch_relative_x = snapshot.right.relative_x;
    actions.right_touch_relative_y = snapshot.right.relative_y;
    previous_left_active_ = snapshot.left.active;
    previous_right_active_ = snapshot.right.active;
    return actions;
}

void DualTps43Fsm::begin_session_if_needed(const PadState& pad, SessionState& session, uint64_t now_us, uint32_t preceding_other_session_id) {
    if (!pad.touch_started && (pad.session_id == session.id || !pad.active)) {
        return;
    }

    session.id = pad.session_id;
    session.started_us = pad.timestamp_us != 0 ? pad.timestamp_us : now_us;
    session.movement_seen = false;
    session.drag_movement_qualified = false;
    session.preceding_other_session_id = preceding_other_session_id;
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
    if (snapshot.right.finger_count == 1 && has_cursor_input(snapshot.right)) {
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
    if (snapshot.right.active && snapshot.right.finger_count == 1 && has_cursor_input(snapshot.right)) {
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
    } else if (snapshot.right.active && snapshot.right.finger_count == 1 && has_cursor_input(snapshot.right)) {
        add_right_cursor(snapshot.right, actions);
    }
    return actions;
}

LogicalActions DualTps43Fsm::process_idle(const DualPadSnapshot& snapshot, bool left_was_stationary, bool right_was_stationary,
    bool right_was_moving, bool right_drag_movement_qualified) {
    // The order below is intentional: same-cycle neutral entry, ordered
    // cross-pad taps, mode-entry gestures, and finally ordinary one-pad output.
    if (snapshot.left.touch_started && snapshot.right.touch_started) {
        clear_pending_left_scroll();
        mode_ = Mode::NeutralDualTouch;
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        return process_neutral(snapshot);
    }

    // Releasing the earlier unmoved touch abandons this stationary overlap,
    // even without a sensor tap event (for example after a long hold). Consume
    // releases only: the remaining pad can still move or host a new tap session.
    const bool earlier_left_released = snapshot.left.touch_ended && snapshot.right.active &&
                                       right_session_.preceding_other_session_id == snapshot.left.session_id;
    const bool earlier_right_released = snapshot.right.touch_ended && snapshot.left.active &&
                                        left_session_.preceding_other_session_id == snapshot.right.session_id;
    if ((earlier_left_released || earlier_right_released) &&
        !left_session_.movement_seen && !right_session_.movement_seen) {
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        return {};
    }

    const bool left_tap =
        is_eligible_tap(snapshot.left, left_session_, snapshot.left.single_tap, left_session_consumed(snapshot.left));
    const bool right_tap =
        is_eligible_tap(snapshot.right, right_session_, snapshot.right.single_tap, right_session_consumed(snapshot.right));
    const bool right_two_finger_tap = is_eligible_tap(
        snapshot.right, right_session_, snapshot.right.two_finger_tap, right_session_consumed(snapshot.right));

    if (left_tap && previous_right_active_) {
        clear_pending_left_scroll();
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        if (right_was_stationary && left_session_.preceding_other_session_id == snapshot.right.session_id) {
            LogicalActions actions;
            actions.left_button = ButtonAction::Click;
            return actions;
        }
        return {};
    }

    if (right_tap && previous_left_active_) {
        clear_pending_left_scroll();
        consume_left_session(snapshot.left);
        consume_right_session(snapshot.right);
        if (left_was_stationary && right_session_.preceding_other_session_id == snapshot.left.session_id) {
            LogicalActions actions;
            actions.right_button = ButtonAction::Click;
            return actions;
        }
        return {};
    }

    const bool left_stationary_now = is_stationary(snapshot.left, left_session_, snapshot.cycle_timestamp_us);

    // Keep deltas gathered before the existing Left-scroll intent threshold.
    // A later movement-status sample confirms intent; without that confirmation
    // the buffer is discarded at lift or when another gesture takes priority.
    if (snapshot.left.active && snapshot.left.fresh_sample && !snapshot.left.movement_reported &&
        (pending_left_scroll_x_ != 0 || pending_left_scroll_y_ != 0 ||
            snapshot.left.relative_x != 0 || snapshot.left.relative_y != 0)) {
        pending_left_scroll_x_ = saturate_int32(
            static_cast<int64_t>(pending_left_scroll_x_) + snapshot.left.relative_x);
        pending_left_scroll_y_ = saturate_int32(
            static_cast<int64_t>(pending_left_scroll_y_) + snapshot.left.relative_y);
        const uint64_t interval = snapshot.left.sample_interval_us != 0
                                      ? snapshot.left.sample_interval_us
                                      : kFallbackMotionSampleIntervalUs;
        pending_left_scroll_interval_us_ = static_cast<uint32_t>(std::min<uint64_t>(
            std::numeric_limits<uint32_t>::max(),
            static_cast<uint64_t>(pending_left_scroll_interval_us_) + interval));
        if (pending_left_scroll_x_ == 0 && pending_left_scroll_y_ == 0) {
            pending_left_scroll_interval_us_ = 0;
        }
    }

    if (snapshot.left.movement_reported) {
        mode_ = Mode::LeftScroll;
        // Enter with the same eligibility rules as ongoing Left-scroll: a held
        // Right tap is not suppressed merely by Left movement. Existing release
        // consumption and ignored two-finger input still apply.
        return process_left_scroll(snapshot);
    }

    if ((left_was_stationary || left_stationary_now) && snapshot.right.active && snapshot.right.finger_count == 1 &&
        right_drag_movement_qualified) {
        clear_pending_left_scroll();
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
        clear_pending_left_scroll();
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
        clear_pending_left_scroll();
    }

    LogicalActions actions;
    if (snapshot.right.active && snapshot.right.finger_count == 1 && has_cursor_input(snapshot.right)) {
        add_right_cursor(snapshot.right, actions);
    } else if (snapshot.right.active && snapshot.right.finger_count == 2 && snapshot.right.scroll_gesture &&
               (!snapshot.left.active || left_was_stationary || left_stationary_now)) {
        clear_pending_left_scroll();
        add_right_scroll(snapshot.right, actions);
        consume_right_session(snapshot.right);
    }

    if (right_two_finger_tap &&
        (!snapshot.left.active || left_was_stationary || left_stationary_now)) {
        clear_pending_left_scroll();
        consume_right_session(snapshot.right);
        actions.right_button = ButtonAction::Click;
    } else if (right_tap && !snapshot.left.active) {
        consume_right_session(snapshot.right);
        actions.left_button = ButtonAction::Click;
    }

    return actions;
}

void DualTps43Fsm::add_right_cursor(const PadState& right, LogicalActions& actions) {
    const uint64_t now_us = right.timestamp_us;
    if ((pending_cursor_x_ != 0 || pending_cursor_y_ != 0) && now_us >= pending_cursor_since_us_ &&
        now_us - pending_cursor_since_us_ > tuning_.subthreshold_cursor_expiry_us) {
        pending_cursor_x_ = 0;
        pending_cursor_y_ = 0;
        pending_cursor_since_us_ = 0;
    }

    if (right.movement_reported) {
        actions.cursor_x += saturate_int32(pending_cursor_x_ + right.relative_x);
        actions.cursor_y += saturate_int32(pending_cursor_y_ + right.relative_y);
        pending_cursor_x_ = 0;
        pending_cursor_y_ = 0;
        pending_cursor_since_us_ = 0;
        return;
    }

    if (pending_cursor_x_ == 0 && pending_cursor_y_ == 0) {
        pending_cursor_since_us_ = now_us;
    }
    pending_cursor_x_ += right.relative_x;
    pending_cursor_y_ += right.relative_y;
    const int32_t threshold = tuning_.subthreshold_cursor_threshold;
    if (pending_cursor_x_ <= -threshold || pending_cursor_x_ >= threshold ||
        pending_cursor_y_ <= -threshold || pending_cursor_y_ >= threshold) {
        actions.cursor_x += saturate_int32(pending_cursor_x_);
        actions.cursor_y += saturate_int32(pending_cursor_y_);
        pending_cursor_x_ = 0;
        pending_cursor_y_ = 0;
        pending_cursor_since_us_ = 0;
    } else if (pending_cursor_x_ == 0 && pending_cursor_y_ == 0) {
        pending_cursor_since_us_ = 0;
    }
}

void DualTps43Fsm::add_left_scroll(const PadState& left, LogicalActions& actions) {
    actions.scroll_x = saturate_int32(
        static_cast<int64_t>(pending_left_scroll_x_) + left.relative_x);
    actions.scroll_y = saturate_int32(
        static_cast<int64_t>(pending_left_scroll_y_) + left.relative_y);
    const uint64_t current_interval = left.sample_interval_us != 0
                                          ? left.sample_interval_us
                                          : kFallbackMotionSampleIntervalUs;
    scroll_sample_interval_override_us_ = static_cast<uint32_t>(std::min<uint64_t>(
        std::numeric_limits<uint32_t>::max(),
        static_cast<uint64_t>(pending_left_scroll_interval_us_) + current_interval));
    clear_pending_left_scroll();
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    const bool classified = classify_scroll_delta(
        ScrollSource::Left, left, scroll_sample_interval_override_us_, actions.scroll_x, actions.scroll_y);
    if (!classified) {
        actions.scroll_x = 0;
        actions.scroll_y = 0;
    }
#endif
    if (left.fresh_sample) {
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
        if (classified) {
            scroll_source_this_cycle_ = ScrollSource::Left;
        }
#else
        scroll_source_this_cycle_ = ScrollSource::Left;
#endif
    }
}

void DualTps43Fsm::add_right_scroll(const PadState& right, LogicalActions& actions) {
    actions.scroll_x += right.relative_x;
    actions.scroll_y += right.relative_y;
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    const bool classified = classify_scroll_delta(
        ScrollSource::Right, right, right.sample_interval_us, actions.scroll_x, actions.scroll_y);
    if (!classified) {
        actions.scroll_x = 0;
        actions.scroll_y = 0;
    }
#endif
    if (right.fresh_sample) {
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
        if (classified) {
            scroll_source_this_cycle_ = ScrollSource::Right;
        }
#else
        scroll_source_this_cycle_ = ScrollSource::Right;
#endif
    }
}

#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
bool DualTps43Fsm::classify_scroll_delta(
    ScrollSource source, const PadState& pad, uint32_t sample_interval_us, int32_t& x, int32_t& y) {
    if (!tuning_.scroll_direction_classification.enabled) {
        reset_scroll_direction(source);
        return true;
    }
    if (!pad.fresh_sample) {
        return true;
    }
    if (scroll_direction_.source != source) {
        scroll_direction_ = {};
        scroll_direction_.source = source;
    }

    if (scroll_direction_.mode == ScrollDirectionMode::Unclassified) {
        if (x == 0 && y == 0) {
            return false;
        }
        scroll_direction_.pending_x += x;
        scroll_direction_.pending_y += y;
        scroll_direction_.pending_interval_us = static_cast<uint32_t>(std::min<uint64_t>(
            std::numeric_limits<uint32_t>::max(),
            static_cast<uint64_t>(scroll_direction_.pending_interval_us) +
                (sample_interval_us != 0 ? sample_interval_us : kFallbackMotionSampleIntervalUs)));

        const uint64_t distance = absolute_int64_value(scroll_direction_.pending_x) +
                                  absolute_int64_value(scroll_direction_.pending_y);
        if (scroll_direction_.pending_x == 0 && scroll_direction_.pending_y == 0) {
            scroll_direction_.pending_interval_us = 0;
        }
        if (distance < tuning_.scroll_direction_classification.classification_distance_counts) {
            return false;
        }

        const uint64_t absolute_x = absolute_int64_value(scroll_direction_.pending_x);
        const uint64_t absolute_y = absolute_int64_value(scroll_direction_.pending_y);
        const uint8_t dominance_ratio = tuning_.scroll_direction_classification.axis_dominance_ratio;
        if (absolute_y >= static_cast<uint64_t>(dominance_ratio) * absolute_x) {
            scroll_direction_.mode = ScrollDirectionMode::Vertical;
        } else if (absolute_x >= static_cast<uint64_t>(dominance_ratio) * absolute_y) {
            scroll_direction_.mode = ScrollDirectionMode::Horizontal;
        } else {
            scroll_direction_.mode = ScrollDirectionMode::Diagonal;
        }
        x = saturate_int32(scroll_direction_.pending_x);
        y = saturate_int32(scroll_direction_.pending_y);
        scroll_sample_interval_override_us_ = scroll_direction_.pending_interval_us;
        scroll_direction_.pending_x = 0;
        scroll_direction_.pending_y = 0;
    }

    switch (scroll_direction_.mode) {
        case ScrollDirectionMode::Vertical:
            x = 0;
            break;
        case ScrollDirectionMode::Horizontal:
            y = 0;
            break;
        case ScrollDirectionMode::Diagonal:
            // Keep the initial classification, but preserve each later sample's actual axis ratio.
            break;
        case ScrollDirectionMode::Unclassified:
            return false;
    }
    return true;
}

void DualTps43Fsm::flush_pending_scroll_direction(
    const DualPadSnapshot& snapshot, LogicalActions& actions) {
    const bool left_ended = snapshot.left.touch_ended && scroll_direction_.source == ScrollSource::Left;
    const bool right_ended = snapshot.right.touch_ended && scroll_direction_.source == ScrollSource::Right;
    if (scroll_direction_.mode != ScrollDirectionMode::Unclassified ||
        (!left_ended && !right_ended) ||
        (scroll_direction_.pending_x == 0 && scroll_direction_.pending_y == 0)) {
        return;
    }

    // A gesture shorter than the startup threshold must still deliver its
    // measured displacement; only direction shaping is skipped for that tail.
    actions.scroll_x = saturate_int32(scroll_direction_.pending_x);
    actions.scroll_y = saturate_int32(scroll_direction_.pending_y);
    scroll_sample_interval_override_us_ = scroll_direction_.pending_interval_us;
    scroll_source_this_cycle_ = scroll_direction_.source;
}

void DualTps43Fsm::reset_scroll_direction(ScrollSource source) {
    if (scroll_direction_.source == source || source == ScrollSource::None) {
        scroll_direction_ = {};
    }
}
#endif

void DualTps43Fsm::clear_pending_left_scroll() {
    pending_left_scroll_x_ = 0;
    pending_left_scroll_y_ = 0;
    pending_left_scroll_interval_us_ = 0;
}

void DualTps43Fsm::reset_cursor_temporal_filter() {
    filtered_cursor_x_q8_ = 0;
    filtered_cursor_y_q8_ = 0;
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
            actions.cursor_x, actions.cursor_y, snapshot.right.sample_interval_us,
            tuning_.cursor_base_scale_q8, cursor_motion_, 100);
        actions.cursor_x = cursor.x;
        actions.cursor_y = cursor.y;
        actions.cursor_x_q8 = cursor.x_q8;
        actions.cursor_y_q8 = cursor.y_q8;
        if (tuning_.cursor_temporal_filter_enabled) {
            // Classify scaled max-axis speed so the bands stay meaningful if
            // the sensor's sample interval changes. Each band has its own
            // configured weight; the weights do not derive from one another.
            const uint32_t sample_interval_us = std::max<uint32_t>(cursor.sample_interval_us, 1);
            const uint64_t x_speed_q8_per_second = absolute_int64(
                multiply_divide_saturated(cursor.x_q8, 1000000, sample_interval_us));
            const uint64_t y_speed_q8_per_second = absolute_int64(
                multiply_divide_saturated(cursor.y_q8, 1000000, sample_interval_us));
            const uint64_t max_axis_speed_q8_per_second = std::max(
                x_speed_q8_per_second, y_speed_q8_per_second);
            uint32_t previous_weight = tuning_.cursor_filter_slow_weight_percent;
            if (max_axis_speed_q8_per_second >
                tuning_.cursor_filter_fast_speed_limit_counts_per_second * 256ULL) {
                previous_weight = tuning_.cursor_filter_fast_weight_percent;
            } else if (max_axis_speed_q8_per_second >
                       tuning_.cursor_filter_slow_speed_limit_counts_per_second * 256ULL) {
                previous_weight = tuning_.cursor_filter_normal_weight_percent;
            }
            const uint32_t current_weight = 100 - previous_weight;
            const auto blend = [current_weight, previous_weight](int64_t current, int64_t previous) {
                // Split before multiplying to keep the weighted sum safe for
                // every representable Q8 cursor delta without floating point.
                return (current / 100) * current_weight + (current % 100) * current_weight / 100 +
                       (previous / 100) * previous_weight + (previous % 100) * previous_weight / 100;
            };
            filtered_cursor_x_q8_ = blend(cursor.x_q8, filtered_cursor_x_q8_);
            filtered_cursor_y_q8_ = blend(cursor.y_q8, filtered_cursor_y_q8_);
            actions.cursor_x_q8 = filtered_cursor_x_q8_;
            actions.cursor_y_q8 = filtered_cursor_y_q8_;
            actions.cursor_x = saturate_int32(filtered_cursor_x_q8_ / 256);
            actions.cursor_y = saturate_int32(filtered_cursor_y_q8_ / 256);
        } else {
            reset_cursor_temporal_filter();
        }
    } else if (snapshot.right.fresh_sample || snapshot.right.touch_ended) {
        // Discard fractional and filter state when input stops rather than
        // emitting a post-stop cursor tail.
        stop_cursor_motion();
    }
    if (snapshot.right.touch_ended) {
        reset_cursor_temporal_filter();
    }

    if (mode_ == Mode::RightLatchedDrag || mode_ == Mode::LeftAssistedDrag) {
        // A finger-count transition can enter either drag without a new touch.
        // Discard the preceding scroll source and its launch history, not just
        // active momentum, so a later lift cannot restart scrolling.
        scroll_motion_ = {};
        actions.scroll_x = actions.scroll_y = 0;
        return;
    }

    if (scroll_source_this_cycle_ != ScrollSource::None) {
        if (scroll_motion_.source != scroll_source_this_cycle_) {
            scroll_motion_.active_scale = {};
            scroll_motion_.gain_scale = {};
            scroll_motion_.filtered_velocity_x_q8_per_second = 0;
            scroll_motion_.filtered_velocity_y_q8_per_second = 0;
        }

        scroll_motion_.source = scroll_source_this_cycle_;
        stop_scroll_momentum();
        const PadState& source = scroll_source_this_cycle_ == ScrollSource::Left ? snapshot.left : snapshot.right;
        const uint64_t sample_interval_us = scroll_sample_interval_override_us_ != 0
                                                ? scroll_sample_interval_override_us_
                                                : source.sample_interval_us;
        const uint16_t gain_percent = active_scroll_gain_percent(
            actions.scroll_x, actions.scroll_y, sample_interval_us);
        const ScaledDelta base_scroll = scale_active_delta(
            actions.scroll_x, actions.scroll_y, sample_interval_us,
            tuning_.scroll_base_scale_q8, scroll_motion_.active_scale, 100);
        const ScaledDelta scroll = tuning_.active_scroll_gain.enabled
                                       ? scale_active_delta(
                                             actions.scroll_x, actions.scroll_y, sample_interval_us,
                                             tuning_.scroll_base_scale_q8, scroll_motion_.gain_scale, gain_percent)
                                       : base_scroll;
        actions.scroll_x = scroll.x;
        actions.scroll_y = scroll.y;
        actions.scroll_x_q8 = scroll.x_q8;
        actions.scroll_y_q8 = scroll.y_q8;
        update_scroll_release_velocity(scroll, snapshot.cycle_timestamp_us);
        return;
    }

    actions.scroll_x = 0;
    actions.scroll_y = 0;
    if (scroll_motion_.source != ScrollSource::None) {
        if (scroll_source_active(snapshot)) {
            const PadState& source = scroll_motion_.source == ScrollSource::Left ? snapshot.left : snapshot.right;
            // No acquisition is not evidence of zero sensor velocity.
            if (!source.fresh_sample) {
                return;
            }
            if (scroll_motion_.source == ScrollSource::Right && source.finger_count != 2) {
                // The sensor can report one remaining finger between releases,
                // including a relative delta from that finger. Preserve the
                // last two-finger velocity only until the 100 ms expiry; the
                // one-finger cursor output above remains unchanged. A different
                // multi-finger gesture abandons the scroll immediately.
                if (source.finger_count != 1) {
                    scroll_motion_ = {};
                }
                return;
            }
            // A stationary contact produces no output but contributes a zero
            // velocity sample, preventing stale fast motion from seeding coast.
            const ScaledDelta stationary = scale_active_delta(
                0, 0, source.sample_interval_us, tuning_.scroll_base_scale_q8,
                scroll_motion_.active_scale, 100);
            update_scroll_release_velocity(stationary, snapshot.cycle_timestamp_us);
            return;
        }

        if (!new_touch_started) {
            start_scroll_momentum(snapshot.cycle_timestamp_us);
        }
        scroll_motion_.source = ScrollSource::None;
        scroll_motion_.active_scale = {};
        return;
    }

    if (scroll_motion_.momentum_active) {
        apply_scroll_momentum(snapshot.cycle_timestamp_us, actions);
    }
}

uint16_t DualTps43Fsm::active_scroll_gain_percent(
    int32_t x,
    int32_t y,
    uint64_t sample_interval_us) const {
    const ActiveScrollGainTuning& tuning = tuning_.active_scroll_gain;
    if (!tuning.enabled || (x == 0 && y == 0)) {
        return 100;
    }

    const uint64_t interval = sample_interval_us != 0 ? sample_interval_us : kFallbackMotionSampleIntervalUs;
    const uint64_t max_axis_delta = std::max(absolute_int64(x), absolute_int64(y));
    const uint64_t speed = max_axis_delta * 1000000 / interval;
    if (speed <= tuning.slow_speed_limit_counts_per_second) {
        return tuning.slow_gain_percent;
    }
    if (speed >= tuning.fast_speed_limit_counts_per_second) {
        return tuning.fast_gain_percent;
    }

    const int32_t gain_delta = static_cast<int32_t>(tuning.fast_gain_percent) -
                               static_cast<int32_t>(tuning.slow_gain_percent);
    const uint64_t speed_span = tuning.fast_speed_limit_counts_per_second -
                                tuning.slow_speed_limit_counts_per_second;
    const uint64_t speed_offset = speed - tuning.slow_speed_limit_counts_per_second;
    const int64_t interpolated_delta = static_cast<int64_t>(gain_delta) *
                                       static_cast<int64_t>(speed_offset) /
                                       static_cast<int64_t>(speed_span);
    return static_cast<uint16_t>(static_cast<int32_t>(tuning.slow_gain_percent) + interpolated_delta);
}

DualTps43Fsm::ScaledDelta DualTps43Fsm::scale_active_delta(
    int32_t x,
    int32_t y,
    uint64_t acquisition_interval_us,
    int32_t base_scale_q8,
    MotionScaleState& state,
    uint16_t gain_percent) const {
    ScaledDelta result;
    const int32_t scale_q8 = std::max<int32_t>(base_scale_q8, 0);
    const uint64_t interval = acquisition_interval_us != 0
                                  ? acquisition_interval_us
                                  : kFallbackMotionSampleIntervalUs;
    result.sample_interval_us = static_cast<uint32_t>(
        std::min<uint64_t>(interval, std::numeric_limits<uint32_t>::max()));

    result.x_q8 = multiply_divide_saturated(static_cast<int64_t>(x) * scale_q8, gain_percent, 100);
    result.y_q8 = multiply_divide_saturated(static_cast<int64_t>(y) * scale_q8, gain_percent, 100);
    result.x = q8_axis(result.x_q8, state.residual_x_q8);
    result.y = q8_axis(result.y_q8, state.residual_y_q8);
    return result;
}

void DualTps43Fsm::stop_cursor_motion() {
    cursor_motion_.residual_x_q8 = 0;
    cursor_motion_.residual_y_q8 = 0;
    reset_cursor_temporal_filter();
}

void DualTps43Fsm::update_scroll_release_velocity(const ScaledDelta& delta, uint64_t now_us) {
    if (scroll_motion_.last_velocity_sample_us != 0 &&
        (now_us < scroll_motion_.last_velocity_sample_us ||
         now_us - scroll_motion_.last_velocity_sample_us > kReleaseVelocityExpiryUs)) {
        scroll_motion_.filtered_velocity_x_q8_per_second = 0;
        scroll_motion_.filtered_velocity_y_q8_per_second = 0;
    }
    const int64_t instantaneous_x = multiply_divide_saturated(delta.x_q8, 1000000, delta.sample_interval_us);
    const int64_t instantaneous_y = multiply_divide_saturated(delta.y_q8, 1000000, delta.sample_interval_us);
    // A fresh stationary sample clears launch history immediately.
    if (delta.x_q8 == 0 && delta.y_q8 == 0) {
        scroll_motion_.filtered_velocity_x_q8_per_second = 0;
        scroll_motion_.filtered_velocity_y_q8_per_second = 0;
        scroll_motion_.last_velocity_sample_us = now_us;
        return;
    }
    // A fresh opposite-direction sample replaces the old direction immediately.
    if ((instantaneous_x > 0 && scroll_motion_.filtered_velocity_x_q8_per_second < 0) ||
        (instantaneous_x < 0 && scroll_motion_.filtered_velocity_x_q8_per_second > 0)) {
        scroll_motion_.filtered_velocity_x_q8_per_second = 0;
    }
    if ((instantaneous_y > 0 && scroll_motion_.filtered_velocity_y_q8_per_second < 0) ||
        (instantaneous_y < 0 && scroll_motion_.filtered_velocity_y_q8_per_second > 0)) {
        scroll_motion_.filtered_velocity_y_q8_per_second = 0;
    }
    scroll_motion_.filtered_velocity_x_q8_per_second = filter_signed(
        scroll_motion_.filtered_velocity_x_q8_per_second, instantaneous_x,
        kReleaseVelocityFilterWeightQ8);
    scroll_motion_.filtered_velocity_y_q8_per_second = filter_signed(
        scroll_motion_.filtered_velocity_y_q8_per_second, instantaneous_y,
        kReleaseVelocityFilterWeightQ8);
    scroll_motion_.last_velocity_sample_us = now_us;
}

void DualTps43Fsm::start_scroll_momentum(uint64_t now_us) {
    if (!tuning_.scroll_momentum.enabled || scroll_motion_.last_velocity_sample_us == 0 ||
        now_us < scroll_motion_.last_velocity_sample_us ||
        now_us - scroll_motion_.last_velocity_sample_us > kReleaseVelocityExpiryUs) {
        stop_scroll_momentum();
        return;
    }
    scroll_motion_.momentum_velocity_x_q8_per_second = multiply_divide_saturated(
        scroll_motion_.filtered_velocity_x_q8_per_second, tuning_.scroll_momentum.launch_strength_percent, 100);
    scroll_motion_.momentum_velocity_y_q8_per_second = multiply_divide_saturated(
        scroll_motion_.filtered_velocity_y_q8_per_second, tuning_.scroll_momentum.launch_strength_percent, 100);
    scroll_motion_.momentum_velocity_x_q8_per_second = std::clamp(scroll_motion_.momentum_velocity_x_q8_per_second,
        -kMomentumMaxLaunchVelocityQ8PerSecond, kMomentumMaxLaunchVelocityQ8PerSecond);
    scroll_motion_.momentum_velocity_y_q8_per_second = std::clamp(scroll_motion_.momentum_velocity_y_q8_per_second,
        -kMomentumMaxLaunchVelocityQ8PerSecond, kMomentumMaxLaunchVelocityQ8PerSecond);
    scroll_motion_.momentum_started_us = now_us;
    scroll_motion_.momentum_timestamp_us = now_us;
    scroll_motion_.momentum_active = true;

    if (std::max(
            absolute_int64(scroll_motion_.momentum_velocity_x_q8_per_second),
            absolute_int64(scroll_motion_.momentum_velocity_y_q8_per_second)) <= kMomentumStopVelocityQ8PerSecond) {
        stop_scroll_momentum();
    }
}

void DualTps43Fsm::apply_scroll_momentum(uint64_t now_us, LogicalActions& actions) {
    if (!scroll_motion_.momentum_active) {
        return;
    }

    if (now_us <= scroll_motion_.momentum_timestamp_us ||
        now_us - scroll_motion_.momentum_timestamp_us > kMomentumGapCancelUs) {
        stop_scroll_momentum();
        return;
    }
    const double half_life_us = static_cast<double>(tuning_.scroll_momentum.half_life_ms) * 1000.0;
    const double previous = std::exp2(-static_cast<double>(scroll_motion_.momentum_timestamp_us -
        scroll_motion_.momentum_started_us) / half_life_us);
    const double current = std::exp2(-static_cast<double>(now_us - scroll_motion_.momentum_started_us) / half_life_us);
    const double integral_seconds = half_life_us / (1000000.0 * kNaturalLogTwo) * (previous - current);
    const auto emit_axis = [integral_seconds](int64_t launch_velocity, int64_t& residual_q32) {
        const double displacement_q32 = static_cast<double>(launch_velocity) * integral_seconds * kQ32PerQ8;
        // The launch cap, 250 ms gap limit, and 1,000 ms maximum half-life
        // bound this increment well inside int64_t.
        const int64_t increment = static_cast<int64_t>(displacement_q32);
        residual_q32 += increment;
        const int64_t output_q8 = residual_q32 / static_cast<int64_t>(kQ32PerQ8);
        residual_q32 %= static_cast<int64_t>(kQ32PerQ8);
        return output_q8;
    };
    actions.scroll_x_q8 = emit_axis(scroll_motion_.momentum_velocity_x_q8_per_second,
        scroll_motion_.momentum_residual_x_q32);
    actions.scroll_y_q8 = emit_axis(scroll_motion_.momentum_velocity_y_q8_per_second,
        scroll_motion_.momentum_residual_y_q32);
    actions.scroll_x = q8_axis(actions.scroll_x_q8, scroll_motion_.momentum_projection_x_q8);
    actions.scroll_y = q8_axis(actions.scroll_y_q8, scroll_motion_.momentum_projection_y_q8);
    scroll_motion_.momentum_timestamp_us = now_us;
    if (std::max(absolute_int64(scroll_motion_.momentum_velocity_x_q8_per_second),
            absolute_int64(scroll_motion_.momentum_velocity_y_q8_per_second)) * current <=
        kMomentumStopVelocityQ8PerSecond) {
        stop_scroll_momentum();
    }
}

void DualTps43Fsm::stop_scroll_momentum() {
    scroll_motion_.momentum_active = false;
    scroll_motion_.momentum_velocity_x_q8_per_second = 0;
    scroll_motion_.momentum_velocity_y_q8_per_second = 0;
    scroll_motion_.momentum_residual_x_q32 = 0;
    scroll_motion_.momentum_residual_y_q32 = 0;
    scroll_motion_.momentum_projection_x_q8 = 0;
    scroll_motion_.momentum_projection_y_q8 = 0;
    scroll_motion_.momentum_started_us = 0;
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
