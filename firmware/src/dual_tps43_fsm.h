#ifndef DUAL_TPS43_FSM_H_
#define DUAL_TPS43_FSM_H_

#include <cstdint>

#include "pad_state.h"

struct DualPadSnapshot {
    // Both states belong to the same logical processing cycle; driver service
    // order must not determine touch-order behavior.
    PadState left;
    PadState right;
    uint64_t cycle_timestamp_us = 0;
};

enum class ButtonAction {
    None,
    Press,
    Release,
    Click,
};

struct LogicalActions {
    // Relative logical output for one processing cycle. The FSM does not write
    // HID reports directly.
    int32_t cursor_x = 0;
    int32_t cursor_y = 0;
    int32_t scroll_x = 0;
    int32_t scroll_y = 0;
    // Per-cycle Q8 motion delivered to the Remapper output boundary. The
    // integral fields above remain the compatibility projection used by
    // behavior tests and diagnostics; the adapter accumulates these fields
    // without quantizing them first.
    int64_t cursor_x_q8 = 0;
    int64_t cursor_y_q8 = 0;
    int64_t scroll_x_q8 = 0;
    int64_t scroll_y_q8 = 0;
    // Right-pad touch metadata for the isolated Phase 16 digitizer proof of
    // concept. The default generic-mouse descriptor ignores these fields.
    bool right_touch_active = false;
    bool right_touch_started = false;
    bool right_touch_ended = false;
    uint8_t right_touch_finger_count = 0;
    int32_t right_touch_relative_x = 0;
    int32_t right_touch_relative_y = 0;
    ButtonAction left_button = ButtonAction::None;
    ButtonAction right_button = ButtonAction::None;
};

// Configures post-release scroll momentum. Velocity uses logical output units
// per second in Q8 fixed point. Q8 filter, launch, and decay coefficients use
// 256 as 1.0; decay must be below 256 for momentum to reach the cutoff.
struct ScrollMomentumTuning {
    // Current scaled-scroll velocity sample weight in Q8.
    uint16_t release_velocity_filter_weight_q8;
    // Multiplier applied to filtered release velocity, in Q8.
    uint16_t launch_gain_q8;
    // Velocity retained after each processing cycle, in Q8.
    uint16_t decay_q8;
    // Logical output units per second below which momentum stops.
    uint32_t stop_velocity_logical_units_per_second;
};

// Optional active-scroll response curve. Speed is measured from normalized
// pad displacement per sensor sample interval; gains are percentages of the
// configured scroll base scale and interpolate between the two speed limits.
struct ActiveScrollGainTuning {
    bool enabled = false;
    uint16_t slow_speed_limit_counts_per_second = 200;
    uint16_t fast_speed_limit_counts_per_second = 1000;
    uint16_t slow_gain_percent = 200;
    uint16_t fast_gain_percent = 50;
};

// Optional gesture-start classifier for active scroll axis locking.
struct ScrollDirectionClassificationTuning {
    bool enabled = true;
    uint8_t classification_distance_counts = 8;
    uint8_t axis_dominance_ratio = 2;
};

// Contains all hardware-independent behavior and motion tuning. Values used by
// host tests are deterministic fixtures, not approved physical tuning.
struct DualTps43Tuning {
    // Microseconds from touch start through the reported tap event.
    uint64_t tap_max_duration_us;
    // Microseconds an unmoved touch must remain active to express stationary intent.
    uint64_t stationary_intent_threshold_us;
    // Does not apply to Right-latched Drag entry, which uses the sensor's first
    // reported three-finger movement without an extra distance threshold. Unit:
    // normalized pad displacement.
    int32_t neutral_activation_threshold;
    // Fixed Q8 base scale applied to active cursor and scroll deltas. 256 means
    // a one-to-one normalized-input to logical-output ratio. Active scroll may
    // optionally apply its separate speed curve to this base scale.
    int32_t cursor_base_scale_q8;
    int32_t scroll_base_scale_q8;
    ActiveScrollGainTuning active_scroll_gain;
    ScrollDirectionClassificationTuning scroll_direction_classification;
    ScrollMomentumTuning scroll_momentum;
    // Minimum absolute dx or dy in one Right report that qualifies
    // Left-assisted Drag. This threshold is per-report, not accumulated.
    int32_t left_assisted_drag_axis_threshold = 2;
    // Unclassified one-count cursor deltas accumulate for this long before
    // being discarded. Unit: microseconds.
    uint32_t subthreshold_cursor_expiry_us = 50000;
    // Per-axis accumulated magnitude required to emit unclassified cursor input.
    uint8_t subthreshold_cursor_threshold = 2;
    // Applies an adaptive temporal EMA only to cursor deltas after the existing base scale.
    bool cursor_temporal_filter_enabled = false;
    uint16_t cursor_filter_slow_speed_limit_counts_per_second = 500;
    uint16_t cursor_filter_fast_speed_limit_counts_per_second = 2000;
    uint8_t cursor_filter_slow_weight_percent = 20;
    uint8_t cursor_filter_normal_weight_percent = 10;
    uint8_t cursor_filter_fast_weight_percent = 0;
    // Sensor register timeouts, separate from gesture processing. Idle timeout
    // is seconds; LP1 timeout is stored in the sensor's 20-second units.
    uint8_t idle_timeout_before_lp1_seconds = 10;
    uint8_t lp1_timeout_before_lp2_20s_units = 1;
};

// Interface between normalized dual-pad input and logical actions.
// Keeps gesture policy independent of hardware and allows host-test substitutes.
class DualPadProcessor {
   public:
    virtual ~DualPadProcessor() = default;

    // Processes one coherent Left/Right snapshot and returns the logical cursor,
    // scroll, and button actions for that processing cycle.
    virtual LogicalActions process(const DualPadSnapshot& snapshot) = 0;
};

// Applies the dual-pad behavior contract and motion scaling to normalized input.
class DualTps43Fsm : public DualPadProcessor {
   public:
    // Creates an FSM with explicit behavior, fixed-scale, and momentum tuning.
    // Input: tuning values whose units and valid coefficient ranges are defined
    // by DualTps43Tuning and its nested tuning structures. Output: a reset FSM.
    explicit DualTps43Fsm(DualTps43Tuning tuning);

    // Processes one timestamped snapshot and returns hardware-independent
    // logical actions, including any active post-release scroll momentum.
    LogicalActions process(const DualPadSnapshot& snapshot) override;

    // Replaces the tuning and clears all gesture, motion, and momentum state.
    // The caller must provide a tuning value that has already passed validation.
    void set_tuning(DualTps43Tuning tuning);

    // Clears all interaction state while preserving the current tuning.
    void reset();

   private:
    enum class Mode {
        Idle,
        NeutralDualTouch,
        LeftScroll,
        LeftAssistedDrag,
        RightLatchedDrag,
    };

    struct SessionState {
        uint32_t id = 0;
        uint64_t started_us = 0;
        bool movement_seen = false;
        // True after one Right report qualifies the approved Left-assisted
        // Drag threshold. This is separate from movement_seen so other
        // gesture paths retain their existing movement semantics.
        bool drag_movement_qualified = false;
        // Identity of the opposite touch already active when this session began.
        uint32_t preceding_other_session_id = 0;
    };

    struct MotionScaleState {
        int64_t residual_x_q8 = 0;
        int64_t residual_y_q8 = 0;
    };

    enum class ScrollSource {
        None,
        Left,
        Right,
    };

#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    enum class ScrollDirectionMode {
        Unclassified,
        Vertical,
        Horizontal,
        Diagonal,
    };

    struct ScrollDirectionState {
        ScrollSource source = ScrollSource::None;
        ScrollDirectionMode mode = ScrollDirectionMode::Unclassified;
        int64_t pending_x = 0;
        int64_t pending_y = 0;
        uint32_t pending_interval_us = 0;
    };
#endif

    struct ScrollMotionState {
        MotionScaleState active_scale;
        MotionScaleState gain_scale;
        ScrollSource source = ScrollSource::None;
        int64_t filtered_velocity_x_q8_per_second = 0;
        int64_t filtered_velocity_y_q8_per_second = 0;
        bool momentum_active = false;
        int64_t momentum_velocity_x_q8_per_second = 0;
        int64_t momentum_velocity_y_q8_per_second = 0;
        int64_t momentum_residual_x_q8 = 0;
        int64_t momentum_residual_y_q8 = 0;
        uint64_t momentum_timestamp_us = 0;
    };

    struct ScaledDelta {
        int32_t x = 0;
        int32_t y = 0;
        int64_t x_q8 = 0;
        int64_t y_q8 = 0;
        uint32_t sample_interval_us = 0;
    };

    // Session and interaction-policy helpers.
    void begin_session_if_needed(const PadState& pad, SessionState& session, uint64_t now_us, uint32_t preceding_other_session_id);
    bool is_stationary(const PadState& pad, const SessionState& session, uint64_t now_us) const;
    bool is_eligible_tap(const PadState& pad, const SessionState& session, bool tap_event, bool consumed) const;
    bool reached_neutral_threshold(int64_t x, int64_t y) const;
    void consume_left_session(const PadState& left);
    void consume_right_session(const PadState& right);
    bool left_session_consumed(const PadState& left) const;
    bool right_session_consumed(const PadState& right) const;
    LogicalActions process_neutral(const DualPadSnapshot& snapshot);
    LogicalActions process_left_scroll(const DualPadSnapshot& snapshot);
    LogicalActions process_left_assisted_drag(const DualPadSnapshot& snapshot);
    LogicalActions process_right_latched_drag(const DualPadSnapshot& snapshot);
    LogicalActions process_idle(const DualPadSnapshot& snapshot, bool left_was_stationary, bool right_was_stationary,
        bool right_was_moving, bool right_drag_movement_qualified);

    // Raw normalized-motion collection helpers. Scaling occurs once after the
    // interaction policy has selected the cycle's logical actions.
    void add_right_cursor(const PadState& right, LogicalActions& actions);
    void add_left_scroll(const PadState& left, LogicalActions& actions);
    void add_right_scroll(const PadState& right, LogicalActions& actions);
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    bool classify_scroll_delta(
        ScrollSource source, const PadState& pad, uint32_t sample_interval_us, int32_t& x, int32_t& y);
    void flush_pending_scroll_direction(const DualPadSnapshot& snapshot, LogicalActions& actions);
    void reset_scroll_direction(ScrollSource source);
#endif
    void clear_pending_left_scroll();
    void reset_cursor_temporal_filter();

    // Active-motion scaling and scroll-momentum helpers.
    void apply_motion(const DualPadSnapshot& snapshot, LogicalActions& actions);
    uint16_t active_scroll_gain_percent(int32_t x, int32_t y, uint64_t sample_interval_us) const;
    ScaledDelta scale_active_delta(
        int32_t x,
        int32_t y,
        uint64_t acquisition_interval_us,
        int32_t base_scale_q8,
        MotionScaleState& state,
        uint16_t gain_percent) const;
    void stop_cursor_motion();
    void update_scroll_release_velocity(const ScaledDelta& delta);
    void start_scroll_momentum(uint64_t now_us);
    void apply_scroll_momentum(uint64_t now_us, LogicalActions& actions);
    void stop_scroll_momentum();
    bool scroll_source_active(const DualPadSnapshot& snapshot) const;
    static uint32_t sample_interval_us(uint64_t now_us, uint64_t& last_timestamp_us, uint32_t fallback_us);
    static int64_t filter_signed(int64_t previous, int64_t current, uint16_t weight_q8);
    static uint64_t absolute_int64(int64_t value);
    static int32_t q8_axis(int64_t delta_q8, int64_t& residual_q8);
    static int64_t multiply_divide_saturated(int64_t value, uint32_t multiplier, uint32_t divisor);
    static int32_t saturate_int32(int64_t value);

    DualTps43Tuning tuning_;
    Mode mode_ = Mode::Idle;
    SessionState left_session_;
    SessionState right_session_;
    bool previous_left_active_ = false;
    bool previous_right_active_ = false;
    uint32_t consumed_left_session_id_ = 0;
    uint32_t consumed_right_session_id_ = 0;
    int64_t neutral_left_x_ = 0;
    int64_t neutral_left_y_ = 0;
    int64_t neutral_right_x_ = 0;
    int64_t neutral_right_y_ = 0;
    bool right_latched_saw_inactive_ = false;
    uint32_t right_latched_drop_session_id_ = 0;
    MotionScaleState cursor_motion_;
    int64_t pending_cursor_x_ = 0;
    int64_t pending_cursor_y_ = 0;
    uint64_t pending_cursor_since_us_ = 0;
    int64_t filtered_cursor_x_q8_ = 0;
    int64_t filtered_cursor_y_q8_ = 0;
    // Preserve pre-activation Left deltas until movement confirms scroll intent.
    int32_t pending_left_scroll_x_ = 0;
    int32_t pending_left_scroll_y_ = 0;
    uint32_t pending_left_scroll_interval_us_ = 0;
    uint32_t scroll_sample_interval_override_us_ = 0;
    ScrollMotionState scroll_motion_;
    ScrollSource scroll_source_this_cycle_ = ScrollSource::None;
#if defined(TPS43_TEST_SCROLL_DIRECTION_CLASSIFICATION)
    ScrollDirectionState scroll_direction_;
#endif
};

#endif
