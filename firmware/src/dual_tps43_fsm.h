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
    ButtonAction left_button = ButtonAction::None;
    ButtonAction right_button = ButtonAction::None;
};

// Configures velocity-dependent gain for one motion channel. Input velocity is
// measured in normalized pad units per second. Gains use Q8 fixed point, where
// 256 means one logical output unit per normalized pad unit. Filter weight is
// in [0, 256], where 256 uses only the current velocity sample.
struct VelocityGainTuning {
    // Logical output units per normalized pad unit, in Q8.
    int32_t minimum_gain_q8;
    int32_t maximum_gain_q8;
    // Normalized pad units per second that select maximum gain.
    uint32_t full_gain_velocity_pad_units_per_second;
    // Current velocity sample weight in Q8.
    uint16_t velocity_filter_weight_q8;
    // Microseconds used only when consecutive timestamps do not advance.
    uint32_t fallback_sample_interval_us;
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
    VelocityGainTuning cursor_gain;
    VelocityGainTuning scroll_gain;
    ScrollMomentumTuning scroll_momentum;
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
    // Creates an FSM with explicit behavior, velocity-gain, and momentum tuning.
    // Input: tuning values whose units and valid coefficient ranges are defined
    // by DualTps43Tuning and its nested tuning structures. Output: a reset FSM.
    explicit DualTps43Fsm(DualTps43Tuning tuning);

    // Processes one timestamped snapshot and returns hardware-independent
    // logical actions, including any active post-release scroll momentum.
    LogicalActions process(const DualPadSnapshot& snapshot) override;

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
    };

    struct VelocityGainState {
        uint64_t filtered_speed_pad_units_per_second = 0;
        int64_t residual_x_q8 = 0;
        int64_t residual_y_q8 = 0;
    };

    enum class ScrollSource {
        None,
        Left,
        Right,
    };

    struct ScrollMotionState {
        VelocityGainState active_gain;
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
    void begin_session_if_needed(const PadState& pad, SessionState& session, uint64_t now_us);
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
    LogicalActions process_idle(const DualPadSnapshot& snapshot, bool left_was_stationary, bool right_was_stationary, bool right_was_moving);

    // Raw normalized-motion collection helpers. Scaling occurs once after the
    // interaction policy has selected the cycle's logical actions.
    void add_right_cursor(const PadState& right, LogicalActions& actions);
    void add_left_scroll(const PadState& left, LogicalActions& actions);
    void add_right_scroll(const PadState& right, LogicalActions& actions);

    // Velocity-gain and scroll-momentum helpers.
    void apply_motion(const DualPadSnapshot& snapshot, LogicalActions& actions);
    ScaledDelta scale_active_delta(int32_t x, int32_t y, uint64_t acquisition_interval_us, const VelocityGainTuning& tuning, VelocityGainState& state) const;
    void stop_cursor_motion();
    void update_scroll_release_velocity(const ScaledDelta& delta);
    void start_scroll_momentum(uint64_t now_us);
    void apply_scroll_momentum(uint64_t now_us, LogicalActions& actions);
    void stop_scroll_momentum();
    bool scroll_source_active(const DualPadSnapshot& snapshot) const;
    static uint32_t sample_interval_us(uint64_t now_us, uint64_t& last_timestamp_us, uint32_t fallback_us);
    static uint64_t vector_magnitude(int32_t x, int32_t y);
    static uint64_t integer_square_root(uint64_t value);
    static uint64_t filter_unsigned(uint64_t previous, uint64_t current, uint16_t weight_q8);
    static int64_t filter_signed(int64_t previous, int64_t current, uint16_t weight_q8);
    static uint64_t absolute_int64(int64_t value);
    static int32_t velocity_gain_q8(uint64_t speed_pad_units_per_second, const VelocityGainTuning& tuning);
    static int32_t scaled_axis(int32_t input, int32_t gain_q8, int64_t& residual_q8);
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
    VelocityGainState cursor_motion_;
    ScrollMotionState scroll_motion_;
    ScrollSource scroll_source_this_cycle_ = ScrollSource::None;
};

#endif
