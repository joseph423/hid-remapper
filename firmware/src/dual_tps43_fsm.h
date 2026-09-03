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

struct DualTps43Tuning {
    uint64_t tap_max_duration_us;
    uint64_t stationary_intent_threshold_us;
    // Does not apply to Right-latched Drag entry, which uses the sensor's first
    // reported three-finger movement without an extra distance threshold.
    int32_t neutral_activation_threshold;
    int32_t cursor_scale;
    int32_t scroll_scale;
};

// Interface between normalized dual-pad input and logical actions.
// Keeps gesture policy independent of hardware and allows host-test substitutes.
class DualPadProcessor {
   public:
    virtual ~DualPadProcessor() = default;
    virtual LogicalActions process(const DualPadSnapshot& snapshot) = 0;
};

class DualTps43Fsm : public DualPadProcessor {
   public:
    explicit DualTps43Fsm(DualTps43Tuning tuning);
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
    void add_right_cursor(const PadState& right, LogicalActions& actions) const;
    void add_left_scroll(const PadState& left, LogicalActions& actions) const;
    void add_right_scroll(const PadState& right, LogicalActions& actions) const;

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
};

#endif
