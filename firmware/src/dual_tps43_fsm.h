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
    int32_t movement_threshold;
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
    DualTps43Tuning tuning_;
};

#endif
