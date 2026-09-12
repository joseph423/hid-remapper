#ifndef TPS43_TIMING_PROCESSOR_H_
#define TPS43_TIMING_PROCESSOR_H_

#include "dual_tps43_fsm.h"

// Temporary conventional one-pad processor for the Phase 7 HID path while
// physical behavior tuning and dual-pad integration remain unresolved. It
// does not replace the production FSM policy.
class Tps43OnePadBringupProcessor final : public DualPadProcessor {
   public:
    // Converts fresh Right-pad conventional gestures into logical actions:
    // one-finger cursor/tap and two-finger scroll/tap.
    LogicalActions process(const DualPadSnapshot& snapshot) override;
};

#endif
