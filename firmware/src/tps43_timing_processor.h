#ifndef TPS43_TIMING_PROCESSOR_H_
#define TPS43_TIMING_PROCESSOR_H_

#include "dual_tps43_fsm.h"

// Temporary one-pad processor for timing bring-up while physical behavior
// tuning remains unresolved. It does not replace the production FSM policy.
class Tps43OnePadBringupProcessor final : public DualPadProcessor {
   public:
    // Converts fresh Right one-finger movement into raw logical cursor motion.
    LogicalActions process(const DualPadSnapshot& snapshot) override;
};

#endif
