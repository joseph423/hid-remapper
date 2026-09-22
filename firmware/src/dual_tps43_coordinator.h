#ifndef DUAL_TPS43_COORDINATOR_H_
#define DUAL_TPS43_COORDINATOR_H_

#include <cstdint>

#include "dual_tps43_fsm.h"
#include "pad_state.h"
#include "tps43_driver.h"
#include "tps43_hid_adapter.h"

// Owns processing order: service both pads, form one coherent logical-cycle
// snapshot, run the policy once, then forward its logical actions.
class DualTps43Coordinator {
   public:
    DualTps43Coordinator(Tps43Driver& left_driver, Tps43Driver& right_driver, DualPadProcessor& processor, Tps43ActionSink& action_sink,
        bool service_right_first = false);

    void service(uint64_t now_us);

    // Discards both normalized touch histories before a new FSM profile starts.
    void reset();

   private:
    Tps43Driver& left_driver_;
    Tps43Driver& right_driver_;
    DualPadProcessor& processor_;
    Tps43ActionSink& action_sink_;
    bool service_right_first_;
    PadStateTracker left_tracker_;
    PadStateTracker right_tracker_;
};

#endif
