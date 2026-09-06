#include "dual_tps43_coordinator.h"

DualTps43Coordinator::DualTps43Coordinator(Tps43Driver& left_driver, Tps43Driver& right_driver, DualPadProcessor& processor, Tps43ActionSink& action_sink)
    : left_driver_(left_driver), right_driver_(right_driver), processor_(processor), action_sink_(action_sink) {
}

void DualTps43Coordinator::service(uint64_t now_us) {
    const bool left_fresh = left_driver_.service(now_us);
    const bool right_fresh = right_driver_.service(now_us);

    DualPadSnapshot snapshot;
    snapshot.left = left_tracker_.update(left_driver_.sample(), left_fresh);
    snapshot.right = right_tracker_.update(right_driver_.sample(), right_fresh);
    snapshot.cycle_timestamp_us = now_us;

    action_sink_.apply(processor_.process(snapshot));
}
