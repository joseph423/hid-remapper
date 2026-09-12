#ifndef TPS43_TIMING_METRICS_H_
#define TPS43_TIMING_METRICS_H_

#include <cstdint>

struct Tps43RuntimeCounters {
    uint64_t usb_host_service_calls = 0;
    uint64_t usb_host_service_total_us = 0;
    uint32_t usb_host_service_max_us = 0;
    uint64_t usb_device_service_calls = 0;
    uint64_t usb_device_service_total_us = 0;
    uint32_t usb_device_service_max_us = 0;
    uint64_t cursor_service_calls = 0;
    uint64_t cursor_nonzero_actions = 0;
};

// Records one USB host service invocation for the physical timing handoff.
void tps43_note_usb_host_service(uint32_t duration_us);

// Records one USB device service invocation for the physical timing handoff.
void tps43_note_usb_device_service(uint32_t duration_us);

// Records one logical cursor-service invocation and whether it carried motion.
void tps43_note_cursor_service(bool nonzero_motion);

// Returns cumulative runtime counters without resetting them.
const Tps43RuntimeCounters& tps43_runtime_counters();

// Resets the counters at the start of one timing interval.
void tps43_reset_runtime_counters();

#endif
