#ifndef TPS43_TIMING_METRICS_H_
#define TPS43_TIMING_METRICS_H_

#include <cstdint>

#include "tps43_driver.h"

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

// Starts a repeatable 15-second RAM capture after a 1-second settling delay.
// Does nothing while recording or dumping; never requests sensor communication.
void tps43_normal_capture_start(uint64_t now_us);

// Reports whether capture/settling/output is active, to suppress other logging.
bool tps43_normal_capture_busy();

// Advances capture timing and prints at most one buffered row per 10 ms after
// recording ends. now_us uses the same monotonic clock as the driver.
void tps43_normal_capture_poll(uint64_t now_us);

// Records a fresh sensor sample plus cumulative driver fault counters. No output.
void tps43_normal_capture_note_sample(uint64_t now_us, const Tps43Sample& sample, bool fresh, uint32_t failures, uint32_t timeouts, uint32_t lifetime_max_poll_us, uint32_t acquisition_us);

// Records an attempted mouse-report submission and its actual TinyUSB result.
// dx/dy are signed report fields; success means queued by USB, not displayed by OS.
void tps43_normal_capture_note_usb(uint64_t now_us, bool success, int32_t dx, int32_t dy);

#endif
