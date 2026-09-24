#ifndef TPS43_TIMING_METRICS_H_
#define TPS43_TIMING_METRICS_H_

#include <cstdint>

#include "tps43_driver.h"

struct Tps43RuntimeCounters {
    uint64_t usb_host_service_calls = 0;
    uint64_t usb_host_service_total_us = 0;
    uint32_t usb_host_service_max_us = 0;
    uint32_t usb_host_service_max_gap_us = 0;
    uint64_t usb_device_service_calls = 0;
    uint64_t usb_device_service_total_us = 0;
    uint32_t usb_device_service_max_us = 0;
    uint32_t usb_device_service_max_gap_us = 0;
    uint64_t cursor_service_calls = 0;
    uint64_t cursor_nonzero_actions = 0;
    uint64_t scroll_report_calls = 0;
    uint32_t scroll_report_max_gap_us = 0;
};

// Records one USB host service invocation for the physical timing handoff.
void tps43_note_usb_host_service(uint64_t finished_us, uint32_t duration_us);

// Records one USB device service invocation for the physical timing handoff.
void tps43_note_usb_device_service(uint64_t finished_us, uint32_t duration_us);

// Enables the opt-in Phase 11 USB timing instrumentation.
void tps43_set_runtime_metrics_enabled(bool enabled);

// Returns whether the current service loop should record Phase 11 timing metrics.
bool tps43_runtime_metrics_enabled();

// Records one logical pointer-service invocation and any cursor or scroll action.
void tps43_note_pointer_service(bool nonzero_cursor_motion, bool nonzero_scroll_motion, uint64_t timestamp_us);

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

// Records descriptor-6 report-3 transfer completion and summarizes its payload.
void tps43_normal_capture_note_digitizer_transfer(
    uint64_t now_us, bool success, const uint8_t* report, uint16_t len);

// Records nonzero active-scroll Q8 output without printing or performing I/O.
void tps43_normal_capture_note_scroll_action(uint64_t now_us, int64_t scroll_x_q8, int64_t scroll_y_q8);

// Records wheel/pan fields from one submitted generic mouse report.
void tps43_normal_capture_note_scroll_usb(uint64_t now_us, bool success, int32_t wheel, int32_t pan);

#endif
