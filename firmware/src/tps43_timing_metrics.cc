#include "tps43_timing_metrics.h"

#include <algorithm>

namespace {

Tps43RuntimeCounters counters;

}  // namespace

void tps43_note_usb_host_service(uint32_t duration_us) {
    counters.usb_host_service_calls++;
    counters.usb_host_service_total_us += duration_us;
    counters.usb_host_service_max_us = std::max(counters.usb_host_service_max_us, duration_us);
}

void tps43_note_usb_device_service(uint32_t duration_us) {
    counters.usb_device_service_calls++;
    counters.usb_device_service_total_us += duration_us;
    counters.usb_device_service_max_us = std::max(counters.usb_device_service_max_us, duration_us);
}

void tps43_note_cursor_service(bool nonzero_motion) {
    counters.cursor_service_calls++;
    if (nonzero_motion) {
        counters.cursor_nonzero_actions++;
    }
}

const Tps43RuntimeCounters& tps43_runtime_counters() {
    return counters;
}

void tps43_reset_runtime_counters() {
    counters = {};
}
