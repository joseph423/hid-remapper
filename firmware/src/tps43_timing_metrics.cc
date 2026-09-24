#include "tps43_timing_metrics.h"

#include <algorithm>
#include <cstdio>

#ifndef TPS43_NORMAL_CAPTURE_TRACE_CAPACITY
#define TPS43_NORMAL_CAPTURE_TRACE_CAPACITY 1
#endif

namespace {

Tps43RuntimeCounters counters;
uint64_t last_usb_host_service_us = 0;
uint64_t last_usb_device_service_us = 0;
uint64_t last_scroll_report_us = 0;
bool runtime_metrics_enabled = false;

void note_service(uint64_t finished_us, uint32_t duration_us, uint64_t& previous_us, uint64_t& calls, uint64_t& total_us, uint32_t& maximum_us, uint32_t& maximum_gap_us) {
    ++calls;
    total_us += duration_us;
    maximum_us = std::max(maximum_us, duration_us);
    if (previous_us != 0 && finished_us >= previous_us) {
        const uint64_t gap_us = finished_us - previous_us;
        maximum_gap_us = std::max(maximum_gap_us, static_cast<uint32_t>(std::min<uint64_t>(gap_us, UINT32_MAX)));
    }
    previous_us = finished_us;
}

}  // namespace

void tps43_note_usb_host_service(uint64_t finished_us, uint32_t duration_us) {
    note_service(finished_us, duration_us, last_usb_host_service_us, counters.usb_host_service_calls,
        counters.usb_host_service_total_us, counters.usb_host_service_max_us, counters.usb_host_service_max_gap_us);
}

void tps43_note_usb_device_service(uint64_t finished_us, uint32_t duration_us) {
    note_service(finished_us, duration_us, last_usb_device_service_us, counters.usb_device_service_calls,
        counters.usb_device_service_total_us, counters.usb_device_service_max_us, counters.usb_device_service_max_gap_us);
}

void tps43_set_runtime_metrics_enabled(bool enabled) {
    runtime_metrics_enabled = enabled;
}

bool tps43_runtime_metrics_enabled() {
    return runtime_metrics_enabled;
}

void tps43_note_pointer_service(bool nonzero_cursor_motion, bool nonzero_scroll_motion, uint64_t timestamp_us) {
    if (!runtime_metrics_enabled) {
        return;
    }
    counters.cursor_service_calls++;
    if (nonzero_cursor_motion) {
        counters.cursor_nonzero_actions++;
    }
    if (nonzero_scroll_motion) {
        ++counters.scroll_report_calls;
        if (last_scroll_report_us != 0 && timestamp_us >= last_scroll_report_us) {
            counters.scroll_report_max_gap_us = std::max(
                counters.scroll_report_max_gap_us,
                static_cast<uint32_t>(std::min<uint64_t>(timestamp_us - last_scroll_report_us, UINT32_MAX)));
        }
        last_scroll_report_us = timestamp_us;
    }
}

const Tps43RuntimeCounters& tps43_runtime_counters() {
    return counters;
}

void tps43_reset_runtime_counters() {
    counters = {};
    last_usb_host_service_us = 0;
    last_usb_device_service_us = 0;
    last_scroll_report_us = 0;
}

namespace {

constexpr uint64_t kNormalDurationUs = 15000000;
// Keep normal firmware within the USB-host RAM budget. A dedicated timing
// build may override this at compile time when full per-event traces are needed.
constexpr size_t kTraceCapacity = TPS43_NORMAL_CAPTURE_TRACE_CAPACITY;

struct IntervalStats {
    uint64_t previous = 0;
    uint64_t total = 0;
    uint32_t count = 0;
    uint32_t maximum = 0;
    // <=1, <=2, <=4, <=8, <=16, <=32, >32 ms. Idle gaps are included.
    uint32_t bins[7] = {};
};

struct TraceRow {
    uint32_t elapsed_us;
    int32_t dx, dy;
    uint8_t count, flags;
};

struct Trace {
    TraceRow rows[kTraceCapacity] = {};
    uint32_t count = 0;
};

struct NormalCapture {
    enum class Phase { Idle,
        Recording,
        Dumping } phase = Phase::Idle;
    uint64_t start = 0;
    uint64_t next_print = 0;
    IntervalStats samples, movement, usb, scroll_usb;
    Trace sample_trace, usb_trace;
    uint32_t counts[7] = {};      // 0..5 fingers, invalid >5.
    uint32_t delta_bins[6] = {};  // max(abs(dx),abs(dy)): 0,1,2..3,4..7,8..15,16+.
    uint32_t failed_usb = 0;
    // A 15-second full-speed interrupt endpoint can complete at most 15,000 reports.
    uint16_t digitizer_transfer_complete = 0;
    uint16_t digitizer_transfer_failed = 0;
    uint16_t digitizer_payload_bad_length = 0;
    uint16_t digitizer_contact_counts[4] = {};  // 0, 1, 2, invalid.
    uint8_t digitizer_contact1_flags_or = 0;
    uint8_t digitizer_contact1_flags_and = 0;
    uint8_t digitizer_contact2_flags_or = 0;
    uint8_t digitizer_contact2_flags_and = 0;
    uint32_t raw_scroll_samples = 0;
    int64_t raw_scroll_x = 0, raw_scroll_y = 0;
    uint32_t scroll_action_samples = 0;
    int64_t scroll_action_x_q8 = 0, scroll_action_y_q8 = 0;
    uint32_t scroll_usb_attempts = 0, scroll_usb_successes = 0;
    int64_t scroll_usb_wheel = 0, scroll_usb_pan = 0;
    uint32_t start_failures = 0, start_timeouts = 0;
    uint32_t failures = 0, timeouts = 0, max_poll = 0, max_acquisition = 0;
    uint32_t dump_row = 0;
};

NormalCapture normal;
uint32_t latest_failures = 0, latest_timeouts = 0;

void add_interval(IntervalStats& stats, uint64_t now) {
    if (stats.previous != 0 && now >= stats.previous) {
        const uint32_t gap = static_cast<uint32_t>(std::min<uint64_t>(now - stats.previous, UINT32_MAX));
        stats.total += gap;
        ++stats.count;
        stats.maximum = std::max(stats.maximum, gap);
        unsigned bin = 0;
        while (bin < 6 && gap > (1000u << bin)) {
            ++bin;
        }
        ++stats.bins[bin];
    }
    stats.previous = now;
}

void add_trace(Trace& trace, TraceRow row) {
    trace.rows[trace.count % kTraceCapacity] = row;
    ++trace.count;
}

bool recording_at(uint64_t now) {
    return normal.phase == NormalCapture::Phase::Recording &&
           now >= normal.start && now - normal.start < kNormalDurationUs;
}

void print_intervals(const char* name, const IntervalStats& stats) {
    printf("normal_%s intervals=%lu mean_us=%lu max_us=%lu bins_le_1_2_4_8_16_32_gt32ms=%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        name, static_cast<unsigned long>(stats.count),
        static_cast<unsigned long>(stats.count ? stats.total / stats.count : 0),
        static_cast<unsigned long>(stats.maximum),
        static_cast<unsigned long>(stats.bins[0]), static_cast<unsigned long>(stats.bins[1]),
        static_cast<unsigned long>(stats.bins[2]), static_cast<unsigned long>(stats.bins[3]),
        static_cast<unsigned long>(stats.bins[4]), static_cast<unsigned long>(stats.bins[5]),
        static_cast<unsigned long>(stats.bins[6]));
}

void print_trace_row(const char* kind, const Trace& trace, uint32_t index) {
    const uint32_t retained = std::min<uint32_t>(trace.count, kTraceCapacity);
    const TraceRow& row = trace.rows[(trace.count - retained + index) % kTraceCapacity];
    printf("normal_%s t_us=%lu dx=%ld dy=%ld count=%u flags=%u\n", kind,
        static_cast<unsigned long>(row.elapsed_us), static_cast<long>(row.dx),
        static_cast<long>(row.dy), row.count, row.flags);
}

}  // namespace

void tps43_normal_capture_start(uint64_t now_us) {
    if (tps43_normal_capture_busy()) {
        return;
    }
    normal = {};
    normal.phase = NormalCapture::Phase::Recording;
    normal.start = now_us + 1000000;
    normal.start_failures = normal.failures = latest_failures;
    normal.start_timeouts = normal.timeouts = latest_timeouts;
    printf("NORMAL: starts in 1 second; move continuously for 15 seconds; no forced reads; output follows afterward\n");
}

bool tps43_normal_capture_busy() {
    return normal.phase != NormalCapture::Phase::Idle;
}

void tps43_normal_capture_note_sample(uint64_t now_us, const Tps43Sample& sample, bool fresh, uint32_t failures, uint32_t timeouts, uint32_t lifetime_max_poll_us, uint32_t acquisition_us) {
    latest_failures = failures;
    latest_timeouts = timeouts;
    // Fault totals are sampled every tick, including ticks with no acquisition.
    if (normal.phase == NormalCapture::Phase::Recording && now_us < normal.start) {
        normal.start_failures = normal.failures = failures;
        normal.start_timeouts = normal.timeouts = timeouts;
    }
    if (recording_at(now_us)) {
        normal.failures = failures;
        normal.timeouts = timeouts;
        normal.max_poll = lifetime_max_poll_us;
    }
    if (!fresh || !recording_at(sample.timestamp_us)) {
        return;
    }
    normal.max_acquisition = std::max(normal.max_acquisition, acquisition_us);
    add_interval(normal.samples, sample.timestamp_us);
    ++normal.counts[std::min<unsigned>(sample.finger_count, 6)];
    if (sample.finger_count == 1) {
        const auto magnitude = [](int32_t value) {
            return static_cast<uint64_t>(value < 0 ? -static_cast<int64_t>(value) : value);
        };
        const uint64_t delta = std::max(magnitude(sample.relative_x), magnitude(sample.relative_y));
        const unsigned bin = delta == 0 ? 0 : delta == 1 ? 1
                                          : delta < 4    ? 2
                                          : delta < 8    ? 3
                                          : delta < 16   ? 4
                                                         : 5;
        ++normal.delta_bins[bin];
        if (sample.movement_reported && delta != 0) {
            add_interval(normal.movement, sample.timestamp_us);
        }
    }
    if (sample.finger_count == 2 && sample.scroll_gesture && sample.movement_reported) {
        ++normal.raw_scroll_samples;
        normal.raw_scroll_x += sample.relative_x;
        normal.raw_scroll_y += sample.relative_y;
    }
    // Sample flags: bit0 movement, bit1 single tap, bit2 two-finger tap, bit3 scroll.
    const uint8_t flags = sample.movement_reported | (sample.single_tap << 1) |
                          (sample.two_finger_tap << 2) | (sample.scroll_gesture << 3);
    add_trace(normal.sample_trace, { static_cast<uint32_t>(sample.timestamp_us - normal.start),
                                       sample.relative_x, sample.relative_y, sample.finger_count, flags });
}

void tps43_normal_capture_note_usb(uint64_t now_us, bool success, int32_t dx, int32_t dy) {
    if (!recording_at(now_us)) {
        return;
    }
    if (success) {
        add_interval(normal.usb, now_us);
    } else {
        ++normal.failed_usb;
    }
    // USB flags: bit0 successful submission. count is unused for USB rows.
    add_trace(normal.usb_trace, { static_cast<uint32_t>(now_us - normal.start), dx, dy, 0,
                                    static_cast<uint8_t>(success) });
}

void tps43_normal_capture_note_digitizer_transfer(
    uint64_t now_us, bool success, const uint8_t* report, uint16_t len) {
    if (!recording_at(now_us)) {
        return;
    }
    if (!success) {
        ++normal.digitizer_transfer_failed;
        return;
    }

    ++normal.digitizer_transfer_complete;
    if (report == nullptr || len != 16) {
        ++normal.digitizer_payload_bad_length;
        return;
    }

    // Report includes the ID byte: contact flags are bytes 1 and 7, and the
    // contact count is byte 13. The three defined contact flags occupy bits 0-2.
    const uint8_t contact1_flags = report[1] & 0x07;
    const uint8_t contact2_flags = report[7] & 0x07;
    const uint8_t contact_count = report[13];
    ++normal.digitizer_contact_counts[contact_count <= 2 ? contact_count : 3];

    if (contact_count >= 1 && contact_count <= 2) {
        if (normal.digitizer_contact_counts[1] + normal.digitizer_contact_counts[2] == 1) {
            normal.digitizer_contact1_flags_and = contact1_flags;
        } else {
            normal.digitizer_contact1_flags_and &= contact1_flags;
        }
        normal.digitizer_contact1_flags_or |= contact1_flags;
    }
    if (contact_count == 2) {
        if (normal.digitizer_contact_counts[2] == 1) {
            normal.digitizer_contact2_flags_and = contact2_flags;
        } else {
            normal.digitizer_contact2_flags_and &= contact2_flags;
        }
        normal.digitizer_contact2_flags_or |= contact2_flags;
    }
}

void tps43_normal_capture_note_scroll_action(uint64_t now_us, int64_t scroll_x_q8, int64_t scroll_y_q8) {
    if (!recording_at(now_us) || (scroll_x_q8 == 0 && scroll_y_q8 == 0)) {
        return;
    }
    ++normal.scroll_action_samples;
    normal.scroll_action_x_q8 += scroll_x_q8;
    normal.scroll_action_y_q8 += scroll_y_q8;
}

void tps43_normal_capture_note_scroll_usb(uint64_t now_us, bool success, int32_t wheel, int32_t pan) {
    if (!recording_at(now_us) || (wheel == 0 && pan == 0)) {
        return;
    }
    ++normal.scroll_usb_attempts;
    if (!success) {
        return;
    }
    ++normal.scroll_usb_successes;
    normal.scroll_usb_wheel += wheel;
    normal.scroll_usb_pan += pan;
    add_interval(normal.scroll_usb, now_us);
}

void tps43_normal_capture_poll(uint64_t now_us) {
    if (normal.phase == NormalCapture::Phase::Recording && now_us >= normal.start + kNormalDurationUs) {
        normal.phase = NormalCapture::Phase::Dumping;
        normal.next_print = now_us;
    }
    if (normal.phase != NormalCapture::Phase::Dumping || now_us < normal.next_print) {
        return;
    }
    // Throttle the post-capture dump to avoid overflowing the CDC ring buffer.
    normal.next_print = now_us + 10000;
    const uint32_t row = normal.dump_row++;
    if (row == 0) {
        printf("normal_summary duration_us=15000000 samples=%lu usb_attempts=%lu usb_failed=%lu digitizer_transfer_complete=%lu digitizer_transfer_failed=%lu failures=%lu timeouts=%lu acquisition_max_us=%lu driver_lifetime_max_poll_us=%lu\n",
            static_cast<unsigned long>(normal.sample_trace.count), static_cast<unsigned long>(normal.usb_trace.count),
            static_cast<unsigned long>(normal.failed_usb), static_cast<unsigned long>(normal.digitizer_transfer_complete),
            static_cast<unsigned long>(normal.digitizer_transfer_failed),
            static_cast<unsigned long>(normal.failures - normal.start_failures),
            static_cast<unsigned long>(normal.timeouts - normal.start_timeouts), static_cast<unsigned long>(normal.max_acquisition),
            static_cast<unsigned long>(normal.max_poll));
    } else if (row <= 3) {
        print_intervals(row == 1 ? "samples" : row == 2 ? "movement"
                                                        : "usb_submitted",
            row == 1 ? normal.samples : row == 2 ? normal.movement
                                                 : normal.usb);
    } else if (row == 4) {
        printf("normal_counts fingers_0_1_2_3_4_5_invalid=%lu,%lu,%lu,%lu,%lu,%lu,%lu delta_0_1_2to3_4to7_8to15_16plus=%lu,%lu,%lu,%lu,%lu,%lu\n",
            static_cast<unsigned long>(normal.counts[0]), static_cast<unsigned long>(normal.counts[1]),
            static_cast<unsigned long>(normal.counts[2]), static_cast<unsigned long>(normal.counts[3]),
            static_cast<unsigned long>(normal.counts[4]), static_cast<unsigned long>(normal.counts[5]),
            static_cast<unsigned long>(normal.counts[6]), static_cast<unsigned long>(normal.delta_bins[0]),
            static_cast<unsigned long>(normal.delta_bins[1]), static_cast<unsigned long>(normal.delta_bins[2]),
            static_cast<unsigned long>(normal.delta_bins[3]), static_cast<unsigned long>(normal.delta_bins[4]),
            static_cast<unsigned long>(normal.delta_bins[5]));
    } else if (row == 5) {
        printf("normal_trace retained_last_per_source=%lu sample_overwritten=%lu usb_overwritten=%lu; sample_flags=movement:1,tap:2,two_tap:4,scroll:8 usb_flags=submitted:1\n",
            static_cast<unsigned long>(kTraceCapacity),
            static_cast<unsigned long>(normal.sample_trace.count > kTraceCapacity ? normal.sample_trace.count - kTraceCapacity : 0),
            static_cast<unsigned long>(normal.usb_trace.count > kTraceCapacity ? normal.usb_trace.count - kTraceCapacity : 0));
    } else if (row == 6) {
        printf("normal_scroll raw_samples=%lu raw_dx=%lld raw_dy=%lld action_samples=%lu action_q8_dx=%lld action_q8_dy=%lld usb_attempts=%lu usb_successes=%lu usb_wheel=%lld usb_pan=%lld usb_mean_us=%lu usb_max_us=%lu\n",
            static_cast<unsigned long>(normal.raw_scroll_samples), static_cast<long long>(normal.raw_scroll_x),
            static_cast<long long>(normal.raw_scroll_y), static_cast<unsigned long>(normal.scroll_action_samples),
            static_cast<long long>(normal.scroll_action_x_q8), static_cast<long long>(normal.scroll_action_y_q8),
            static_cast<unsigned long>(normal.scroll_usb_attempts), static_cast<unsigned long>(normal.scroll_usb_successes),
            static_cast<long long>(normal.scroll_usb_wheel), static_cast<long long>(normal.scroll_usb_pan),
            static_cast<unsigned long>(normal.scroll_usb.count ? normal.scroll_usb.total / normal.scroll_usb.count : 0),
            static_cast<unsigned long>(normal.scroll_usb.maximum));
    } else if (row == 7) {
        printf("normal_digitizer payload_reports=%u bad_length=%u contact_count_0=%u contact_count_1=%u contact_count_2=%u contact_count_invalid=%u contact1_active=%u contact1_flags_or=0x%02x contact1_flags_and=0x%02x contact2_active=%u contact2_flags_or=0x%02x contact2_flags_and=0x%02x flags=tip:bit0,in_range:bit1,touch_valid:bit2\n",
            static_cast<unsigned>(normal.digitizer_contact_counts[0] + normal.digitizer_contact_counts[1] +
                                 normal.digitizer_contact_counts[2] + normal.digitizer_contact_counts[3]),
            static_cast<unsigned>(normal.digitizer_payload_bad_length),
            static_cast<unsigned>(normal.digitizer_contact_counts[0]),
            static_cast<unsigned>(normal.digitizer_contact_counts[1]),
            static_cast<unsigned>(normal.digitizer_contact_counts[2]),
            static_cast<unsigned>(normal.digitizer_contact_counts[3]),
            static_cast<unsigned>(normal.digitizer_contact_counts[1] + normal.digitizer_contact_counts[2]),
            static_cast<unsigned>(normal.digitizer_contact1_flags_or),
            static_cast<unsigned>(normal.digitizer_contact1_flags_and),
            static_cast<unsigned>(normal.digitizer_contact_counts[2]),
            static_cast<unsigned>(normal.digitizer_contact2_flags_or),
            static_cast<unsigned>(normal.digitizer_contact2_flags_and));
    } else {
        const uint32_t sample_rows = std::min<uint32_t>(normal.sample_trace.count, kTraceCapacity);
        const uint32_t usb_rows = std::min<uint32_t>(normal.usb_trace.count, kTraceCapacity);
        if (row - 8 < sample_rows) {
            print_trace_row("sample", normal.sample_trace, row - 8);
        } else if (row - 8 - sample_rows < usb_rows) {
            print_trace_row("usb", normal.usb_trace, row - 8 - sample_rows);
        } else {
            printf("NORMAL DONE: summaries cover all 15 seconds; traces retain last %lu events per source; M repeats\n",
                static_cast<unsigned long>(kTraceCapacity));
            normal.phase = NormalCapture::Phase::Idle;
        }
    }
}
