#include "tps43_timing_metrics.h"

#include <algorithm>
#include <cstdio>

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

namespace {

constexpr uint64_t kNormalDurationUs = 15000000;
constexpr size_t kTraceCapacity = 512;

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
    IntervalStats samples, movement, usb;
    Trace sample_trace, usb_trace;
    uint32_t counts[7] = {};      // 0..5 fingers, invalid >5.
    uint32_t delta_bins[6] = {};  // max(abs(dx),abs(dy)): 0,1,2..3,4..7,8..15,16+.
    uint32_t failed_usb = 0;
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
        printf("normal_summary duration_us=15000000 samples=%lu usb_attempts=%lu usb_failed=%lu failures=%lu timeouts=%lu acquisition_max_us=%lu driver_lifetime_max_poll_us=%lu\n",
            static_cast<unsigned long>(normal.sample_trace.count), static_cast<unsigned long>(normal.usb_trace.count),
            static_cast<unsigned long>(normal.failed_usb), static_cast<unsigned long>(normal.failures - normal.start_failures),
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
        printf("normal_trace retained_last_per_source=512 sample_overwritten=%lu usb_overwritten=%lu; sample_flags=movement:1,tap:2,two_tap:4,scroll:8 usb_flags=submitted:1\n",
            static_cast<unsigned long>(normal.sample_trace.count > kTraceCapacity ? normal.sample_trace.count - kTraceCapacity : 0),
            static_cast<unsigned long>(normal.usb_trace.count > kTraceCapacity ? normal.usb_trace.count - kTraceCapacity : 0));
    } else {
        const uint32_t sample_rows = std::min<uint32_t>(normal.sample_trace.count, kTraceCapacity);
        const uint32_t usb_rows = std::min<uint32_t>(normal.usb_trace.count, kTraceCapacity);
        if (row - 6 < sample_rows) {
            print_trace_row("sample", normal.sample_trace, row - 6);
        } else if (row - 6 - sample_rows < usb_rows) {
            print_trace_row("usb", normal.usb_trace, row - 6 - sample_rows);
        } else {
            printf("NORMAL DONE: summaries cover all 15 seconds; traces retain last 512 events per source; M repeats\n");
            normal.phase = NormalCapture::Phase::Idle;
        }
    }
}
