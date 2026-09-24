#include "tps43_timing_capture.h"

#include <cassert>
#include <deque>

uint64_t fake_now = 100;
namespace {
std::deque<int> input;
}

// Supplies queued serial bytes without waiting; no input yields the SDK error.
int getchar_timeout_us(uint32_t timeout_us) {
    assert(timeout_us == 0);
    if (input.empty())
        return -1;
    const int c = input.front();
    input.pop_front();
    return c;
}

// Verifies M/Enter isolation, bounded serial consumption and repeatable captures.
int main() {
    Tps43TimingCapture capture;
    uint16_t requested_interval_ms = 0;
    input = { '8' };
    capture.poll_serial();
    assert(capture.take_active_report_interval_request(requested_interval_ms));
    assert(requested_interval_ms == 8);
    input = { '7' };
    capture.poll_serial();
    assert(capture.take_active_report_interval_request(requested_interval_ms));
    assert(requested_interval_ms == 7);
    input = { 'B' };
    capture.poll_serial();
    assert(capture.take_active_report_interval_request(requested_interval_ms));
    assert(requested_interval_ms == 13);
    assert(!capture.take_active_report_interval_request(requested_interval_ms));
    input = { 'D' };
    capture.poll_serial();
    assert(capture.manual_debug_enabled());
    input = { 'd' };
    capture.poll_serial();
    assert(!capture.manual_debug_enabled());
    input = { 'm', '\r', '\n' };
    capture.poll_serial();
    assert(tps43_normal_capture_busy());
    assert(!capture.read_requested());
    assert(!capture.diagnostic_contact_requested());
    input.assign(100, 'x');
    capture.poll_serial();
    assert(input.size() == 68);
    input.clear();
    fake_now += 16000000;
    for (int i = 0; i < 10; ++i) {
        capture.poll_serial();
        fake_now += 10000;
    }
    assert(!tps43_normal_capture_busy());
    input = { 'm' };
    capture.poll_serial();
    assert(tps43_normal_capture_busy() && !capture.read_requested());
    fake_now += 16000000;
    for (int i = 0; i < 10; ++i) {
        capture.poll_serial();
        fake_now += 10000;
    }
    input = { '\n' };
    capture.poll_serial();
    assert(capture.read_requested());
    input = { 'm' };
    capture.poll_serial();
    assert(!tps43_normal_capture_busy());

    Tps43TimingCapture phase11_capture;
    input = { 'D' };
    phase11_capture.poll_serial();
    assert(phase11_capture.manual_debug_enabled());
    input = { 'C' };
    phase11_capture.poll_serial();
    assert(phase11_capture.phase11_capture_busy());
    assert(!phase11_capture.manual_debug_enabled());
    assert(!tps43_runtime_metrics_enabled());
    Tps43Sample sample;
    Tps43ServiceTiming timing;
    fake_now += 1000000;
    phase11_capture.record_dual_input(fake_now, sample, timing, sample, timing);
    assert(tps43_runtime_metrics_enabled());
    fake_now += 40000000;
    phase11_capture.record_dual_input(fake_now, sample, timing, sample, timing);
    assert(!phase11_capture.phase11_capture_busy());
    assert(!tps43_runtime_metrics_enabled());

    tps43_reset_runtime_counters();
    tps43_set_runtime_metrics_enabled(true);
    tps43_note_pointer_service(false, true, 1000);
    tps43_note_pointer_service(false, true, 1250);
    const Tps43RuntimeCounters& counters = tps43_runtime_counters();
    assert(counters.scroll_report_calls == 2);
    assert(counters.scroll_report_max_gap_us == 250);
    tps43_set_runtime_metrics_enabled(false);
}
