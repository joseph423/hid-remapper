#include "tps43_timing_capture.h"

#include <cassert>
#include <deque>
#include <vector>

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

// Verifies line framing, fail-closed terminal input, and opt-in commands.
int main() {
    Tps43TimingCapture capture;
    uint16_t requested_interval_ms = 0;

    // A bare Enter never arms a capture; commands execute only after newline.
    input = { '\r', '\n' };
    capture.poll_serial();
    assert(!capture.read_requested());

    // Rate changes are unavailable in normal builds and explicit in the
    // optional diagnostic build; 7 ms is no longer accepted by the driver.
    input = { 'r', 'a', 't', 'e', ' ', 'r', 'i', 'g', 'h', 't', ' ', '1', '3', '\n' };
    capture.poll_serial();
#ifdef TPS43_ENABLE_SERIAL_RATE_OVERRIDE
    assert(capture.take_active_report_interval_request(requested_interval_ms));
    assert(requested_interval_ms == 13);
    assert(!capture.take_active_report_interval_request(requested_interval_ms));
    input = { 'r', 'a', 't', 'e', ' ', 'r', 'i', 'g', 'h', 't', ' ', '8', '\n' };
    capture.poll_serial();
    assert(capture.take_active_report_interval_request(requested_interval_ms));
    assert(requested_interval_ms == 8);
#else
    assert(!capture.take_active_report_interval_request(requested_interval_ms));
#endif
    input = { '7', '\n', '8', '\n', 'B', '\n' };
    capture.poll_serial();
    assert(!capture.take_active_report_interval_request(requested_interval_ms));

    input = { 'M' };
    capture.poll_serial();
    assert(!tps43_normal_capture_busy());
    input = { '\n' };
    capture.poll_serial();
    assert(tps43_normal_capture_busy());
    assert(!capture.read_requested());

    // Terminal escape/mouse bytes invalidate the whole line, even when split
    // across polls and followed by characters that are valid commands alone.
    const std::vector<std::vector<int>> terminal_sequences = {
        { 0x1b, '[', 'B' },
        { 0x1b, '[', '7', '~' },
        { 0x1b, 'O', 'B' },
        { 0x1b, '[', '<', '0', ';', '2', ';', '3', 'M' },
        { 0x1b, '[', '8' },
        { 0x1b, '[', 'D' },
        { 0x1b, '[', 'C' },
    };
    for (const auto& sequence : terminal_sequences) {
        input.assign(sequence.begin(), sequence.end());
        input.push_back('\r');
        input.push_back('\n');
        capture.poll_serial();
        assert(!capture.manual_debug_enabled());
        assert(tps43_normal_capture_busy());
    }
    input = { 0xc3, 'M', '\n' };
    capture.poll_serial();
    assert(tps43_normal_capture_busy());

    // 15-second measurement remains available with an explicit command line.
    fake_now += 16000000;
    for (int i = 0; i < 10; ++i) {
        capture.poll_serial();
        fake_now += 10000;
    }
    assert(!tps43_normal_capture_busy());
    input = { 'm', '\r', '\n' };
    capture.poll_serial();
    assert(tps43_normal_capture_busy());
    assert(!capture.read_requested());

    // Oversized lines are discarded through their delimiter and do not
    // accidentally execute a command embedded after the buffer overflow.
    input.assign(60, 'x');
    input.push_back('D');
    input.push_back('\n');
    capture.poll_serial();
    while (!input.empty())
        capture.poll_serial();
    assert(!capture.manual_debug_enabled());

    // Split command lines remain pending until completed; D executes once.
    input = { 'D' };
    capture.poll_serial();
    assert(!capture.manual_debug_enabled());
    input = { '\r', '\n' };
    capture.poll_serial();
    assert(capture.manual_debug_enabled());
    input = { 'd', '\n' };
    capture.poll_serial();
    assert(!capture.manual_debug_enabled());

    fake_now += 16000000;
    for (int i = 0; i < 10; ++i) {
        capture.poll_serial();
        fake_now += 10000;
    }
    assert(!tps43_normal_capture_busy());

    // An explicit capture line arms a staged report capture; bare Enter does not.
    Tps43TimingCapture staged_capture;
    input = { '\n' };
    staged_capture.poll_serial();
    assert(!staged_capture.read_requested());
    input = { 'c', 'a', 'p', 't', 'u', 'r', 'e', '\n' };
    staged_capture.poll_serial();
    assert(staged_capture.read_requested());
    assert(!staged_capture.diagnostic_contact_requested());

    // C starts the existing Phase 11 capture; D remains an explicit toggle.
    Tps43TimingCapture phase11_capture;
    input = { 'D', '\n' };
    phase11_capture.poll_serial();
    assert(phase11_capture.manual_debug_enabled());
    input = { 'C', '\n' };
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
