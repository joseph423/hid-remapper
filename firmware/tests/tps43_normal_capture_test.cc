#include "tps43_timing_metrics.h"

#include <unistd.h>
#include <cassert>
#include <cstdio>
#include <string>

namespace {

void finish_dump(uint64_t now) {
    for (unsigned i = 0; i < 1100 && tps43_normal_capture_busy(); ++i) {
        tps43_normal_capture_poll(now + i * 10000ULL);
    }
    assert(!tps43_normal_capture_busy());
}

std::string read_output(FILE* output) {
    fflush(stdout);
    fseek(output, 0, SEEK_END);
    const long length = ftell(output);
    rewind(output);
    std::string text(length, '\0');
    assert(fread(text.data(), 1, text.size(), output) == text.size());
    fseek(output, 0, SEEK_END);
    return text;
}

void test_capture(FILE* output) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 1;
    sample.movement_reported = true;
    sample.relative_x = -1;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, false, 5, 2, 3, 0);
    tps43_normal_capture_start(100);
    fflush(stdout);
    const long before = lseek(fileno(stdout), 0, SEEK_CUR);
    // Settling and stale samples cannot contribute movement or USB intervals.
    sample.timestamp_us = 500;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 5, 2, 3, 400);
    tps43_normal_capture_note_usb(500, true, 99, 0);
    sample.timestamp_us = 1000100;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 5, 2, 3, 400);
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, false, 5, 2, 3, 400);
    tps43_normal_capture_note_usb(1000200, true, -1, 0);
    tps43_normal_capture_note_usb(1000500, false, -1, 0);
    sample.timestamp_us += 1000;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 5, 2, 3, 500);
    tps43_normal_capture_note_usb(1001200, true, -1, 0);
    // The capture includes failures even if the driver publishes no sample.
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, false, 6, 3, 4, 0);
    tps43_normal_capture_poll(15000000);
    fflush(stdout);
    assert(lseek(fileno(stdout), 0, SEEK_CUR) == before);  // No printing during recording.
    finish_dump(16000100);
    const auto text = read_output(output);
    assert(text.find("samples=2 usb_attempts=3 usb_failed=1 failures=1 timeouts=1") != std::string::npos);
    assert(text.find("normal_samples intervals=1 mean_us=1000 max_us=1000") != std::string::npos);
    assert(text.find("normal_usb_submitted intervals=1 mean_us=1000 max_us=1000") != std::string::npos);
    assert(text.find("normal_sample t_us=0 dx=-1 dy=0 count=1 flags=1") != std::string::npos);
    assert(text.find("normal_usb t_us=400 dx=-1 dy=0 count=0 flags=0") != std::string::npos);
}

void test_ring_and_repeat(FILE* output) {
    tps43_normal_capture_start(20000000);
    Tps43Sample sample;
    sample.finger_count = 1;
    for (unsigned i = 0; i < 600; ++i) {
        sample.timestamp_us = 21000000 + i * 1000;
        sample.relative_x = i;
        tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 6, 3, 4, 500);
    }
    finish_dump(36000000);
    const auto text = read_output(output);
    assert(text.find("samples=600 usb_attempts=0 usb_failed=0 failures=0 timeouts=0") != std::string::npos);
    assert(text.find("sample_overwritten=88 usb_overwritten=0") != std::string::npos);
    assert(text.find("normal_sample t_us=88000 dx=88") != std::string::npos);
    assert(text.find("normal_sample t_us=599000 dx=599") != std::string::npos);
    assert(text.find("normal_sample t_us=87000 dx=87") == std::string::npos);
}

}  // namespace

// Verifies silent recording, USB failure accounting, intervals and ring retention.
int main() {
    FILE* output = tmpfile();
    assert(output);
    const int original_stdout = dup(fileno(stdout));
    assert(dup2(fileno(output), fileno(stdout)) >= 0);
    test_capture(output);
    test_ring_and_repeat(output);
    fflush(stdout);
    assert(dup2(original_stdout, fileno(stdout)) >= 0);
    close(original_stdout);
    fclose(output);
    puts("normal capture tests passed");
}
