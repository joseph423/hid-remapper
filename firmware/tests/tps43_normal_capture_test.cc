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
    sample.previous_cycle_time_ms = 13;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, false, 5, 2, 3, 0);
    tps43_normal_capture_start(100);
    fflush(stdout);
    const long before = lseek(fileno(stdout), 0, SEEK_CUR);
    // Settling and stale samples cannot contribute movement or USB intervals.
    sample.timestamp_us = 500;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 5, 2, 3, 400);
    tps43_normal_capture_note_usb(500, true, 99, 0);
    const uint8_t two_finger_report[16] = { 3, 0x07, 0, 0, 0, 0, 0, 0x07, 0, 0, 0, 0, 0, 2, 0, 0 };
    const uint8_t one_finger_report[16] = { 3, 0x07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 };
    tps43_normal_capture_note_digitizer_transfer(500, true, two_finger_report, sizeof(two_finger_report));
    sample.timestamp_us = 1000100;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 5, 2, 3, 400);
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, false, 5, 2, 3, 400);
    tps43_normal_capture_note_usb(1000200, true, -1, 0);
    tps43_normal_capture_note_usb(1000500, false, -1, 0);
    tps43_normal_capture_note_digitizer_transfer(
        1000250, true, two_finger_report, sizeof(two_finger_report));
    tps43_normal_capture_note_digitizer_transfer(
        1000300, true, one_finger_report, sizeof(one_finger_report));
    tps43_normal_capture_note_digitizer_transfer(
        1000550, false, two_finger_report, sizeof(two_finger_report));
    tps43_normal_capture_note_scroll_action(1000600, 4, -8);
    tps43_normal_capture_note_scroll_usb(1000700, true, 1, -2);
    sample.timestamp_us += 1000;
    sample.previous_cycle_time_ms = 8;
    sample.report_rate_missed = true;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 5, 2, 3, 500);
    tps43_normal_capture_note_usb(1001200, true, -1, 0);
    // The capture includes failures even if the driver publishes no sample.
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, false, 6, 3, 4, 0);
    tps43_normal_capture_poll(15000000);
    fflush(stdout);
    assert(lseek(fileno(stdout), 0, SEEK_CUR) == before);  // No printing during recording.
    finish_dump(16000100);
    const auto text = read_output(output);
    assert(text.find("samples=2 usb_attempts=3 usb_failed=1 digitizer_transfer_complete=2 digitizer_transfer_failed=1 failures=1 timeouts=1") != std::string::npos);
    assert(text.find("normal_samples intervals=1 mean_us=1000 max_us=1000") != std::string::npos);
    assert(text.find("normal_usb_submitted intervals=1 mean_us=1000 max_us=1000") != std::string::npos);
    assert(text.find("normal_hid_cursor motion_reports=2 max_delta=1 delta_1_2to3_4to7_8to15_16plus=2,0,0,0,0") != std::string::npos);
    assert(text.find("normal_sensor_rate samples=2 mean_cycle_ms=10 min_cycle_ms=8 max_cycle_ms=13 rr_missed=1") != std::string::npos);
    assert(text.find("normal_scroll raw_samples=0 raw_dx=0 raw_dy=0 action_samples=1 action_q8_dx=4 action_q8_dy=-8 usb_attempts=1 usb_successes=1 usb_wheel=1 usb_pan=-2") != std::string::npos);
    assert(text.find("normal_sample t_us=0 dx=-1 dy=0 count=1 flags=1") != std::string::npos);
    assert(text.find("normal_usb t_us=400 dx=-1 dy=0 count=0 flags=0") != std::string::npos);
    assert(text.find("normal_digitizer payload_reports=2 bad_length=0 contact_count_0=0 contact_count_1=1 contact_count_2=1 contact_count_invalid=0 contact1_active=2 contact1_flags_or=0x07 contact1_flags_and=0x07 contact2_active=1 contact2_flags_or=0x07 contact2_flags_and=0x07") != std::string::npos);
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
    assert(text.find("samples=600 usb_attempts=0 usb_failed=0 digitizer_transfer_complete=0 digitizer_transfer_failed=0 failures=0 timeouts=0") != std::string::npos);
    assert(text.find("normal_counts fingers_0_1_2_3_4_5_invalid=0,600,0,0,0,0,0 delta_0_1_2to3_4to7_8to15_16plus=0,0,0,0,0,0") != std::string::npos);
    assert(text.find("sample_overwritten=88 usb_overwritten=0") != std::string::npos);
    assert(text.find("normal_sample t_us=88000 dx=88") != std::string::npos);
    assert(text.find("normal_sample t_us=599000 dx=599") != std::string::npos);
    assert(text.find("normal_sample t_us=87000 dx=87") == std::string::npos);
}

void test_scroll_capture_counts_unclassified_gesture_deltas(FILE* output) {
    tps43_normal_capture_start(40000000);
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 2;
    sample.scroll_gesture = true;
    sample.movement_reported = false;
    sample.relative_y = -5;
    sample.timestamp_us = 41000001;
    tps43_normal_capture_note_sample(sample.timestamp_us, sample, true, 0, 0, 0, 100);
    finish_dump(56000000);
    const auto text = read_output(output);
    assert(text.find("normal_scroll raw_samples=1 raw_dx=0 raw_dy=-5") != std::string::npos);
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
    test_scroll_capture_counts_unclassified_gesture_deltas(output);
    fflush(stdout);
    assert(dup2(original_stdout, fileno(stdout)) >= 0);
    close(original_stdout);
    fclose(output);
    puts("normal capture tests passed");
}
