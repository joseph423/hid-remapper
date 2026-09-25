#include "tps43_timing_capture.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"

namespace {

constexpr size_t kCompactSamples = 20;
constexpr size_t kContactSamples = 20;
constexpr uint64_t kConcurrentSettlingUs = 1000000;
constexpr uint64_t kConcurrentDurationUs = 40 * 1000000ULL;
constexpr uint64_t kManualDebugIntervalUs = 100000;

uint32_t absolute_delta(int32_t value) {
    return static_cast<uint32_t>(value < 0 ? -static_cast<int64_t>(value) : value);
}

void print_contact_slots(const Tps43Sample& sample) {
    for (size_t slot = 0; slot < sample.contacts.size(); ++slot) {
        const Tps43Contact& contact = sample.contacts[slot];
        printf("diagnostic_contact_slot=%zu absolute_x=%u absolute_y=%u strength=%u area=%u active=%s\n",
            slot, contact.x, contact.y, contact.strength, contact.area,
            contact.active ? "yes" : "no");
    }
}

}  // namespace

void Tps43TimingCapture::begin() {
    printf("TPS43 serial commands are complete lines; M=normal capture, D=debug toggle, C=Phase 11, capture=arm report stage\n");
#ifdef TPS43_ENABLE_SERIAL_RATE_OVERRIDE
    printf("Diagnostic only: rate right 8|13 = request volatile report interval in ms\n");
#endif
    printf("STAGE: one-finger compact report samples=%zu\n", kCompactSamples);
    print_sample_prompt();
}

void Tps43TimingCapture::record(
    uint64_t now_us,
    const Tps43Sample& sample,
    const Tps43ServiceTiming& timing,
    const Tps43Sample* diagnostic_contact_sample) {
    if (stage_ == Stage::ConcurrentSettling || stage_ == Stage::Concurrent) {
        return;
    }

    if (!timing.last_sample_published) {
        return;
    }

    if (stage_ == Stage::OneFinger || stage_ == Stage::TwoFinger || stage_ == Stage::Contact) {
        if (!armed_) {
            return;
        }

        const uint8_t expected_finger_count = stage_ == Stage::OneFinger ? 1 : stage_ == Stage::TwoFinger ? 2
                                                                                                          : 3;
        const bool matches = sample.finger_count == expected_finger_count &&
                             (stage_ != Stage::Contact || timing.last_sample_had_contact_read);

        ++captured_samples_;
        if (!matches) {
            ++stage_mismatches_;
        }

        printf(
            "timing_sample=%zu finger_count=%u expected=%s match=%s publish_us=%lu "
            "compact_read_us=%lu contact_read_us=%lu acquisition_us=%lu lifetime_poll_max_us=%lu\n",
            captured_samples_, sample.finger_count,
            stage_ == Stage::OneFinger ? "1" : stage_ == Stage::TwoFinger ? "2"
                                                                          : "3_with_contact",
            matches ? "yes" : "no",
            static_cast<unsigned long>(timing.last_service_us),
            static_cast<unsigned long>(timing.last_compact_read_us),
            static_cast<unsigned long>(timing.last_contact_read_us),
            static_cast<unsigned long>(timing.last_acquisition_us),
            static_cast<unsigned long>(timing.max_poll_us));
        if (diagnostic_contact_sample != nullptr) {
            printf("diagnostic_contact_read=%s diagnostic_contact_read_us=%lu\n",
                diagnostic_contact_sample->contact_details_available ? "ok" : "fail",
                static_cast<unsigned long>(timing.last_diagnostic_contact_read_us));
            if (diagnostic_contact_sample->contact_details_available) {
                print_contact_slots(*diagnostic_contact_sample);
            }
        }
        if (((stage_ == Stage::OneFinger || stage_ == Stage::TwoFinger) && captured_samples_ == kCompactSamples) ||
            (stage_ == Stage::Contact && captured_samples_ == kContactSamples)) {
            finish_report_stage(now_us);
        }
        return;
    }
}

void Tps43TimingCapture::record_dual_input(
    uint64_t now_us,
    const Tps43Sample& left_sample,
    const Tps43ServiceTiming& left_timing,
    const Tps43Sample& right_sample,
    const Tps43ServiceTiming& right_timing) {
    if (stage_ == Stage::ConcurrentSettling && now_us >= concurrent_start_at_us_) {
        start_concurrent_capture(now_us, left_timing, right_timing);
    }
    if (stage_ == Stage::Concurrent) {
        update_concurrent_capture(now_us, left_sample, left_timing, right_sample, right_timing);
        return;
    }

    if (manual_debug_enabled_) {
        record_input("left", left_sample, left_timing, left_input_session_);
        record_input("right", right_sample, right_timing, right_input_session_);
    }

    if (!manual_debug_enabled_ || now_us - last_manual_debug_us_ < kManualDebugIntervalUs ||
        (!left_timing.last_sample_published && !right_timing.last_sample_published)) {
        return;
    }
    last_manual_debug_us_ = now_us;
    printf(
        "TPS43 debug timestamp_us=%lu left_fresh=%s left_active=%s left_fingers=%u left_dx=%ld left_dy=%ld "
        "left_move=%s left_single_tap=%s left_two_finger_tap=%s left_scroll=%s left_failures=%lu left_timeouts=%lu "
        "right_fresh=%s right_active=%s right_fingers=%u right_dx=%ld right_dy=%ld right_move=%s "
        "right_single_tap=%s right_two_finger_tap=%s right_scroll=%s right_failures=%lu right_timeouts=%lu\n",
        static_cast<unsigned long>(now_us), left_timing.last_sample_published ? "yes" : "no",
        left_sample.active ? "yes" : "no", left_sample.finger_count, static_cast<long>(left_sample.relative_x),
        static_cast<long>(left_sample.relative_y), left_sample.movement_reported ? "yes" : "no",
        left_sample.single_tap ? "yes" : "no", left_sample.two_finger_tap ? "yes" : "no",
        left_sample.scroll_gesture ? "yes" : "no", static_cast<unsigned long>(left_timing.transfer_failures),
        static_cast<unsigned long>(left_timing.transfer_timeouts), right_timing.last_sample_published ? "yes" : "no",
        right_sample.active ? "yes" : "no", right_sample.finger_count, static_cast<long>(right_sample.relative_x),
        static_cast<long>(right_sample.relative_y), right_sample.movement_reported ? "yes" : "no",
        right_sample.single_tap ? "yes" : "no", right_sample.two_finger_tap ? "yes" : "no",
        right_sample.scroll_gesture ? "yes" : "no", static_cast<unsigned long>(right_timing.transfer_failures),
        static_cast<unsigned long>(right_timing.transfer_timeouts));
}

bool Tps43TimingCapture::manual_debug_enabled() const {
    return manual_debug_enabled_;
}

bool Tps43TimingCapture::phase11_capture_busy() const {
    return stage_ == Stage::ConcurrentSettling || stage_ == Stage::Concurrent;
}

void Tps43TimingCapture::record_input(
    const char* pad_name,
    const Tps43Sample& sample,
    const Tps43ServiceTiming& timing,
    InputSession& session) {
    if (!timing.last_sample_published) {
        return;
    }

    if (sample.active && !session.active) {
        session = {};
        session.active = true;
        session.started_us = sample.timestamp_us;
        printf("TPS43 input pad=%s event=touch_start timestamp_us=%lu fingers=%u\n",
            pad_name, static_cast<unsigned long>(sample.timestamp_us), sample.finger_count);
    }

    if (sample.active && session.active) {
        ++session.fresh_samples;
        session.relative_x += sample.relative_x;
        session.relative_y += sample.relative_y;
        if (sample.movement_reported) {
            ++session.movement_samples;
        }
        session.maximum_delta = std::max(session.maximum_delta,
            std::max(absolute_delta(sample.relative_x), absolute_delta(sample.relative_y)));
    }

    if (!sample.active && session.active) {
        const uint64_t duration_us = sample.timestamp_us >= session.started_us
                                         ? sample.timestamp_us - session.started_us
                                         : 0;
        printf(
            "TPS43 input pad=%s event=touch_end duration_us=%lu fresh_samples=%lu "
            "movement_samples=%lu net_dx=%ld net_dy=%ld max_delta=%lu single_tap=%s two_finger_tap=%s\n",
            pad_name, static_cast<unsigned long>(duration_us),
            static_cast<unsigned long>(session.fresh_samples),
            static_cast<unsigned long>(session.movement_samples),
            static_cast<long>(session.relative_x), static_cast<long>(session.relative_y),
            static_cast<unsigned long>(session.maximum_delta), sample.single_tap ? "yes" : "no",
            sample.two_finger_tap ? "yes" : "no");
        session = {};
    }
}

bool Tps43TimingCapture::wants_diagnostic_contact(
    const Tps43Sample& sample,
    const Tps43ServiceTiming& timing) const {
    return stage_ == Stage::Contact && armed_ && timing.last_sample_published &&
           sample.finger_count != 3;
}

void Tps43TimingCapture::finish_report_stage(uint64_t now_us) {
    armed_ = false;
    if (stage_ == Stage::OneFinger) {
        one_finger_mismatches_ = stage_mismatches_;
        printf("one_finger_count_result=%s mismatches=%zu\n",
            one_finger_mismatches_ == 0 ? "pass" : "fail", one_finger_mismatches_);
        stage_ = Stage::TwoFinger;
        captured_samples_ = 0;
        stage_mismatches_ = 0;
        printf("STAGE: two-finger compact report samples=%zu\n", kCompactSamples);
        print_sample_prompt();
        return;
    }

    if (stage_ == Stage::TwoFinger) {
        two_finger_mismatches_ = stage_mismatches_;
        printf("two_finger_count_result=%s mismatches=%zu\n",
            two_finger_mismatches_ == 0 ? "pass" : "fail", two_finger_mismatches_);
        stage_ = Stage::Contact;
        captured_samples_ = 0;
        stage_mismatches_ = 0;
        printf("STAGE: conditional three-finger contact report samples=%zu\n", kContactSamples);
        print_sample_prompt();
        return;
    }

    contact_mismatches_ = stage_mismatches_;
    printf("contact_count_result=%s mismatches=%zu\n",
        contact_mismatches_ == 0 ? "pass" : "fail", contact_mismatches_);
    printf(
        "report_count_result=%s one_finger_mismatches=%zu two_finger_mismatches=%zu "
        "contact_mismatches=%zu\n",
        one_finger_mismatches_ == 0 && two_finger_mismatches_ == 0 && contact_mismatches_ == 0 ? "pass" : "fail",
        one_finger_mismatches_, two_finger_mismatches_, contact_mismatches_);
    request_concurrent_capture(now_us);
}

bool Tps43TimingCapture::diagnostic_contact_requested() const {
    return stage_ == Stage::Contact && armed_;
}

void Tps43TimingCapture::poll_serial() {
    tps43_normal_capture_poll(time_us_64());
    int character;
    // Bound input processing even if the host continuously writes CDC data.
    for (unsigned n = 0; n < 32 && (character = getchar_timeout_us(0)) >= 0; ++n) {
        if (serial_ignore_next_lf_) {
            serial_ignore_next_lf_ = false;
            if (character == '\n') {
                continue;
            }
        }

        if (character == '\r' || character == '\n') {
            if (serial_discarding_line_) {
                serial_discarding_line_ = false;
                serial_line_length_ = 0;
            } else {
                serial_line_[serial_line_length_] = '\0';
                dispatch_serial_line();
                serial_line_length_ = 0;
            }
            serial_line_[0] = '\0';
            serial_ignore_next_lf_ = character == '\r';
            continue;
        }

        // Terminals can send ESC/CSI/SS3 and mouse-report bytes on scroll.
        // Reject the entire line and stay in discard mode through its delimiter.
        if (character < 0x20 || character > 0x7e) {
            serial_discarding_line_ = true;
            serial_line_length_ = 0;
            serial_line_[0] = '\0';
            continue;
        }
        if (!serial_discarding_line_) {
            if (serial_line_length_ + 1 >= kSerialLineCapacity) {
                serial_discarding_line_ = true;
                serial_line_length_ = 0;
                serial_line_[0] = '\0';
            } else {
                serial_line_[serial_line_length_++] = static_cast<char>(character);
                serial_line_[serial_line_length_] = '\0';
            }
        }
    }
}

void Tps43TimingCapture::dispatch_serial_line() {
    if (strcmp(serial_line_, "M") == 0 || strcmp(serial_line_, "m") == 0) {
        if (!armed_ && !phase11_capture_busy()) {
            tps43_normal_capture_start(time_us_64());
        }
        return;
    }
    if (strcmp(serial_line_, "D") == 0 || strcmp(serial_line_, "d") == 0) {
        if (!phase11_capture_busy()) {
            manual_debug_enabled_ = !manual_debug_enabled_;
            last_manual_debug_us_ = 0;
            printf("manual_debug=%s interval_ms=100 source=cached_compact_samples\n",
                manual_debug_enabled_ ? "on" : "off");
        }
        return;
    }
    if (strcmp(serial_line_, "C") == 0 || strcmp(serial_line_, "c") == 0) {
        if (!armed_ && !tps43_normal_capture_busy() && !phase11_capture_busy()) {
            manual_debug_enabled_ = false;
            left_input_session_ = {};
            right_input_session_ = {};
            request_concurrent_capture(time_us_64());
        }
        return;
    }
    if (strcmp(serial_line_, "capture") == 0) {
        if (!armed_ && !tps43_normal_capture_busy() &&
            !phase11_capture_busy() && stage_ != Stage::Complete) {
            armed_ = true;
            captured_samples_ = 0;
            stage_mismatches_ = 0;
            printf("capture_armed=yes; collecting %zu complete reports automatically\n", kCompactSamples);
        }
        return;
    }
#ifdef TPS43_ENABLE_SERIAL_RATE_OVERRIDE
    if (strcmp(serial_line_, "rate right 8") == 0 || strcmp(serial_line_, "rate right 13") == 0) {
        requested_active_report_interval_ms_ = strcmp(serial_line_, "rate right 8") == 0
                                                   ? kTps43DefaultActiveReportIntervalMs
                                                   : kTps43BaselineActiveReportIntervalMs;
        if (!armed_ && !tps43_normal_capture_busy() && !phase11_capture_busy() &&
            !active_report_interval_request_pending_) {
            active_report_interval_request_pending_ = true;
            printf("TPS43 active_report_interval_request_ms=%u pad=right result=queued persistence=volatile\n",
                requested_active_report_interval_ms_);
        } else {
            printf("TPS43 active_report_interval_request_ms=%u pad=right result=busy\n",
                requested_active_report_interval_ms_);
        }
    }
#endif
}

bool Tps43TimingCapture::take_active_report_interval_request(uint16_t& interval_ms) {
    if (!active_report_interval_request_pending_) {
        return false;
    }
    interval_ms = requested_active_report_interval_ms_;
    active_report_interval_request_pending_ = false;
    return true;
}

bool Tps43TimingCapture::read_requested() const {
    return armed_;
}

void Tps43TimingCapture::print_sample_prompt() const {
    if (stage_ == Stage::OneFinger) {
        printf("ACTION: establish exactly one stable finger, then send capture + Enter to capture %zu reports automatically\n",
            kCompactSamples);
    } else if (stage_ == Stage::TwoFinger) {
        printf("ACTION: establish exactly two stable fingers, then send capture + Enter to capture %zu reports automatically\n",
            kCompactSamples);
    } else if (stage_ == Stage::Contact) {
        printf("ACTION: establish exactly three stable fingers, then send capture + Enter to capture %zu reports automatically\n",
            kContactSamples);
    }
}

void Tps43TimingCapture::request_concurrent_capture(uint64_t now_us) {
    stage_ = Stage::ConcurrentSettling;
    concurrent_start_at_us_ = now_us + kConcurrentSettlingUs;
    printf("PHASE11: starts in 1 second; run the four 10-second blocks for 40 seconds\n");
}

void Tps43TimingCapture::start_concurrent_capture(uint64_t now_us, const Tps43ServiceTiming& left_timing, const Tps43ServiceTiming& right_timing) {
    stage_ = Stage::Concurrent;
    concurrent_started_us_ = now_us;
    concurrent_left_ = {};
    concurrent_right_ = {};
    concurrent_left_.start_failures = left_timing.transfer_failures;
    concurrent_left_.start_timeouts = left_timing.transfer_timeouts;
    concurrent_right_.start_failures = right_timing.transfer_failures;
    concurrent_right_.start_timeouts = right_timing.transfer_timeouts;
    tps43_reset_runtime_counters();
    tps43_set_runtime_metrics_enabled(true);
    printf("PHASE11: recording 40 seconds; output is suppressed until the summary\n");
}

void Tps43TimingCapture::update_concurrent_pad(uint64_t now_us, const Tps43Sample& sample, const Tps43ServiceTiming& timing, ConcurrentPadMetrics& metrics) {
    metrics.service_max_us = std::max(metrics.service_max_us, timing.last_service_us);
    if (metrics.last_service_us != 0 && now_us >= metrics.last_service_us) {
        const uint64_t gap_us = now_us - metrics.last_service_us;
        metrics.service_max_gap_us = std::max(metrics.service_max_gap_us,
            static_cast<uint32_t>(std::min<uint64_t>(gap_us, UINT32_MAX)));
    }
    metrics.last_service_us = now_us;
    if (timing.last_sample_published) {
        ++metrics.samples;
        metrics.movement_samples += sample.movement_reported;
    }
}

void Tps43TimingCapture::update_concurrent_capture(uint64_t now_us, const Tps43Sample& left_sample, const Tps43ServiceTiming& left_timing, const Tps43Sample& right_sample, const Tps43ServiceTiming& right_timing) {
    update_concurrent_pad(now_us, left_sample, left_timing, concurrent_left_);
    update_concurrent_pad(now_us, right_sample, right_timing, concurrent_right_);
    if (now_us - concurrent_started_us_ >= kConcurrentDurationUs) {
        finish_concurrent_capture(now_us, left_timing, right_timing);
    }
}

void Tps43TimingCapture::finish_concurrent_capture(uint64_t now_us, const Tps43ServiceTiming& left_timing, const Tps43ServiceTiming& right_timing) {
    const Tps43RuntimeCounters& end_counters = tps43_runtime_counters();
    const uint64_t elapsed_us = now_us - concurrent_started_us_;
    const uint64_t host_calls = end_counters.usb_host_service_calls;
    const uint64_t host_total_us = end_counters.usb_host_service_total_us;
    const uint64_t device_calls = end_counters.usb_device_service_calls;
    const uint64_t device_total_us = end_counters.usb_device_service_total_us;
    const uint64_t cursor_calls = end_counters.cursor_service_calls;
    const uint64_t cursor_nonzero = end_counters.cursor_nonzero_actions;
    const uint64_t scroll_calls = end_counters.scroll_report_calls;
    printf(
        "PHASE11 elapsed_us=%lu left_samples=%lu left_movement=%lu left_service_max_us=%lu left_service_gap_max_us=%lu "
        "left_failures=%lu left_timeouts=%lu right_samples=%lu right_movement=%lu right_service_max_us=%lu "
        "right_service_gap_max_us=%lu right_failures=%lu right_timeouts=%lu host_service_calls=%lu host_total_us=%lu "
        "host_max_us=%lu host_gap_max_us=%lu device_service_calls=%lu device_total_us=%lu device_max_us=%lu "
        "device_gap_max_us=%lu cursor_service_calls=%lu cursor_nonzero_actions=%lu "
        "scroll_report_calls=%lu scroll_report_gap_max_us=%lu\n",
        static_cast<unsigned long>(elapsed_us), static_cast<unsigned long>(concurrent_left_.samples),
        static_cast<unsigned long>(concurrent_left_.movement_samples), static_cast<unsigned long>(concurrent_left_.service_max_us),
        static_cast<unsigned long>(concurrent_left_.service_max_gap_us),
        static_cast<unsigned long>(left_timing.transfer_failures - concurrent_left_.start_failures),
        static_cast<unsigned long>(left_timing.transfer_timeouts - concurrent_left_.start_timeouts),
        static_cast<unsigned long>(concurrent_right_.samples), static_cast<unsigned long>(concurrent_right_.movement_samples),
        static_cast<unsigned long>(concurrent_right_.service_max_us),
        static_cast<unsigned long>(concurrent_right_.service_max_gap_us),
        static_cast<unsigned long>(right_timing.transfer_failures - concurrent_right_.start_failures),
        static_cast<unsigned long>(right_timing.transfer_timeouts - concurrent_right_.start_timeouts),
        static_cast<unsigned long>(host_calls), static_cast<unsigned long>(host_total_us),
        static_cast<unsigned long>(end_counters.usb_host_service_max_us),
        static_cast<unsigned long>(end_counters.usb_host_service_max_gap_us), static_cast<unsigned long>(device_calls),
        static_cast<unsigned long>(device_total_us), static_cast<unsigned long>(end_counters.usb_device_service_max_us),
        static_cast<unsigned long>(end_counters.usb_device_service_max_gap_us), static_cast<unsigned long>(cursor_calls),
        static_cast<unsigned long>(cursor_nonzero), static_cast<unsigned long>(scroll_calls),
        static_cast<unsigned long>(end_counters.scroll_report_max_gap_us));
    printf("PHASE11 DONE: review the summary against approved conditions before setting timing limits\n");
    tps43_set_runtime_metrics_enabled(false);
    stage_ = Stage::Complete;
}
