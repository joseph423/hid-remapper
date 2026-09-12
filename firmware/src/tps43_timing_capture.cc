#include "tps43_timing_capture.h"

#include <algorithm>
#include <cstdio>

#include "pico/stdlib.h"

namespace {

constexpr size_t kCompactSamples = 20;
constexpr size_t kContactSamples = 20;
constexpr uint64_t kConcurrentDurationUs = 60 * 1000000ULL;

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
    printf("TPS43 async runtime timing capture; M = 15-second normal-use capture (no forced reads)\n");
    printf("STAGE: one-finger compact report samples=%zu\n", kCompactSamples);
    print_sample_prompt();
}

void Tps43TimingCapture::record(
    uint64_t now_us,
    const Tps43Sample& sample,
    const Tps43ServiceTiming& timing,
    const Tps43Sample* diagnostic_contact_sample) {
    if (stage_ == Stage::Concurrent) {
        concurrent_max_service_us_ = std::max(concurrent_max_service_us_, timing.last_service_us);
        if (now_us - concurrent_started_us_ >= kConcurrentDurationUs) {
            finish_concurrent_capture(now_us, timing);
        }
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
            finish_report_stage(now_us, timing);
        }
        return;
    }
}

bool Tps43TimingCapture::wants_diagnostic_contact(
    const Tps43Sample& sample,
    const Tps43ServiceTiming& timing) const {
    return stage_ == Stage::Contact && armed_ && timing.last_sample_published &&
           sample.finger_count != 3;
}

void Tps43TimingCapture::finish_report_stage(uint64_t now_us, const Tps43ServiceTiming& timing) {
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
    start_concurrent_capture(now_us, timing);
}

bool Tps43TimingCapture::diagnostic_contact_requested() const {
    return stage_ == Stage::Contact && armed_;
}

void Tps43TimingCapture::poll_serial() {
    tps43_normal_capture_poll(time_us_64());
    bool enter_received = false;
    int character;
    // Bound input processing even if the host continuously writes CDC data.
    for (unsigned n = 0; n < 32 && (character = getchar_timeout_us(0)) >= 0; ++n) {
        if ((character == 'm' || character == 'M') && !armed_ && stage_ != Stage::Concurrent) {
            tps43_normal_capture_start(time_us_64());
        } else if (character == '\r' || character == '\n') {
            enter_received = true;
        }
    }
    if (enter_received && !armed_ && !tps43_normal_capture_busy() &&
        stage_ != Stage::Concurrent && stage_ != Stage::Complete) {
        armed_ = true;
        captured_samples_ = 0;
        stage_mismatches_ = 0;
        printf("capture_armed=yes; collecting %zu complete reports automatically\n", kCompactSamples);
    }
}

bool Tps43TimingCapture::read_requested() const {
    return armed_;
}

void Tps43TimingCapture::print_sample_prompt() const {
    if (stage_ == Stage::OneFinger) {
        printf("ACTION: establish exactly one stable finger, then press Enter to capture %zu reports automatically\n",
            kCompactSamples);
    } else if (stage_ == Stage::TwoFinger) {
        printf("ACTION: establish exactly two stable fingers, then press Enter to capture %zu reports automatically\n",
            kCompactSamples);
    } else if (stage_ == Stage::Contact) {
        printf("ACTION: establish exactly three stable fingers, then press Enter to capture %zu reports automatically\n",
            kContactSamples);
    }
}

void Tps43TimingCapture::start_concurrent_capture(uint64_t now_us, const Tps43ServiceTiming& timing) {
    stage_ = Stage::Concurrent;
    captured_samples_ = 0;
    concurrent_started_us_ = now_us;
    concurrent_max_service_us_ = 0;
    concurrent_start_failures_ = timing.transfer_failures;
    tps43_reset_runtime_counters();
    printf("STAGE: concurrent sensor, USB, and cursor service duration=60 seconds\n");
    printf("ACTION: use the TPS43 and USB keyboard continuously for 60 seconds; the run is recorded automatically\n");
}

void Tps43TimingCapture::finish_concurrent_capture(uint64_t now_us, const Tps43ServiceTiming& timing) {
    const Tps43RuntimeCounters& end_counters = tps43_runtime_counters();
    const uint64_t elapsed_us = now_us - concurrent_started_us_;
    const uint64_t host_calls = end_counters.usb_host_service_calls;
    const uint64_t host_total_us = end_counters.usb_host_service_total_us;
    const uint64_t device_calls = end_counters.usb_device_service_calls;
    const uint64_t device_total_us = end_counters.usb_device_service_total_us;
    const uint64_t cursor_calls = end_counters.cursor_service_calls;
    const uint64_t cursor_nonzero = end_counters.cursor_nonzero_actions;
    printf(
        "concurrent_elapsed_us=%lu host_service_calls=%lu host_total_us=%lu host_max_us=%lu "
        "device_service_calls=%lu device_total_us=%lu device_max_us=%lu cursor_service_calls=%lu "
        "cursor_nonzero_actions=%lu max_tps43_publish_us=%lu transfer_failures=%lu\n",
        static_cast<unsigned long>(elapsed_us), static_cast<unsigned long>(host_calls),
        static_cast<unsigned long>(host_total_us), static_cast<unsigned long>(end_counters.usb_host_service_max_us),
        static_cast<unsigned long>(device_calls), static_cast<unsigned long>(device_total_us),
        static_cast<unsigned long>(end_counters.usb_device_service_max_us), static_cast<unsigned long>(cursor_calls),
        static_cast<unsigned long>(cursor_nonzero), static_cast<unsigned long>(concurrent_max_service_us_),
        static_cast<unsigned long>(timing.transfer_failures - concurrent_start_failures_));
    printf("RESULT: concurrent timing capture completed; compare USB and cursor observations with the approved error condition\n");
    stage_ = Stage::Complete;
}
