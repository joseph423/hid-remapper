#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace {

constexpr i2c_inst_t* kI2c = i2c0;
constexpr uint8_t kI2cAddress = 0x74;
constexpr uint8_t kSdaPin = 4;
constexpr uint8_t kSclPin = 5;
constexpr uint8_t kRdyPin = 8;
constexpr uint32_t kI2cFrequency = 400000;
constexpr uint16_t kCompactReportRegister = 0x000C;
constexpr size_t kCompactReportBytes = 10;
constexpr uint16_t kContactReportRegister = 0x0016;
constexpr size_t kContactReportBytes = 35;
constexpr size_t kContactSlots = 5;
constexpr size_t kSamplesPerStage = 4;
constexpr size_t kTransitionSamples = 7;
constexpr uint16_t kEndCommunicationRegister = 0xEEEE;

struct CompactReport {
    uint8_t previous_cycle_time;
    uint8_t gesture_events_0;
    uint8_t gesture_events_1;
    uint8_t system_info_0;
    uint8_t system_info_1;
    uint8_t number_of_fingers;
    int16_t relative_x;
    int16_t relative_y;
};

struct Contact {
    uint16_t absolute_x;
    uint16_t absolute_y;
    uint16_t strength;
    uint8_t area;
};

struct ReportSample {
    CompactReport compact;
    Contact contacts[kContactSlots];
    bool contact_details_available;
    uint8_t area_nonzero_slots;
    uint32_t area_nonzero_centroid_x;
    uint32_t area_nonzero_centroid_y;
    uint32_t compact_read_us;
    uint32_t contact_read_us;
    uint32_t total_read_us;
    uint32_t timestamp_us;
};

uint16_t read_big_endian_u16(const uint8_t* data);
int16_t read_big_endian_i16(const uint8_t* data);
bool read_register(uint16_t address, uint8_t* data, size_t length);
bool end_communication_window();
bool read_compact_report(CompactReport& report, uint32_t& duration_us);
bool read_contact_report(ReportSample& sample);
bool read_report_sample(ReportSample& sample);
void decode_contact(Contact& contact, const uint8_t* data);
void print_compact_report(const ReportSample& sample, size_t sample_number);
void print_contact_report(const ReportSample& sample);
void configure_i2c();
void configure_rdy_input();
void wait_for_operator();
void wait_for_report_window();
void capture_stage(const char* name, const char* action);
bool capture_transition_sequence();
[[noreturn]] void fail(const char* reason);

}  // namespace

/**
 * Captures compact TPS43 reports and conditionally captures all contact slots
 * for exactly three fingers without changing sensor configuration or
 * invoking HID output.
 */
int main() {
    stdio_init_all();
    configure_rdy_input();
    configure_i2c();
    sleep_ms(1500);

    printf("TPS43 report probe\n");
    printf("I2C0 SDA=GP%u SCL=GP%u frequency=%lu address=0x%02x\n", kSdaPin, kSclPin, kI2cFrequency,
        kI2cAddress);
    printf("RDY=GP%u active=high\n", kRdyPin);
    printf("compact_register=0x%04x bytes=%zu\n", kCompactReportRegister, kCompactReportBytes);
    printf("contact_register=0x%04x bytes=%zu slots=%zu only_when_fingers=3\n", kContactReportRegister,
        kContactReportBytes, kContactSlots);

    capture_stage("finger-count and compact report", "place one finger and move slightly for each sample");
    capture_stage("one-finger movement", "keep one finger down and move in several directions");
    capture_stage("two-finger compact report", "place two fingers and move them together or separately");
    capture_stage("three-finger contacts and centroid", "place exactly three fingers and move them gently");

    const bool transition_passed = capture_transition_sequence();
    printf("transition_sequence_result=%s\n", transition_passed ? "pass" : "fail");
    printf("RESULT: TPS43 compact, conditional contact, and transition report capture completed\n");
    while (true) {
        tight_loop_contents();
    }
}

namespace {

uint16_t read_big_endian_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0] << 8) | data[1];
}

int16_t read_big_endian_i16(const uint8_t* data) {
    return static_cast<int16_t>(read_big_endian_u16(data));
}

bool read_register(uint16_t address, uint8_t* data, size_t length) {
    const uint8_t register_address[] = {
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xff),
    };
    // Keep the bus active for the repeated-start portion of the documented
    // 16-bit random-read transaction.
    const int write_result =
        i2c_write_blocking(kI2c, kI2cAddress, register_address, sizeof(register_address), true);
    if (write_result != static_cast<int>(sizeof(register_address))) {
        return false;
    }

    return i2c_read_blocking(kI2c, kI2cAddress, data, length, false) == static_cast<int>(length);
}

bool end_communication_window() {
    const uint8_t register_address_and_data[] = {
        static_cast<uint8_t>(kEndCommunicationRegister >> 8),
        static_cast<uint8_t>(kEndCommunicationRegister & 0xff),
        1,
    };
    // A STOP alone does not close the B000 communication window or clear RDY.
    return i2c_write_blocking(kI2c, kI2cAddress, register_address_and_data,
               sizeof(register_address_and_data), false) ==
           static_cast<int>(sizeof(register_address_and_data));
}

bool read_compact_report(CompactReport& report, uint32_t& duration_us) {
    uint8_t data[kCompactReportBytes] = {};
    const uint32_t started_us = time_us_32();
    if (!read_register(kCompactReportRegister, data, sizeof(data))) {
        duration_us = time_us_32() - started_us;
        return false;
    }
    duration_us = time_us_32() - started_us;

    report.previous_cycle_time = data[0];
    report.gesture_events_0 = data[1];
    report.gesture_events_1 = data[2];
    report.system_info_0 = data[3];
    report.system_info_1 = data[4];
    report.number_of_fingers = data[5];
    report.relative_x = read_big_endian_i16(&data[6]);
    report.relative_y = read_big_endian_i16(&data[8]);
    return true;
}

void decode_contact(Contact& contact, const uint8_t* data) {
    contact.absolute_x = read_big_endian_u16(&data[0]);
    contact.absolute_y = read_big_endian_u16(&data[2]);
    contact.strength = read_big_endian_u16(&data[4]);
    contact.area = data[6];
}

bool read_contact_report(ReportSample& sample) {
    uint8_t data[kContactReportBytes] = {};
    const uint32_t started_us = time_us_32();
    if (!read_register(kContactReportRegister, data, sizeof(data))) {
        sample.contact_read_us = time_us_32() - started_us;
        return false;
    }
    sample.contact_read_us = time_us_32() - started_us;

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    sample.area_nonzero_slots = 0;
    for (size_t slot = 0; slot < kContactSlots; ++slot) {
        // The primary slot is at 0x0016; each following tracked slot is seven
        // bytes later. Nonzero area is reported as a diagnostic proxy only.
        decode_contact(sample.contacts[slot], &data[slot * 7]);
        if (sample.contacts[slot].area != 0) {
            ++sample.area_nonzero_slots;
            sum_x += sample.contacts[slot].absolute_x;
            sum_y += sample.contacts[slot].absolute_y;
        }
    }

    if (sample.area_nonzero_slots == 3) {
        // Keep this centroid explicitly tied to the diagnostic area heuristic;
        // production active-slot semantics require physical verification.
        sample.area_nonzero_centroid_x = sum_x / 3;
        sample.area_nonzero_centroid_y = sum_y / 3;
    }
    return true;
}

bool read_report_sample(ReportSample& sample) {
    const uint32_t started_us = time_us_32();
    if (!read_compact_report(sample.compact, sample.compact_read_us)) {
        end_communication_window();
        return false;
    }

    if (sample.compact.number_of_fingers == 3) {
        sample.contact_details_available = true;
        if (!read_contact_report(sample)) {
            end_communication_window();
            return false;
        }
    }

    if (!end_communication_window()) {
        return false;
    }
    sample.total_read_us = time_us_32() - started_us;
    sample.timestamp_us = time_us_32();
    return true;
}

void print_compact_report(const ReportSample& sample, size_t sample_number) {
    const CompactReport& report = sample.compact;
    printf(
        "sample=%zu timestamp_us=%lu total_read_us=%lu compact_read_us=%lu contact_read_us=%lu "
        "previous_cycle_time=%u gesture0=0x%02x gesture1=0x%02x system_info0=0x%02x "
        "system_info1=0x%02x finger_count=%u relative_x=%d relative_y=%d movement=%s "
        "single_tap=%s two_finger_tap=%s scroll=%s\n",
        sample_number, static_cast<unsigned long>(sample.timestamp_us),
        static_cast<unsigned long>(sample.total_read_us), static_cast<unsigned long>(sample.compact_read_us),
        static_cast<unsigned long>(sample.contact_read_us), report.previous_cycle_time, report.gesture_events_0,
        report.gesture_events_1, report.system_info_0, report.system_info_1, report.number_of_fingers,
        report.relative_x, report.relative_y, (report.system_info_1 & 0x01) ? "yes" : "no",
        (report.gesture_events_0 & 0x01) ? "yes" : "no", (report.gesture_events_1 & 0x01) ? "yes" : "no",
        (report.gesture_events_1 & 0x02) ? "yes" : "no");
}

void print_contact_report(const ReportSample& sample) {
    for (size_t slot = 0; slot < kContactSlots; ++slot) {
        const Contact& contact = sample.contacts[slot];
        printf("contact_slot=%zu absolute_x=%u absolute_y=%u strength=%u area=%u area_nonzero=%s\n", slot,
            contact.absolute_x, contact.absolute_y, contact.strength, contact.area,
            contact.area != 0 ? "yes" : "no");
    }

    if (sample.area_nonzero_slots == 3) {
        printf("area_nonzero_centroid_valid=yes centroid_x=%lu centroid_y=%lu\n",
            static_cast<unsigned long>(sample.area_nonzero_centroid_x),
            static_cast<unsigned long>(sample.area_nonzero_centroid_y));
    } else {
        printf("area_nonzero_centroid_valid=no area_nonzero_slots=%u reported_fingers=%u\n",
            sample.area_nonzero_slots, sample.compact.number_of_fingers);
    }
}

void configure_i2c() {
    i2c_init(kI2c, kI2cFrequency);
    gpio_set_function(kSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(kSclPin, GPIO_FUNC_I2C);

    // Leave pull-up selection to the verified sensor breakout and wiring.
    gpio_set_pulls(kSdaPin, false, false);
    gpio_set_pulls(kSclPin, false, false);
}

void configure_rdy_input() {
    gpio_init(kRdyPin);
    gpio_set_dir(kRdyPin, GPIO_IN);
    gpio_set_pulls(kRdyPin, false, false);
}

void wait_for_operator() {
    while (true) {
        const int character = getchar_timeout_us(1000);
        if (character == '\r' || character == '\n') {
            break;
        }
    }

    // The Windows monitor sends CRLF for Enter. Drain the paired line ending
    // so it cannot advance the next sample without another operator action.
    sleep_ms(10);
    while (getchar_timeout_us(0) >= 0) {
    }
}

void wait_for_report_window() {
    while (!gpio_get(kRdyPin)) {
        tight_loop_contents();
    }
}

void capture_stage(const char* name, const char* action) {
    printf("STAGE: %s samples=%zu\n", name, kSamplesPerStage);
    for (size_t sample_number = 1; sample_number <= kSamplesPerStage; ++sample_number) {
        printf("ACTION: %s; press Enter to capture sample %zu\n", action, sample_number);
        wait_for_operator();
        wait_for_report_window();

        ReportSample sample{};
        if (!read_report_sample(sample)) {
            fail("RESULT: TPS43 report capture failed");
        }
        print_compact_report(sample, sample_number);
        if (sample.contact_details_available) {
            print_contact_report(sample);
        }
    }
}

// Keep the approved transition order in one guided capture so missed and
// spurious finger-count reports can be compared without changing production policy.
bool capture_transition_sequence() {
    constexpr uint8_t expected_finger_counts[kTransitionSamples] = { 0, 1, 2, 3, 2, 1, 0 };
    constexpr const char* actions[kTransitionSamples] = {
        "release all fingers and keep the pad inactive",
        "release all fingers, then place one finger and hold it steady",
        "release all fingers, then place two fingers and hold them steady",
        "release all fingers, then place exactly three fingers and hold them steady",
        "release all fingers, then place two fingers and hold them steady",
        "release all fingers, then place one finger and hold it steady",
        "release all fingers and keep the pad inactive",
    };
    bool all_transitions_match = true;

    printf("STAGE: recorded finger-count transitions samples=%zu sequence=0->1->2->3->2->1->0\n",
        kTransitionSamples);
    for (size_t sample_number = 0; sample_number < kTransitionSamples; ++sample_number) {
        printf("ACTION: %s; press Enter to capture transition sample %zu expected_finger_count=%u\n",
            actions[sample_number], sample_number + 1, expected_finger_counts[sample_number]);
        wait_for_operator();
        wait_for_report_window();

        ReportSample sample{};
        if (!read_report_sample(sample)) {
            fail("RESULT: TPS43 transition report capture failed");
        }
        print_compact_report(sample, sample_number + 1);
        if (sample.contact_details_available) {
            print_contact_report(sample);
        }

        const bool transition_matches =
            sample.compact.number_of_fingers == expected_finger_counts[sample_number];
        printf("transition_expected_finger_count=%u observed_finger_count=%u match=%s\n",
            expected_finger_counts[sample_number], sample.compact.number_of_fingers,
            transition_matches ? "yes" : "no");
        all_transitions_match = all_transitions_match && transition_matches;
    }

    return all_transitions_match;
}

[[noreturn]] void fail(const char* reason) {
    printf("%s\n", reason);
    while (true) {
        tight_loop_contents();
    }
}

}  // namespace
