#include "tps43_iqs5xx_driver.h"

#include <cstddef>

#include <hardware/gpio.h>
#include <pico/time.h>

namespace {

constexpr uint16_t kCompactReportRegister = 0x000C;
constexpr uint8_t kCompactReportBytes = 10;
constexpr uint16_t kContactReportRegister = 0x0016;
constexpr uint8_t kContactReportBytes = 35;
constexpr uint8_t kContactRecordBytes = 7;
constexpr uint8_t kContactSlots = 5;
constexpr uint16_t kEndCommunicationRegister = 0xEEEE;

uint16_t read_big_endian_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0] << 8) | data[1];
}

int16_t read_big_endian_i16(const uint8_t* data) {
    return static_cast<int16_t>(read_big_endian_u16(data));
}

}  // namespace

Tps43Iqs5xxDriver::Tps43Iqs5xxDriver(Tps43Iqs5xxConfig config)
    : config_(config) {
}

bool Tps43Iqs5xxDriver::initialize() {
    if (config_.bus == nullptr) {
        return false;
    }

    i2c_init(config_.bus, config_.i2c_frequency_hz);
    gpio_set_function(config_.sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config_.scl_pin, GPIO_FUNC_I2C);

    // Pull-up selection remains a property of the verified breakout and
    // interconnect, so the Pico does not add an internal pull here.
    gpio_set_pulls(config_.sda_pin, false, false);
    gpio_set_pulls(config_.scl_pin, false, false);

    gpio_init(config_.rdy_pin);
    gpio_set_dir(config_.rdy_pin, GPIO_IN);
    gpio_set_pulls(config_.rdy_pin, false, false);

    initialized_ = true;
    return true;
}

bool Tps43Iqs5xxDriver::service(uint64_t now_us) {
    const uint32_t started_us = time_us_32();
    const bool force_communication = forced_read_requested_;
    forced_read_requested_ = false;
    timing_.last_sample_had_contact_read = false;
    timing_.last_sample_published = false;
    timing_.last_compact_read_us = 0;
    timing_.last_contact_read_us = 0;

    const auto finish = [&](bool success) {
        timing_.last_service_us = time_us_32() - started_us;
        return success;
    };

    if (!initialized_ || (!force_communication && !gpio_get(config_.rdy_pin))) {
        return finish(false);
    }

    Tps43Sample next_sample;
    if (!read_compact_report(next_sample, force_communication)) {
        ++timing_.transfer_failures;
        end_communication_window();
        return finish(false);
    }

    if (next_sample.finger_count == 3) {
        if (!read_contact_report(next_sample, force_communication)) {
            ++timing_.transfer_failures;
            end_communication_window();
            return finish(false);
        }
        timing_.last_sample_had_contact_read = true;
    }

    if (!end_communication_window()) {
        ++timing_.transfer_failures;
        return finish(false);
    }

    next_sample.timestamp_us = time_us_64();
    if (next_sample.timestamp_us < now_us) {
        next_sample.timestamp_us = now_us;
    }
    sample_ = next_sample;
    timing_.last_sample_published = true;
    return finish(true);
}

void Tps43Iqs5xxDriver::request_forced_read() {
    forced_read_requested_ = true;
}

bool Tps43Iqs5xxDriver::read_contact_report_for_diagnostic(Tps43Sample& sample) {
    const uint32_t started_us = time_us_32();
    timing_.last_diagnostic_contact_read_us = 0;
    const bool read_succeeded = read_contact_report(sample, true);
    const bool window_closed = end_communication_window();
    timing_.last_diagnostic_contact_read_us = time_us_32() - started_us;
    if (!read_succeeded || !window_closed) {
        ++timing_.transfer_failures;
        return false;
    }
    return true;
}

Tps43Sample Tps43Iqs5xxDriver::sample() const {
    return sample_;
}

const Tps43ServiceTiming& Tps43Iqs5xxDriver::timing() const {
    return timing_;
}

bool Tps43Iqs5xxDriver::read_register(uint16_t address, uint8_t* data, uint8_t length) {
    const uint8_t register_address[] = {
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xff),
    };
    const int write_result = i2c_write_blocking(
        config_.bus, config_.address, register_address, sizeof(register_address), true);
    if (write_result != static_cast<int>(sizeof(register_address))) {
        return false;
    }

    return i2c_read_blocking(config_.bus, config_.address, data, length, false) ==
           static_cast<int>(length);
}

bool Tps43Iqs5xxDriver::read_register_forced(uint16_t address, uint8_t* data, uint8_t length) {
    if (read_register(address, data, length)) {
        return true;
    }

    // A low-power Event Mode request may NACK once while the sensor wakes.
    sleep_us(150);
    return read_register(address, data, length);
}

bool Tps43Iqs5xxDriver::end_communication_window() {
    const uint8_t register_address_and_data[] = {
        static_cast<uint8_t>(kEndCommunicationRegister >> 8),
        static_cast<uint8_t>(kEndCommunicationRegister & 0xff),
        1,
    };
    return i2c_write_blocking(
               config_.bus, config_.address, register_address_and_data,
               sizeof(register_address_and_data), false) ==
           static_cast<int>(sizeof(register_address_and_data));
}

bool Tps43Iqs5xxDriver::read_compact_report(Tps43Sample& sample, bool force_communication) {
    uint8_t data[kCompactReportBytes] = {};
    const uint32_t started_us = time_us_32();
    const bool read_succeeded = force_communication
                                    ? read_register_forced(kCompactReportRegister, data, sizeof(data))
                                    : read_register(kCompactReportRegister, data, sizeof(data));
    if (!read_succeeded) {
        timing_.last_compact_read_us = time_us_32() - started_us;
        return false;
    }
    timing_.last_compact_read_us = time_us_32() - started_us;

    sample.active = data[5] != 0;
    sample.finger_count = data[5];
    sample.relative_x = read_big_endian_i16(&data[6]);
    sample.relative_y = read_big_endian_i16(&data[8]);
    sample.movement_reported = (data[4] & 0x01) != 0;
    sample.single_tap = (data[1] & 0x01) != 0;
    sample.two_finger_tap = (data[2] & 0x01) != 0;
    sample.scroll_gesture = (data[2] & 0x02) != 0;
    return true;
}

bool Tps43Iqs5xxDriver::read_contact_report(Tps43Sample& sample, bool force_communication) {
    uint8_t data[kContactReportBytes] = {};
    const uint32_t started_us = time_us_32();
    const bool read_succeeded = force_communication
                                    ? read_register_forced(kContactReportRegister, data, sizeof(data))
                                    : read_register(kContactReportRegister, data, sizeof(data));
    if (!read_succeeded) {
        timing_.last_contact_read_us = time_us_32() - started_us;
        return false;
    }
    timing_.last_contact_read_us = time_us_32() - started_us;

    sample.contact_details_available = true;
    for (uint8_t slot = 0; slot < kContactSlots; ++slot) {
        const uint8_t* record = &data[slot * kContactRecordBytes];
        Tps43Contact& contact = sample.contacts[slot];
        contact.x = read_big_endian_u16(&record[0]);
        contact.y = read_big_endian_u16(&record[2]);
        contact.strength = read_big_endian_u16(&record[4]);
        contact.area = record[6];
        // The physical capture verified three nonzero-area records for the
        // three-finger state. The stability run remains responsible for
        // checking this inactive-slot representation across repeated reads.
        contact.active = contact.area != 0;
    }
    return true;
}
