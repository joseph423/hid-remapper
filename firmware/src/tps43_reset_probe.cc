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
constexpr uint8_t kResetPin = 10;
constexpr uint32_t kI2cFrequency = 400000;
constexpr uint16_t kIdentityRegister = 0x0000;
constexpr uint16_t kSystemInfo0Register = 0x000F;
constexpr uint16_t kSystemControl0Register = 0x0431;
constexpr uint16_t kEndCommunicationRegister = 0xEEEE;
constexpr uint8_t kAckReset = 0x80;
constexpr uint8_t kShowReset = 0x80;

struct Identity {
    uint16_t product_number;
    uint16_t project_number;
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t bootloader_status;
};

volatile uint32_t reset_rise_count = 0;
volatile uint32_t reset_fall_count = 0;
volatile uint32_t last_reset_rise_us = 0;
volatile uint32_t last_reset_fall_us = 0;

bool read_register(uint16_t address, uint8_t* data, size_t length);
bool write_register(uint16_t address, uint8_t data);
bool end_communication_window();
bool read_identity_and_status(Identity& identity, uint8_t& system_info0);
bool acknowledge_reset();
void configure_i2c();
void configure_inputs();
void reset_gpio_irq(uint gpio, uint32_t events);
void wait_for_rdy(const char* action);
void wait_for_next_rdy(const char* action);
[[noreturn]] void fail(const char* reason);

}  // namespace

/**
 * Observes a manually asserted hardware reset on the temporary Right-pad
 * wiring and checks the sensor's documented reset indication.
 */
int main() {
    stdio_init_all();
    configure_inputs();
    configure_i2c();
    sleep_ms(1500);

    const bool initial_reset_high = gpio_get(kResetPin);
    printf("TPS43 hardware reset probe\n");
    printf("I2C0 SDA=GP%u SCL=GP%u frequency=%lu address=0x%02x\n", kSdaPin, kSclPin, kI2cFrequency,
        kI2cAddress);
    printf("NRST=GP%u active=low initial=%s\n", kResetPin, initial_reset_high ? "high" : "low");
    printf("RDY=GP%u active=high\n", kRdyPin);

    if (!initial_reset_high) {
        printf("ACTION: release the temporary RST-to-GND connection\n");
        while (!gpio_get(kResetPin)) {
            tight_loop_contents();
        }
    }

    wait_for_rdy("touch or move the TPS43 once to request the pre-reset communication window");
    Identity before_reset{};
    uint8_t before_status = 0;
    if (!read_identity_and_status(before_reset, before_status)) {
        fail("RESULT: pre-reset identity/status read failed");
    }

    printf(
        "pre_reset_response=ok product=%u project=%u version=%u.%u bootloader=0x%02x "
        "system_info0=0x%02x show_reset=%s\n",
        before_reset.product_number, before_reset.project_number, before_reset.major_version,
        before_reset.minor_version, before_reset.bootloader_status, before_status,
        (before_status & kShowReset) != 0 ? "set" : "clear");

    // Power-up itself can set SHOW_RESET. Clear only that documented status
    // indication so the later hardware-reset observation has a known baseline.
    if ((before_status & kShowReset) != 0) {
        wait_for_next_rdy("touch or move the TPS43 once to request the ACK_RESET communication window");
        if (!acknowledge_reset()) {
            fail("RESULT: could not clear the pre-reset reset indication");
        }
        wait_for_next_rdy("touch or move the TPS43 once to request the baseline verification window");
        if (!read_identity_and_status(before_reset, before_status) || (before_status & kShowReset) != 0) {
            fail("RESULT: pre-reset reset indication did not clear");
        }
        printf("pre_reset_show_reset=clear\n");
    }

    const uint32_t falls_before_reset = reset_fall_count;
    const uint32_t rises_before_reset = reset_rise_count;
    printf("ACTION: briefly connect TPS43 RST to GND, then remove the connection\n");
    while (reset_fall_count == falls_before_reset) {
        tight_loop_contents();
    }
    const uint32_t reset_asserted_us = last_reset_fall_us;
    printf("nrst_asserted=low\n");

    while (reset_rise_count == rises_before_reset) {
        tight_loop_contents();
    }
    const uint32_t reset_released_us = last_reset_rise_us;
    printf("nrst_released=high observed_low_interval_us=%lu\n", reset_released_us - reset_asserted_us);

    wait_for_rdy("touch or move the TPS43 once to request the post-reset communication window");
    Identity after_reset{};
    uint8_t after_status = 0;
    if (!read_identity_and_status(after_reset, after_status)) {
        fail("RESULT: post-reset identity/status read failed");
    }

    printf(
        "post_reset_response=ok product=%u project=%u version=%u.%u bootloader=0x%02x "
        "system_info0=0x%02x show_reset=%s\n",
        after_reset.product_number, after_reset.project_number, after_reset.major_version,
        after_reset.minor_version, after_reset.bootloader_status, after_status,
        (after_status & kShowReset) != 0 ? "set" : "clear");

    if ((after_status & kShowReset) == 0) {
        fail("RESULT: hardware reset was not indicated by SHOW_RESET");
    }

    printf("RESULT: TPS43 hardware reset was observed\n");
    while (true) {
        tight_loop_contents();
    }
}

namespace {

bool read_register(uint16_t address, uint8_t* data, size_t length) {
    const uint8_t register_address[] = {
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xff),
    };
    const int write_result =
        i2c_write_blocking(kI2c, kI2cAddress, register_address, sizeof(register_address), true);
    if (write_result != static_cast<int>(sizeof(register_address))) {
        return false;
    }

    return i2c_read_blocking(kI2c, kI2cAddress, data, length, false) == static_cast<int>(length);
}

bool write_register(uint16_t address, uint8_t data) {
    const uint8_t register_address_and_data[] = {
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xff),
        data,
    };
    return i2c_write_blocking(kI2c, kI2cAddress, register_address_and_data,
               sizeof(register_address_and_data), false) ==
           static_cast<int>(sizeof(register_address_and_data));
}

bool end_communication_window() {
    return write_register(kEndCommunicationRegister, 1);
}

bool read_identity_and_status(Identity& identity, uint8_t& system_info0) {
    uint8_t data[7] = {};
    if (!read_register(kIdentityRegister, data, sizeof(data))) {
        end_communication_window();
        return false;
    }
    if (!read_register(kSystemInfo0Register, &system_info0, sizeof(system_info0))) {
        end_communication_window();
        return false;
    }

    identity.product_number = static_cast<uint16_t>(data[0] << 8) | data[1];
    identity.project_number = static_cast<uint16_t>(data[2] << 8) | data[3];
    identity.major_version = data[4];
    identity.minor_version = data[5];
    identity.bootloader_status = data[6];
    return end_communication_window();
}

bool acknowledge_reset() {
    uint8_t system_control0 = 0;
    if (!read_register(kSystemControl0Register, &system_control0, sizeof(system_control0))) {
        end_communication_window();
        return false;
    }

    const uint8_t acknowledged_system_control0 = system_control0 | kAckReset;
    printf("system_control0_before_ack=0x%02x acknowledged=0x%02x\n", system_control0,
        acknowledged_system_control0);
    if (!write_register(kSystemControl0Register, acknowledged_system_control0)) {
        end_communication_window();
        return false;
    }
    return end_communication_window();
}

void configure_i2c() {
    i2c_init(kI2c, kI2cFrequency);
    gpio_set_function(kSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(kSclPin, GPIO_FUNC_I2C);

    // Leave pull-up selection to the verified sensor breakout and wiring.
    gpio_set_pulls(kSdaPin, false, false);
    gpio_set_pulls(kSclPin, false, false);
}

void configure_inputs() {
    gpio_init(kRdyPin);
    gpio_set_dir(kRdyPin, GPIO_IN);
    gpio_set_pulls(kRdyPin, false, false);

    gpio_init(kResetPin);
    gpio_set_dir(kResetPin, GPIO_IN);
    // NRST has a sensor-side pull-up. The probe observes a temporary external
    // ground connection and deliberately does not select a Pico pull.
    gpio_set_pulls(kResetPin, false, false);
    gpio_set_irq_enabled_with_callback(kResetPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, reset_gpio_irq);
}

void reset_gpio_irq(uint gpio, uint32_t events) {
    if (gpio != kResetPin) {
        return;
    }

    if ((events & GPIO_IRQ_EDGE_RISE) != 0) {
        last_reset_rise_us = time_us_32();
        ++reset_rise_count;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0) {
        last_reset_fall_us = time_us_32();
        ++reset_fall_count;
    }
}

void wait_for_rdy(const char* action) {
    if (!gpio_get(kRdyPin)) {
        printf("ACTION: %s\n", action);
    }
    while (!gpio_get(kRdyPin)) {
        tight_loop_contents();
    }
}

void wait_for_next_rdy(const char* action) {
    while (gpio_get(kRdyPin)) {
        tight_loop_contents();
    }
    wait_for_rdy(action);
}

[[noreturn]] void fail(const char* reason) {
    printf("%s\n", reason);
    while (true) {
        tight_loop_contents();
    }
}

}  // namespace
