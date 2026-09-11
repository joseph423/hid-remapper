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
constexpr uint16_t kStartupStatusRegister = 0x000F;
constexpr uint16_t kEndCommunicationRegister = 0xEEEE;

volatile uint32_t rdy_rise_count = 0;
volatile uint32_t rdy_fall_count = 0;
volatile uint32_t last_rdy_rise_us = 0;
volatile uint32_t last_rdy_fall_us = 0;

bool read_register(uint16_t address, uint8_t* data, size_t length);
bool end_communication_window();
void configure_i2c();
void configure_rdy_input();
void rdy_gpio_irq(uint gpio, uint32_t events);
[[noreturn]] void halt();

}  // namespace

/**
 * Observes the TPS43 RDY communication-window sequence on the temporary
 * Right-pad wiring. The diagnostic has no inputs and does not return.
 */
int main() {
    stdio_init_all();
    configure_rdy_input();
    configure_i2c();
    sleep_ms(1500);

    const bool initial_rdy = gpio_get(kRdyPin);
    printf("TPS43 RDY probe\n");
    printf("I2C0 SDA=GP%u SCL=GP%u frequency=%lu address=0x%02x\n", kSdaPin, kSclPin, kI2cFrequency,
        kI2cAddress);
    printf("RDY=GP%u active=high initial=%s\n", kRdyPin, initial_rdy ? "high" : "low");

    if (!initial_rdy) {
        printf("ACTION: touch or move the TPS43 once to request a communication window\n");
        while (!gpio_get(kRdyPin)) {
            tight_loop_contents();
        }
    }

    const bool rdy_before_read = gpio_get(kRdyPin);
    if (!rdy_before_read) {
        printf("RESULT: RDY was not high before the read\n");
        halt();
    }

    uint8_t startup_status = 0;
    const bool read_ok = read_register(kStartupStatusRegister, &startup_status, sizeof(startup_status));
    const bool rdy_before_end = gpio_get(kRdyPin);
    const uint32_t falls_before_end = rdy_fall_count;
    const uint32_t rises_before_end = rdy_rise_count;
    const uint32_t end_command_started_us = time_us_32();
    const bool end_ok = end_communication_window();

    printf("rdy_before_read=%s read=%s startup_status_0x000f=0x%02x rdy_before_end=%s end_communication=%s\n",
        rdy_before_read ? "high" : "low", read_ok ? "ok" : "failed", startup_status,
        rdy_before_end ? "high" : "low", end_ok ? "ok" : "failed");

    if (!read_ok || !rdy_before_end || !end_ok) {
        printf("RESULT: RDY communication-window check failed before edge verification\n");
        halt();
    }

    // The IRQ counters retain short edges that polling after the I2C command
    // could miss. No elapsed-time value is treated as an acceptance threshold.
    while (rdy_fall_count == falls_before_end) {
        tight_loop_contents();
    }
    const uint32_t fall_timestamp_us = last_rdy_fall_us;
    printf("rdy_falling_edge_after_end=observed elapsed_us=%lu\n", fall_timestamp_us - end_command_started_us);

    if (rdy_rise_count == rises_before_end) {
        printf("ACTION: touch or move the TPS43 once to request the next communication window\n");
        while (rdy_rise_count == rises_before_end) {
            tight_loop_contents();
        }
    }

    const uint32_t rise_timestamp_us = last_rdy_rise_us;
    printf("rdy_next_rising_edge=observed low_interval_us=%lu\n", rise_timestamp_us - fall_timestamp_us);
    printf("RESULT: TPS43 RDY followed the communication-window sequence\n");
    halt();
}

namespace {

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
    // RDY is sensor-driven; do not mask the observed level with a Pico pull.
    gpio_set_pulls(kRdyPin, false, false);
    gpio_set_irq_enabled_with_callback(kRdyPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, rdy_gpio_irq);
}

void rdy_gpio_irq(uint gpio, uint32_t events) {
    if (gpio != kRdyPin) {
        return;
    }

    if ((events & GPIO_IRQ_EDGE_RISE) != 0) {
        last_rdy_rise_us = time_us_32();
        ++rdy_rise_count;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0) {
        last_rdy_fall_us = time_us_32();
        ++rdy_fall_count;
    }
}

[[noreturn]] void halt() {
    while (true) {
        tight_loop_contents();
    }
}

}  // namespace
