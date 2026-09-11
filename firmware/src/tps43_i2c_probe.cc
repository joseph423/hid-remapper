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
constexpr uint32_t kI2cFrequency = 400000;
constexpr uint16_t kIdentityRegister = 0x0000;
constexpr uint16_t kStartupStatusRegister = 0x000F;
constexpr uint16_t kEndCommunicationRegister = 0xEEEE;

struct Identity {
    uint16_t product_number;
    uint16_t project_number;
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t bootloader_status;
};

bool read_register(uint16_t address, uint8_t* data, size_t length) {
    const uint8_t register_address[] = {
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xff),
    };
    // The sensor's random-read transaction selects a 16-bit register and
    // keeps the bus active for the repeated-start read.
    const int write_result = i2c_write_blocking(kI2c, kI2cAddress, register_address, sizeof(register_address), true);
    if (write_result != static_cast<int>(sizeof(register_address))) {
        return false;
    }

    return i2c_read_blocking(kI2c, kI2cAddress, data, length, false) == static_cast<int>(length);
}

void end_communication_window() {
    const uint8_t end_byte = 1;
    const uint8_t register_address_and_data[] = {
        static_cast<uint8_t>(kEndCommunicationRegister >> 8),
        static_cast<uint8_t>(kEndCommunicationRegister & 0xff),
        end_byte,
    };
    // A STOP alone does not close the B000 communication window.
    i2c_write_blocking(kI2c, kI2cAddress, register_address_and_data, sizeof(register_address_and_data), false);
}

bool read_identity_and_startup_status(Identity& identity, uint8_t& startup_status) {
    uint8_t data[7] = {};
    if (!read_register(kIdentityRegister, data, sizeof(data))) {
        return false;
    }

    // Read startup status in the same communication window as identity so the
    // check does not add a configuration write or a separate reset sequence.
    if (!read_register(kStartupStatusRegister, &startup_status, sizeof(startup_status))) {
        end_communication_window();
        return false;
    }

    identity.product_number = static_cast<uint16_t>(data[0] << 8) | data[1];
    identity.project_number = static_cast<uint16_t>(data[2] << 8) | data[3];
    identity.major_version = data[4];
    identity.minor_version = data[5];
    identity.bootloader_status = data[6];
    end_communication_window();
    return true;
}

void configure_i2c() {
    i2c_init(kI2c, kI2cFrequency);
    gpio_set_function(kSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(kSclPin, GPIO_FUNC_I2C);

    // Leave pull-up selection to the verified sensor breakout and wiring.
    gpio_set_pulls(kSdaPin, false, false);
    gpio_set_pulls(kSclPin, false, false);
}

}  // namespace

int main() {
    stdio_init_all();
    configure_i2c();
    sleep_ms(1500);

    printf("TPS43 I2C probe\n");
    printf("I2C0 SDA=GP%u SCL=GP%u frequency=%lu address=0x%02x\n", kSdaPin, kSclPin, kI2cFrequency, kI2cAddress);

    for (int attempt = 1; attempt <= 5; ++attempt) {
        Identity identity{};
        uint8_t startup_status = 0;
        if (read_identity_and_startup_status(identity, startup_status)) {
            printf("response=ok attempt=%d product=%u project=%u version=%u.%u bootloader=0x%02x startup_status_0x000f=0x%02x\n", attempt, identity.product_number, identity.project_number, identity.major_version, identity.minor_version, identity.bootloader_status, startup_status);
            printf("RESULT: TPS43 responded over I2C\n");
            printf("RESULT: TPS43 startup status read\n");
            while (true) {
                tight_loop_contents();
            }
        }

        printf("response=failed attempt=%d\n", attempt);
        sleep_us(200);
    }

    printf("RESULT: no response from the expected TPS43 address\n");
    while (true) {
        tight_loop_contents();
    }
}
