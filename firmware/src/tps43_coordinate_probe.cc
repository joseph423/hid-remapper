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
constexpr uint16_t kAbsoluteCoordinateRegister = 0x0017;
constexpr size_t kAbsoluteCoordinateBytes = 4;
constexpr uint16_t kEndCommunicationRegister = 0xEEEE;

struct Coordinates {
    uint16_t absolute_x;
    uint16_t absolute_y;
};

bool read_register(uint16_t address, uint8_t* data, size_t length);
bool end_communication_window();
bool read_absolute_coordinates(Coordinates& coordinates);
void configure_i2c();
void configure_rdy_input();
void wait_for_rdy(const char* action);
[[noreturn]] void fail(const char* reason);

}  // namespace

/**
 * Reads one absolute X/Y coordinate sample from the temporary Right-pad
 * wiring without changing sensor configuration or invoking HID output.
 */
int main() {
    stdio_init_all();
    configure_rdy_input();
    configure_i2c();
    sleep_ms(1500);

    printf("TPS43 coordinate probe\n");
    printf("I2C0 SDA=GP%u SCL=GP%u frequency=%lu address=0x%02x\n", kSdaPin, kSclPin, kI2cFrequency,
        kI2cAddress);
    printf("RDY=GP%u active=high\n", kRdyPin);

    wait_for_rdy("place and hold one finger on the TPS43 to request the coordinate communication window");

    Coordinates coordinates{};
    if (!read_absolute_coordinates(coordinates)) {
        fail("RESULT: absolute coordinate read failed");
    }

    printf("coordinate_response=ok absolute_x=%u absolute_y=%u\n", coordinates.absolute_x,
        coordinates.absolute_y);
    printf("RESULT: TPS43 absolute coordinate read completed\n");
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

bool end_communication_window() {
    const uint8_t command[] = {
        static_cast<uint8_t>(kEndCommunicationRegister >> 8),
        static_cast<uint8_t>(kEndCommunicationRegister & 0xff),
        1,
    };
    return i2c_write_blocking(kI2c, kI2cAddress, command, sizeof(command), false) ==
           static_cast<int>(sizeof(command));
}

bool read_absolute_coordinates(Coordinates& coordinates) {
    uint8_t data[kAbsoluteCoordinateBytes] = {};
    if (!read_register(kAbsoluteCoordinateRegister, data, sizeof(data))) {
        end_communication_window();
        return false;
    }

    coordinates.absolute_x = static_cast<uint16_t>(data[0] << 8) | data[1];
    coordinates.absolute_y = static_cast<uint16_t>(data[2] << 8) | data[3];
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

void configure_rdy_input() {
    gpio_init(kRdyPin);
    gpio_set_dir(kRdyPin, GPIO_IN);
    gpio_set_pulls(kRdyPin, false, false);
}

void wait_for_rdy(const char* action) {
    if (!gpio_get(kRdyPin)) {
        printf("ACTION: %s\n", action);
    }
    while (!gpio_get(kRdyPin)) {
        tight_loop_contents();
    }
}

[[noreturn]] void fail(const char* reason) {
    printf("%s\n", reason);
    while (true) {
        tight_loop_contents();
    }
}

}  // namespace
