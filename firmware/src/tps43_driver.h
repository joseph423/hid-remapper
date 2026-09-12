#ifndef TPS43_DRIVER_H_
#define TPS43_DRIVER_H_

#include <array>
#include <cstdint>

constexpr uint8_t TPS43_MAX_CONTACTS = 5;

struct Tps43Contact {
    // Driver-decoded slot validity. Raw IQS572 slot encoding stays in the
    // eventual physical driver.
    bool active = false;
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t strength = 0;
    uint8_t area = 0;
};

struct Tps43Sample {
    bool active = false;
    uint8_t finger_count = 0;

    // Signed relative movement from the compact report. These are not
    // absolute contact coordinates.
    int32_t relative_x = 0;
    int32_t relative_y = 0;

    // Sensor-reported status/events, before any FSM timing or distance policy.
    bool movement_reported = false;
    bool single_tap = false;
    bool two_finger_tap = false;
    bool scroll_gesture = false;
    // The contact array is optional. False means consumers must use only the
    // compact fields above; it does not make the compact sample invalid.
    bool contact_details_available = false;
    std::array<Tps43Contact, TPS43_MAX_CONTACTS> contacts{};
    // Acquisition time in microseconds on the coordinator clock, preserved
    // while this result is retained between service calls.
    uint64_t timestamp_us = 0;
};

class Tps43Driver {
   public:
    virtual ~Tps43Driver() = default;

    // Perform one bounded, non-blocking hardware step at now_us (microseconds).
    // sample() then
    // returns the latest coherent result produced by the driver. Return true
    // only when this call publishes a new complete acquisition, even if its
    // values or timestamp equal the preceding acquisition. False retains the
    // prior sample (including before the first acquisition); it is not a
    // stationary report or a synthetic release.
    virtual bool service(uint64_t now_us) = 0;
    // Returns the latest complete acquisition without consuming or changing it.
    virtual Tps43Sample sample() const = 0;
};

// Supplies an intentionally inactive pad to the coordinator while one-sensor
// physical bring-up is in progress.
class Tps43InactiveDriver final : public Tps43Driver {
   public:
    // Reports that no new acquisition exists and that the pad is inactive.
    bool service(uint64_t now_us) override {
        (void) now_us;
        return false;
    }

    // Returns the unchanged inactive sample.
    Tps43Sample sample() const override {
        return {};
    }
};

#endif
