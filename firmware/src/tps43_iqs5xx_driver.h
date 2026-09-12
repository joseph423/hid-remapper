#ifndef TPS43_IQS5XX_DRIVER_H_
#define TPS43_IQS5XX_DRIVER_H_

#include <cstdint>

#include <hardware/i2c.h>

#include "tps43_driver.h"

struct Tps43Iqs5xxConfig {
    i2c_inst_t* bus;
    uint8_t address;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t rdy_pin;
    uint32_t i2c_frequency_hz;
};

struct Tps43ServiceTiming {
    uint32_t last_service_us = 0;
    uint32_t last_compact_read_us = 0;
    uint32_t last_contact_read_us = 0;
    uint32_t last_diagnostic_contact_read_us = 0;
    uint32_t transfer_failures = 0;
    bool last_sample_had_contact_read = false;
    bool last_sample_published = false;
};

// Reads the verified B000 compact report and the conditional five-slot
// contact report for one physical TPS43 without applying dual-pad policy.
class Tps43Iqs5xxDriver final : public Tps43Driver {
   public:
    // Stores the hardware configuration; initialize() performs GPIO and I²C setup.
    explicit Tps43Iqs5xxDriver(Tps43Iqs5xxConfig config);

    // Configures the bus and RDY input. Returns false for an invalid bus.
    bool initialize();

    // Performs one bounded report acquisition when RDY is high. Returns true
    // only after the complete compact/contact transaction is published.
    bool service(uint64_t now_us) override;

    // Requests one operator-gated acquisition even when Event Mode leaves RDY
    // low. The request is consumed by the next service call.
    void request_forced_read();

    // Reads the detailed contact block for bring-up diagnosis without
    // publishing it as the production sample. The caller must use this only
    // for an explicitly requested diagnostic mismatch.
    bool read_contact_report_for_diagnostic(Tps43Sample& sample);

    // Returns the latest complete acquisition without changing it.
    Tps43Sample sample() const override;

    // Returns timing and failure counters for the physical timing handoff.
    const Tps43ServiceTiming& timing() const;

   private:
    bool read_register(uint16_t address, uint8_t* data, uint8_t length);
    bool read_register_forced(uint16_t address, uint8_t* data, uint8_t length);
    bool end_communication_window();
    bool read_compact_report(Tps43Sample& sample, bool force_communication);
    bool read_contact_report(Tps43Sample& sample, bool force_communication);

    Tps43Iqs5xxConfig config_;
    Tps43Sample sample_;
    Tps43ServiceTiming timing_;
    bool initialized_ = false;
    bool forced_read_requested_ = false;
};

#endif
