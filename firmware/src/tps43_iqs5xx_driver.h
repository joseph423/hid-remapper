#ifndef TPS43_IQS5XX_DRIVER_H_
#define TPS43_IQS5XX_DRIVER_H_

#include <cstdint>

#include <hardware/i2c.h>

#include "tps43_driver.h"

constexpr uint16_t kTps43DefaultActiveReportIntervalMs = 8;
constexpr uint16_t kTps43BaselineActiveReportIntervalMs = 13;

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
    uint32_t transfer_timeouts = 0;
    uint32_t max_poll_us = 0;
    uint32_t last_acquisition_us = 0;
    uint16_t active_report_interval_ms = 0;
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

    // Advances at most one transaction stage and 16 FIFO commands; never waits.
    // Call between USB services, independently of the coordinator's 1 ms tick.
    void poll();

    // Consumes one completed acquisition; poll() acquires while RDY is high.
    // Returns true only once per complete sample; now_us is the coordinator time.
    bool service(uint64_t now_us) override;

    // Requests an asynchronous forced acquisition. Optional mismatch details
    // are read in the same communication window, without changing gesture policy.
    void request_forced_read(bool diagnose_contact_mismatch = false);

    // Requests a volatile report-interval change for the diagnostic profile.
    // Only the 8 ms default and the 13 ms diagnostic baseline are accepted.
    bool request_active_report_interval(uint16_t interval_ms);

    // Returns the latest complete acquisition without changing it.
    Tps43Sample sample() const override;

    // Returns timing and failure counters for the physical timing handoff.
    const Tps43ServiceTiming& timing() const;

   private:
    enum class Stage { Idle,
        Compact,
        Contact,
        Close,
        RateWrite,
        RateClose,
        Wake,
        Reset,
        RecoverClose };
    enum class TransferResult { Pending,
        Complete,
        Failed };

    void start_transfer(uint16_t address, uint8_t length, Stage stage);
    void start_write_transfer(uint16_t address, const uint8_t* data, uint8_t length, Stage stage);
    TransferResult poll_transfer();
    void fail_acquisition(bool timeout);
    void reset_controller();
    void restore_controller();
    void decode_compact();
    void decode_contacts();

    Tps43Iqs5xxConfig config_;
    Tps43Sample sample_;
    Tps43ServiceTiming timing_;
    bool initialized_ = false;
    bool forced_read_requested_ = false;
    bool diagnostic_requested_ = false;
    bool diagnostic_active_ = false;
    bool sample_ready_ = false;
    bool forced_active_ = false;
    bool wake_retried_ = false;
    bool recovery_close_attempted_ = false;
    bool active_rate_request_pending_ = false;
    uint16_t active_rate_request_ms_ = 0;
    uint16_t active_rate_in_progress_ms_ = 0;
    Stage stage_ = Stage::Idle;
    Tps43Sample next_sample_;
    uint8_t data_[35] = {};
    uint16_t register_address_ = 0;
    uint8_t read_length_ = 0;
    uint8_t write_data_[2] = {};
    uint8_t write_length_ = 0;
    uint8_t commands_sent_ = 0;
    uint8_t bytes_received_ = 0;
    uint64_t acquisition_started_us_ = 0;
    uint64_t deadline_us_ = 0;
    uint64_t transaction_started_us_ = 0;
    uint64_t retry_after_us_ = 0;
    // Preserve SDK-configured timing across peripheral-only fault recovery.
    uint32_t con_ = 0, hcnt_ = 0, lcnt_ = 0, hold_ = 0, spklen_ = 0;
};

#endif
