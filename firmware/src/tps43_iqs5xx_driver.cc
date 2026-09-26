#include "tps43_iqs5xx_driver.h"

#include <hardware/gpio.h>
#include <hardware/resets.h>
#include <pico/time.h>
#include <algorithm>
#include <cstdio>

namespace {

constexpr uint16_t kCompactReportRegister = 0x000C;
constexpr uint16_t kContactReportRegister = 0x0016;
constexpr uint16_t kEndCommunicationRegister = 0xEEEE;
constexpr uint16_t kActiveReportIntervalRegister = 0x057A;
constexpr uint16_t kIdleTimeoutRegister = 0x0586;
constexpr uint16_t kLp1TimeoutRegister = 0x0587;
// Delay configuration until after the sensor's startup debounce interval.
constexpr uint64_t kPowerTimeoutApplyStartupDelayUs = 600000;
constexpr uint64_t kPowerTimeoutApplyDeadlineUs = 250000;
// A fault deadline, not a report-rate/tuning target. Historical forced reads
// reached 13 ms. All waits below yield to USB; one acquisition gets 20 ms total.
constexpr uint64_t kAcquisitionDeadlineUs = 20000;
constexpr uint64_t kRecoveryDeadlineUs = 2000;
constexpr uint64_t kRetryCooldownUs = 1000;
constexpr uint8_t kFifoDepth = 16;

uint16_t read_big_endian_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0] << 8) | data[1];
}

}  // namespace

Tps43Iqs5xxDriver::Tps43Iqs5xxDriver(Tps43Iqs5xxConfig config)
    : config_(config) {
}

bool Tps43Iqs5xxDriver::initialize() {
    if (config_.bus != i2c0 && config_.bus != i2c1) {
        return false;
    }
    // SDK initialization is startup-only. Recovery never calls its blocking
    // reset helper and never toggles the sensor's reset or GPIO bus lines.
    i2c_init(config_.bus, config_.i2c_frequency_hz);
    auto* hw = config_.bus->hw;
    con_ = hw->con;
    hcnt_ = hw->fs_scl_hcnt;
    lcnt_ = hw->fs_scl_lcnt;
    hold_ = hw->sda_hold;
    spklen_ = hw->fs_spklen;
    hw->enable = 0;
    hw->tar = config_.address;
    hw->enable = 1;
    gpio_set_function(config_.sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config_.scl_pin, GPIO_FUNC_I2C);
    gpio_set_pulls(config_.sda_pin, false, false);
    gpio_set_pulls(config_.scl_pin, false, false);
    gpio_init(config_.rdy_pin);
    gpio_set_dir(config_.rdy_pin, GPIO_IN);
    gpio_set_pulls(config_.rdy_pin, false, false);
    initialized_ = true;
    return true;
}

bool Tps43Iqs5xxDriver::request_power_mode_timeouts(
    uint8_t idle_timeout_seconds, uint8_t lp1_timeout_20s_units) {
    if (!initialized_ || idle_timeout_seconds == 0 || idle_timeout_seconds == 255 ||
        lp1_timeout_20s_units == 0 || lp1_timeout_20s_units == 255) {
        return false;
    }
    idle_timeout_requested_seconds_ = idle_timeout_seconds;
    lp1_timeout_requested_20s_units_ = lp1_timeout_20s_units;
    low_power_timeout_configuration_pending_ = true;
    low_power_timeout_apply_after_us_ = time_us_64() + kPowerTimeoutApplyStartupDelayUs;
    return true;
}

bool Tps43Iqs5xxDriver::service(uint64_t now_us) {
    (void) now_us;
    const uint32_t started_us = time_us_32();
    timing_.last_sample_published = sample_ready_;
    if (sample_ready_) {
        sample_ready_ = false;
        timing_.last_service_us = time_us_32() - started_us;
        return true;
    }
    timing_.last_service_us = time_us_32() - started_us;
    return false;
}

void Tps43Iqs5xxDriver::poll() {
    const uint32_t started_us = time_us_32();
    // The lambda gives every early exit the same bounded-work instrumentation.
    const auto advance = [&]() {
        if (!initialized_) {
            return;
        }
        const uint64_t now = time_us_64();
        if (stage_ == Stage::Idle && low_power_timeout_configuration_pending_ &&
            now >= low_power_timeout_apply_after_us_) {
            forced_active_ = true;
            wake_retried_ = false;
            deadline_us_ = now + kPowerTimeoutApplyDeadlineUs;
            start_transfer(kCompactReportRegister, 10, Stage::Compact);
            return;
        }
        if (stage_ == Stage::Idle && !sample_ready_
            && !low_power_timeout_configuration_pending_
            && now >= retry_after_us_ &&
            (active_rate_request_pending_ ? gpio_get(config_.rdy_pin)
                                          : (forced_read_requested_ || gpio_get(config_.rdy_pin)))) {
            if (active_rate_request_pending_) {
                const uint8_t interval_bytes[] = {
                    static_cast<uint8_t>(active_rate_request_ms_ >> 8),
                    static_cast<uint8_t>(active_rate_request_ms_ & 0xff),
                };
                active_rate_in_progress_ms_ = active_rate_request_ms_;
                active_rate_request_pending_ = false;
                deadline_us_ = now + kAcquisitionDeadlineUs;
                start_write_transfer(kActiveReportIntervalRegister, interval_bytes, 2, Stage::RateWrite);
                return;
            }
            forced_active_ = forced_read_requested_;
            diagnostic_active_ = diagnostic_requested_;
            forced_read_requested_ = diagnostic_requested_ = false;
            wake_retried_ = recovery_close_attempted_ = false;
            next_sample_ = {};
            timing_.last_sample_had_contact_read = false;
            timing_.last_compact_read_us = timing_.last_contact_read_us = 0;
            timing_.last_diagnostic_contact_read_us = 0;
            acquisition_started_us_ = time_us_64();
            deadline_us_ = acquisition_started_us_ + kAcquisitionDeadlineUs;
            start_transfer(kCompactReportRegister, 10, Stage::Compact);
            return;
        }

        if (stage_ == Stage::Idle) {
            return;
        }
        if (stage_ == Stage::Reset) {
            const uint32_t mask = 1u << (config_.bus == i2c0 ? RESET_I2C0 : RESET_I2C1);
            if (!(resets_hw->reset_done & mask) || now < retry_after_us_) {
                return;
            }
            restore_controller();
            if (recovery_close_attempted_) {
                stage_ = Stage::Idle;
                return;
            }
            recovery_close_attempted_ = true;
            deadline_us_ = now + kRecoveryDeadlineUs;
            start_transfer(kEndCommunicationRegister, 0, Stage::RecoverClose);
            return;
        }
        if (now >= deadline_us_) {
            fail_acquisition(true);
            return;
        }
        if (stage_ == Stage::Wake) {
            if (now >= retry_after_us_) {
                start_transfer(kCompactReportRegister, 10, Stage::Compact);
            }
            return;
        }
        const TransferResult result = poll_transfer();
        if (result == TransferResult::Pending) {
            return;
        }
        if (result == TransferResult::Failed) {
            // Only the documented forced-wake address NACK gets one retry.
            // The deadline is shared with the first attempt, not restarted.
            // TX_FLUSH_CNT can add upper bits to this register; match the
            // address-NACK cause bit instead of comparing the whole value.
            const bool wake_nack = (config_.bus->hw->tx_abrt_source &
                                    I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS) != 0;
            if (stage_ == Stage::Compact && forced_active_ && !wake_retried_ && wake_nack) {
#ifndef TPS43_QUIET_PRODUCTION
                if (low_power_timeout_configuration_pending_) {
                    printf("TPS43 pad=%s power_timeout_retry_scheduled_after_nack=150us tx_abrt=0x%08lx\n",
                        config_.bus == i2c0 ? "right" : "left",
                        static_cast<unsigned long>(config_.bus->hw->tx_abrt_source));
                }
#endif
                (void) static_cast<uint32_t>(config_.bus->hw->clr_tx_abrt);
                (void) static_cast<uint32_t>(config_.bus->hw->clr_stop_det);
                wake_retried_ = true;
                retry_after_us_ = now + 150;
                stage_ = Stage::Wake;
                return;
            }
            fail_acquisition(false);
            return;
        }
        const uint32_t elapsed = static_cast<uint32_t>(now - transaction_started_us_);
        switch (stage_) {
            case Stage::IdleTimeoutWrite:
            {
                const uint8_t timeout = lp1_timeout_requested_20s_units_;
                start_write_transfer(kLp1TimeoutRegister, &timeout, 1, Stage::Lp1TimeoutWrite);
                break;
            }
            case Stage::Lp1TimeoutRead:
                start_transfer(kEndCommunicationRegister, 0, Stage::PowerTimeoutReadClose);
                break;
            case Stage::PowerTimeoutReadClose:
                // The end-window write does not touch the existing RX buffer.
#ifndef TPS43_QUIET_PRODUCTION
                printf(
                    "TPS43 pad=%s idle_timeout_register=0x%04x requested_s=%u readback_s=%u "
                    "lp1_timeout_register=0x%04x requested_20s_units=%u readback_20s_units=%u "
                    "sensor_persistence=volatile result=%s\n",
                    config_.bus == i2c0 ? "right" : "left", kIdleTimeoutRegister,
                    idle_timeout_requested_seconds_, data_[0], kLp1TimeoutRegister,
                    lp1_timeout_requested_20s_units_, data_[1],
                    data_[0] == idle_timeout_requested_seconds_ && data_[1] == lp1_timeout_requested_20s_units_
                        ? "verified" : "mismatch");
#endif
                low_power_timeout_configuration_pending_ = false;
                forced_active_ = false;
                stage_ = Stage::Idle;
                break;
            case Stage::Lp1TimeoutWrite:
                start_transfer(kIdleTimeoutRegister, 2, Stage::Lp1TimeoutRead);
                break;
            case Stage::Compact:
                timing_.last_compact_read_us = elapsed;
                decode_compact();
                if (low_power_timeout_configuration_pending_) {
                    const uint8_t timeout = idle_timeout_requested_seconds_;
                    start_write_transfer(kIdleTimeoutRegister, &timeout, 1, Stage::IdleTimeoutWrite);
                    break;
                }
                if (next_sample_.finger_count == 3 || diagnostic_active_) {
                    start_transfer(kContactReportRegister, 35, Stage::Contact);
                } else {
                    start_transfer(kEndCommunicationRegister, 0, Stage::Close);
                }
                break;
            case Stage::Contact:
                timing_.last_contact_read_us = elapsed;
                timing_.last_sample_had_contact_read = true;
                if (next_sample_.finger_count != 3) {
                    timing_.last_diagnostic_contact_read_us = elapsed;
                }
                decode_contacts();
                start_transfer(kEndCommunicationRegister, 0, Stage::Close);
                break;
            case Stage::Close:
                next_sample_.timestamp_us = now;
                sample_ = next_sample_;
                sample_ready_ = true;
                timing_.last_acquisition_us = static_cast<uint32_t>(now - acquisition_started_us_);
                stage_ = Stage::Idle;
                break;
            case Stage::RateWrite:
                start_transfer(kEndCommunicationRegister, 0, Stage::RateClose);
                break;
            case Stage::RateClose:
                timing_.active_report_interval_ms = active_rate_in_progress_ms_;
#ifndef TPS43_QUIET_PRODUCTION
                printf("TPS43 pad=%s active_report_interval_ms=%u result=applied persistence=volatile\n",
                    config_.bus == i2c0 ? "right" : "left",
                    active_rate_in_progress_ms_);
#endif
                stage_ = Stage::Idle;
                break;
            case Stage::RecoverClose:
                stage_ = Stage::Idle;
                retry_after_us_ = now + kRetryCooldownUs;
                break;
            default:
                break;
        }
    };
    advance();
    timing_.max_poll_us = std::max(timing_.max_poll_us, time_us_32() - started_us);
}

void Tps43Iqs5xxDriver::request_forced_read(bool diagnose_contact_mismatch) {
    if (stage_ != Stage::Idle || sample_ready_) {
        return;
    }
    forced_read_requested_ = true;
    diagnostic_requested_ = diagnose_contact_mismatch;
}

bool Tps43Iqs5xxDriver::request_active_report_interval(uint16_t interval_ms) {
    if ((interval_ms != kTps43DefaultActiveReportIntervalMs &&
            interval_ms != kTps43BaselineActiveReportIntervalMs) ||
        active_rate_request_pending_ ||
        stage_ == Stage::RateWrite || stage_ == Stage::RateClose) {
        return false;
    }
    active_rate_request_ms_ = interval_ms;
    active_rate_request_pending_ = true;
    return true;
}

Tps43Sample Tps43Iqs5xxDriver::sample() const {
    return sample_;
}

const Tps43ServiceTiming& Tps43Iqs5xxDriver::timing() const {
    return timing_;
}

void Tps43Iqs5xxDriver::start_transfer(uint16_t address, uint8_t length, Stage stage) {
    register_address_ = address;
    read_length_ = length;
    write_length_ = 0;
    commands_sent_ = bytes_received_ = 0;
    transaction_started_us_ = time_us_64();
    stage_ = stage;
    (void) static_cast<uint32_t>(config_.bus->hw->clr_stop_det);
}

void Tps43Iqs5xxDriver::start_write_transfer(
    uint16_t address,
    const uint8_t* data,
    uint8_t length,
    Stage stage) {
    register_address_ = address;
    read_length_ = 0;
    write_length_ = length;
    for (uint8_t i = 0; i < length; ++i) {
        write_data_[i] = data[i];
    }
    commands_sent_ = bytes_received_ = 0;
    transaction_started_us_ = time_us_64();
    stage_ = stage;
    (void) static_cast<uint32_t>(config_.bus->hw->clr_stop_det);
}

Tps43Iqs5xxDriver::TransferResult Tps43Iqs5xxDriver::poll_transfer() {
    auto* hw = config_.bus->hw;
    const uint32_t status = hw->raw_intr_stat;
    if (status & (I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS | I2C_IC_RAW_INTR_STAT_RX_OVER_BITS)) {
        return TransferResult::Failed;
    }
    // Read only bytes already present. Limit issued reads to FIFO capacity so
    // a delayed next poll cannot overflow RX or stretch SCL due to a full FIFO.
    for (uint8_t n = 0; n < kFifoDepth && bytes_received_ < read_length_ && hw->rxflr; ++n) {
        data_[bytes_received_++] = static_cast<uint8_t>(hw->data_cmd);
    }
    const uint8_t total_commands = read_length_ ? read_length_ + 2 : (write_length_ ? write_length_ + 2 : 3);
    for (uint8_t n = 0; n < kFifoDepth && commands_sent_ < total_commands && hw->txflr < kFifoDepth; ++n) {
        uint32_t command;
        if (commands_sent_ == 0) {
            command = register_address_ >> 8;
        } else if (commands_sent_ == 1) {
            command = register_address_ & 0xff;
        } else if (read_length_) {
            if (commands_sent_ - 2 - bytes_received_ >= kFifoDepth) {
                break;
            }
            command = I2C_IC_DATA_CMD_CMD_BITS;
            if (commands_sent_ == 2) {
                command |= I2C_IC_DATA_CMD_RESTART_BITS;
            }
        } else if (write_length_ && commands_sent_ < write_length_ + 2) {
            command = write_data_[commands_sent_ - 2];
        } else {
            command = 1;  // End-communication payload.
        }
        if (commands_sent_ + 1 == total_commands) {
            command |= I2C_IC_DATA_CMD_STOP_BITS;
        }
        hw->data_cmd = command;
        ++commands_sent_;
    }
    // STOP, not TX FIFO empty, establishes completion of the last bus byte.
    const uint32_t final_status = hw->raw_intr_stat;
    if (final_status & (I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS | I2C_IC_RAW_INTR_STAT_RX_OVER_BITS)) {
        return TransferResult::Failed;
    }
    if (commands_sent_ == total_commands && bytes_received_ == read_length_ &&
        (final_status & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS)) {
        (void) static_cast<uint32_t>(hw->clr_stop_det);
        return TransferResult::Complete;
    }
    return TransferResult::Pending;
}

void Tps43Iqs5xxDriver::fail_acquisition(bool timeout) {
    ++timing_.transfer_failures;
    timing_.transfer_timeouts += timeout;
    const Stage failed_stage = stage_;
    if (low_power_timeout_configuration_pending_) {
#ifndef TPS43_QUIET_PRODUCTION
        const char* phase = failed_stage == Stage::Compact || failed_stage == Stage::Wake ? "wake_read" :
            (failed_stage == Stage::IdleTimeoutWrite ? "write_idle_timeout" :
            (failed_stage == Stage::Lp1TimeoutWrite ? "write_lp1_timeout" :
            (failed_stage == Stage::Lp1TimeoutRead ? "readback_timeouts" :
            (failed_stage == Stage::PowerTimeoutReadClose ? "readback_close" :
            (failed_stage == Stage::RecoverClose ? "recovery_close" : "recovery")))));
        printf(
            "TPS43 pad=%s power_timeouts idle_s=%u lp1_20s_units=%u result=failed failure=%s phase=%s stage=%u tx_abrt=0x%08lx intr=0x%08lx rdy=%u retried=%u sensor_persistence=volatile\n",
            config_.bus == i2c0 ? "right" : "left", idle_timeout_requested_seconds_,
            lp1_timeout_requested_20s_units_,
            timeout ? "timeout" : "transfer", phase, static_cast<unsigned>(failed_stage),
            static_cast<unsigned long>(config_.bus->hw->tx_abrt_source),
            static_cast<unsigned long>(config_.bus->hw->raw_intr_stat), gpio_get(config_.rdy_pin),
            wake_retried_);
#endif
        low_power_timeout_configuration_pending_ = false;
        forced_active_ = false;
    }
    if (failed_stage == Stage::RateWrite || failed_stage == Stage::RateClose) {
#ifndef TPS43_QUIET_PRODUCTION
        printf("TPS43 pad=%s active_report_interval_ms=%u result=%s failure=%s\n",
            config_.bus == i2c0 ? "right" : "left", active_rate_in_progress_ms_,
            failed_stage == Stage::RateClose ? "unknown" : "failed",
            timeout ? "timeout" : "transfer");
#endif
    }
    // Discard the partial report, retain the last published state, and perform
    // at most one bounded cleanup write after peripheral recovery.
    reset_controller();
}

void Tps43Iqs5xxDriver::reset_controller() {
    const uint32_t block = config_.bus == i2c0 ? RESET_I2C0 : RESET_I2C1;
    reset_block_num(block);
    unreset_block_num(block);
    retry_after_us_ = time_us_64() + kRetryCooldownUs;
    stage_ = Stage::Reset;
}

void Tps43Iqs5xxDriver::restore_controller() {
    auto* hw = config_.bus->hw;
    hw->enable = 0;
    hw->con = con_;
    hw->fs_scl_hcnt = hcnt_;
    hw->fs_scl_lcnt = lcnt_;
    hw->sda_hold = hold_;
    hw->fs_spklen = spklen_;
    hw->tx_tl = hw->rx_tl = 0;
    hw->tar = config_.address;
    hw->enable = 1;
}

void Tps43Iqs5xxDriver::decode_compact() {
    next_sample_.previous_cycle_time_ms = data_[0];
    next_sample_.active = data_[5] != 0;
    next_sample_.finger_count = data_[5];
    next_sample_.relative_x = static_cast<int16_t>(read_big_endian_u16(&data_[6]));
    next_sample_.relative_y = static_cast<int16_t>(read_big_endian_u16(&data_[8]));
    next_sample_.movement_reported = (data_[4] & 0x01) != 0;
    next_sample_.report_rate_missed = (data_[4] & 0x08) != 0;
    next_sample_.single_tap = (data_[1] & 0x01) != 0;
    next_sample_.two_finger_tap = (data_[2] & 0x01) != 0;
    next_sample_.scroll_gesture = (data_[2] & 0x02) != 0;
}

void Tps43Iqs5xxDriver::decode_contacts() {
    next_sample_.contact_details_available = true;
    for (uint8_t slot = 0; slot < TPS43_MAX_CONTACTS; ++slot) {
        const uint8_t* record = &data_[slot * 7];
        Tps43Contact& contact = next_sample_.contacts[slot];
        contact.x = read_big_endian_u16(&record[0]);
        contact.y = read_big_endian_u16(&record[2]);
        contact.strength = read_big_endian_u16(&record[4]);
        contact.area = record[6];
        // Preserve the physically observed slot decoder; this change does not
        // select new contact thresholds or filtering.
        contact.active = contact.area != 0;
    }
}
