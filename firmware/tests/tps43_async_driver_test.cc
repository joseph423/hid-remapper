#include <hardware/resets.h>
#include "tps43_iqs5xx_driver.h"

#include <array>
#include <cassert>
#include <cstdio>
#include <vector>

uint64_t fake_now = 100;
bool fake_rdy = false;
i2c_hw_t hw0, hw1;
i2c_inst_t bus0{ &hw0 }, bus1{ &hw1 };
i2c_inst_t *i2c0 = &bus0, *i2c1 = &bus1;
FakeResets resets;
FakeResets* resets_hw = &resets;

namespace {
std::array<uint8_t, 10> compact;
std::array<uint8_t, 35> contacts;
uint16_t address = 0;
unsigned address_bytes = 0, offset = 0, closes = 0, resets_seen = 0;
std::vector<std::vector<uint8_t>> report_interval_writes;
std::vector<uint8_t> pending_write;
bool nack_once = false;

Tps43Iqs5xxDriver make_driver();
void step_bus();
void tick(Tps43Iqs5xxDriver& driver, bool advance_bus = true);
bool acquire(Tps43Iqs5xxDriver& driver);

void test_compact_and_no_replay() {
    auto driver = make_driver();
    for (int i = 0; i < 100; ++i)
        tick(driver);
    assert(hw0.written.empty());
    fake_rdy = true;
    assert(acquire(driver));
    const auto sample = driver.sample();
    assert(sample.finger_count == 1 && sample.relative_x == -2 && sample.relative_y == 3);
    assert(sample.movement_reported && sample.single_tap);
    assert(!sample.contact_details_available && closes == 1);
    assert(!driver.service(fake_now));
    fake_rdy = false;
    for (int i = 0; i < 100; ++i)
        tick(driver);
    assert(!driver.service(fake_now));
    assert(driver.sample().timestamp_us == sample.timestamp_us);
}

void test_contact_and_close_gate() {
    auto driver = make_driver();
    compact[5] = 3;
    contacts[0] = 1;
    contacts[1] = 2;
    contacts[6] = 4;
    fake_rdy = true;
    // Complete data reads but stop the bus before the end-communication write.
    for (int i = 0; i < 200 && address != 0xEEEE; ++i) {
        tick(driver);
        assert(!driver.service(fake_now));
    }
    assert(address == 0xEEEE);
    assert(!driver.sample().active);
    assert(acquire(driver));
    assert(driver.sample().contacts[0].x == 258);
    assert(driver.sample().contacts[0].active);
    assert(driver.sample().contact_details_available);
    assert(closes == 1);
}

void test_timeout_recovery_and_retained_sample() {
    auto driver = make_driver();
    fake_rdy = true;
    assert(acquire(driver));
    const auto timestamp = driver.sample().timestamp_us;
    tick(driver, false);  // Begin the next acquisition.
    fake_now += 20001;
    driver.poll();
    assert(driver.timing().transfer_timeouts == 1);
    assert(driver.timing().transfer_failures == 1 && resets_seen == 1);
    assert(!driver.service(fake_now));
    assert(driver.sample().timestamp_us == timestamp);
    fake_now += 1001;
    assert(acquire(driver));
    assert(closes == 3);  // First acquisition, cleanup, successful retry.
    assert(driver.sample().timestamp_us > timestamp);
}

void test_stuck_cleanup_is_bounded() {
    auto driver = make_driver();
    fake_rdy = true;
    tick(driver, false);
    fake_now += 20001;
    driver.poll();
    fake_now += 1001;
    driver.poll();  // RecoverClose starts.
    fake_now += 2001;
    driver.poll();  // Cleanup times out, reset once more.
    assert(driver.timing().transfer_timeouts == 2);
    assert(resets_seen == 2);
    fake_now += 1001;
    driver.poll();  // Restore and stop cleanup attempts.
    assert(driver.timing().transfer_failures == 2);
    assert(acquire(driver));
}

void test_forced_wake_and_fault_rejection() {
    auto driver = make_driver();
    driver.request_forced_read();
    nack_once = true;
    tick(driver);
    tick(driver);
    const size_t written = hw0.written.size();
    for (int i = 0; i < 4; ++i)
        tick(driver);
    assert(hw0.written.size() == written);  // Still within 150 us wake delay.
    assert(acquire(driver));
    assert(driver.timing().transfer_failures == 0);
    assert(closes == 1);
    // An ordinary address NACK must fail, not silently retry as a forced wake.
    fake_rdy = true;
    nack_once = true;
    for (int i = 0; i < 10 && driver.timing().transfer_failures == 0; ++i)
        tick(driver);
    assert(driver.timing().transfer_failures == 1);
}

void test_diagnostic_same_window_and_clock_rollover() {
    auto driver = make_driver();
    fake_now = UINT32_MAX - 50ULL;
    driver.request_forced_read(true);
    contacts[6] = 1;
    assert(acquire(driver));
    assert(driver.sample().finger_count == 1 && driver.sample().contact_details_available);
    assert(driver.sample().timestamp_us > UINT32_MAX);
    assert(closes == 1 && driver.timing().last_diagnostic_contact_read_us > 0);
}

void test_delayed_fifo_poll_and_reset_not_ready() {
    auto driver = make_driver();
    compact[5] = 3;
    fake_rdy = true;
    bool published = false;
    for (int i = 0; i < 30 && !published; ++i) {
        fake_now += 200;
        driver.poll();
        // Let hardware drain all queued commands while firmware services USB.
        while (!hw0.tx.empty())
            step_bus();
        assert(hw0.rx.size() <= 16);
        published = driver.service(fake_now);
    }
    assert(published && driver.sample().contact_details_available);
    driver.poll();
    fake_now += 20001;
    driver.poll();
    resets.reset_done = 0;
    const size_t written = hw0.written.size();
    for (int i = 0; i < 100; ++i)
        tick(driver, false);
    assert(hw0.written.size() == written);
    resets.reset_done = ~0u;
    assert(acquire(driver));
}

void test_active_report_interval_default_and_diagnostic_baseline() {
    auto driver = make_driver();
    assert(driver.request_active_report_interval(8));
    for (int i = 0; i < 20; ++i)
        tick(driver);
    assert(report_interval_writes.empty());  // Configuration waits for RDY.

    fake_rdy = true;
    for (int i = 0; i < 100 && driver.timing().active_report_interval_ms != 8; ++i)
        tick(driver);
    fake_rdy = false;
    assert(driver.timing().active_report_interval_ms == 8);
    assert(report_interval_writes.size() == 1);
    assert((report_interval_writes[0] == std::vector<uint8_t>{ 0, 8 }));
    assert(!driver.service(fake_now));  // A setting write is not an input report.

    assert(driver.request_active_report_interval(13));
    for (int i = 0; i < 20; ++i)
        tick(driver);
    assert(report_interval_writes.size() == 1);
    fake_rdy = true;
    for (int i = 0; i < 100 && driver.timing().active_report_interval_ms != 13; ++i)
        tick(driver);
    fake_rdy = false;
    assert(driver.timing().active_report_interval_ms == 13);
    assert(report_interval_writes.size() == 2);
    assert((report_interval_writes[1] == std::vector<uint8_t>{ 0, 13 }));
    assert(!driver.service(fake_now));
    assert(report_interval_writes.size() == 2);
    assert(!driver.request_active_report_interval(7));
    assert(!driver.request_active_report_interval(50));
}

Tps43Iqs5xxDriver make_driver() {
    fake_now = 100;
    fake_rdy = false;
    hw0.tx.clear();
    hw0.rx.clear();
    hw0.written.clear();
    hw0.raw_intr_stat = hw0.tx_abrt_source = 0;
    address = 0;
    address_bytes = offset = closes = resets_seen = 0;
    report_interval_writes.clear();
    pending_write.clear();
    nack_once = false;
    compact = { 0, 1, 0, 0, 1, 1, 0xff, 0xfe, 0, 3 };
    contacts = {};
    Tps43Iqs5xxDriver driver({ i2c0, 0x74, 4, 5, 8, 400000 });
    assert(driver.initialize());
    return driver;
}

void step_bus() {
    if (hw0.tx.empty())
        return;
    const uint32_t command = hw0.tx.front();
    hw0.tx.pop_front();
    if (nack_once) {
        nack_once = false;
        hw0.tx.clear();
        hw0.raw_intr_stat |= I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS | I2C_IC_RAW_INTR_STAT_STOP_DET_BITS;
        hw0.tx_abrt_source = 1;
        address_bytes = offset = 0;
        return;
    }
    if (command & I2C_IC_DATA_CMD_CMD_BITS) {
        assert(address_bytes == 2);
        assert((offset != 0) || (command & I2C_IC_DATA_CMD_RESTART_BITS));
        assert(hw0.rx.size() < 16);
        hw0.rx.push_back(address == 0x000C ? compact.at(offset++) : contacts.at(offset++));
    } else if (address_bytes < 2) {
        address = static_cast<uint16_t>((address << 8) | (command & 0xff));
        ++address_bytes;
    } else {
        if (address == 0xEEEE) {
            assert((command & 0xff) == 1);
            ++closes;
        } else {
            assert(address == 0x057A);
            pending_write.push_back(static_cast<uint8_t>(command & 0xff));
        }
    }
    if (command & I2C_IC_DATA_CMD_STOP_BITS) {
        hw0.raw_intr_stat |= I2C_IC_RAW_INTR_STAT_STOP_DET_BITS;
        if (address == 0x057A) {
            report_interval_writes.push_back(pending_write);
            pending_write.clear();
        }
        address_bytes = offset = 0;
    }
}

void tick(Tps43Iqs5xxDriver& driver, bool advance_bus) {
    fake_now += 25;
    const size_t before = hw0.written.size();
    driver.poll();
    assert(hw0.written.size() - before <= 16);
    if (advance_bus)
        step_bus();
}

bool acquire(Tps43Iqs5xxDriver& driver) {
    for (int i = 0; i < 1000; ++i) {
        tick(driver);
        if (driver.service(fake_now))
            return true;
    }
    return false;
}
}  // namespace

FakeRegister::operator uint32_t() const {
    switch (kind) {
        case Data: {
            assert(!hw->rx.empty());
            const auto value = hw->rx.front();
            hw->rx.pop_front();
            return value;
        }
        case RxLevel:
            return hw->rx.size();
        case TxLevel:
            return hw->tx.size();
        case ClearStop:
            hw->raw_intr_stat &= ~I2C_IC_RAW_INTR_STAT_STOP_DET_BITS;
            return 0;
        case ClearAbort:
            hw->raw_intr_stat &= ~I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS;
            hw->tx_abrt_source = 0;
            return 0;
    }
    return 0;
}

void FakeRegister::operator=(uint32_t value) {
    assert(kind == Data && hw->tx.size() < 16);
    hw->tx.push_back(value);
    hw->written.push_back(value);
}

void i2c_init(i2c_inst_t*, uint32_t) {}
void reset_block_num(uint32_t) {
    ++resets_seen;
    hw0.tx.clear();
    hw0.rx.clear();
    hw0.raw_intr_stat = hw0.tx_abrt_source = 0;
    address_bytes = offset = 0;
}
void unreset_block_num(uint32_t) {}

// Runs bus fault, framing, publication, and timer rollover regressions; no inputs.
int main() {
    test_compact_and_no_replay();
    test_contact_and_close_gate();
    test_timeout_recovery_and_retained_sample();
    test_stuck_cleanup_is_bounded();
    test_forced_wake_and_fault_rejection();
    test_diagnostic_same_window_and_clock_rollover();
    test_delayed_fifo_poll_and_reset_not_ready();
    test_active_report_interval_default_and_diagnostic_baseline();
    puts("asynchronous driver tests passed");
}
