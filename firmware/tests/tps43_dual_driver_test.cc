#include <hardware/resets.h>

#include "dual_tps43_coordinator.h"
#include "tps43_iqs5xx_driver.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>

uint64_t fake_now = 100;
bool fake_rdy = false;
i2c_hw_t hw0, hw1;
i2c_inst_t bus0{ &hw0 }, bus1{ &hw1 };
i2c_inst_t *i2c0 = &bus0, *i2c1 = &bus1;
FakeResets resets;
FakeResets* resets_hw = &resets;

namespace {

struct SensorModel {
    i2c_hw_t* hw;
    std::array<uint8_t, 10> compact;
    uint16_t address = 0;
    unsigned address_bytes = 0;
    unsigned offset = 0;
    unsigned closes = 0;
    bool rdy = false;
    bool stall = false;
};

SensorModel left{ &hw1, { 0, 0, 0, 0, 1, 1, 0, 4, 0, 6 } };
SensorModel right{ &hw0, { 0, 0, 0, 0, 1, 1, 0, 2, 0, 3 } };

class RecordingProcessor final : public DualPadProcessor {
   public:
    LogicalActions process(const DualPadSnapshot& snapshot) override {
        last_snapshot = snapshot;
        ++call_count;
        return {};
    }

    DualPadSnapshot last_snapshot;
    unsigned call_count = 0;
};

class NullActionSink final : public Tps43ActionSink {
   public:
    void apply(const LogicalActions&) override {
    }
};

void clear_model(SensorModel& model) {
    model.hw->tx.clear();
    model.hw->rx.clear();
    model.hw->written.clear();
    model.hw->raw_intr_stat = model.hw->tx_abrt_source = 0;
    model.address = 0;
    model.address_bytes = model.offset = model.closes = 0;
    model.rdy = model.stall = false;
}

void step_bus(SensorModel& model) {
    if (model.stall || model.hw->tx.empty()) {
        return;
    }
    const uint32_t command = model.hw->tx.front();
    model.hw->tx.pop_front();
    if (command & I2C_IC_DATA_CMD_CMD_BITS) {
        assert(model.address_bytes == 2);
        assert(model.address == 0x000C);
        model.hw->rx.push_back(model.compact.at(model.offset++));
    } else if (model.address_bytes < 2) {
        model.address = static_cast<uint16_t>((model.address << 8) | (command & 0xff));
        ++model.address_bytes;
    } else {
        assert(model.address == 0xEEEE && (command & 0xff) == 1);
        ++model.closes;
    }
    if (command & I2C_IC_DATA_CMD_STOP_BITS) {
        model.hw->raw_intr_stat |= I2C_IC_RAW_INTR_STAT_STOP_DET_BITS;
        model.address = 0;
        model.address_bytes = model.offset = 0;
    }
}

void tick(Tps43Iqs5xxDriver& left_driver, Tps43Iqs5xxDriver& right_driver) {
    fake_now += 25;
    left_driver.poll();
    right_driver.poll();
    step_bus(left);
    step_bus(right);
}

void verify_independent_simultaneous_publication() {
    clear_model(left);
    clear_model(right);
    Tps43Iqs5xxDriver left_driver({ i2c1, 0x74, 6, 7, 9, 400000 });
    Tps43Iqs5xxDriver right_driver({ i2c0, 0x74, 4, 5, 8, 400000 });
    assert(left_driver.initialize() && right_driver.initialize());
    left.rdy = right.rdy = true;

    bool left_published = false;
    bool right_published = false;
    for (int i = 0; i < 1000 && !(left_published && right_published); ++i) {
        tick(left_driver, right_driver);
        left_published = left_driver.service(fake_now) || left_published;
        right_published = right_driver.service(fake_now) || right_published;
    }

    assert(left_published && right_published);
    assert(left_driver.sample().relative_x == 4 && left_driver.sample().relative_y == 6);
    assert(right_driver.sample().relative_x == 2 && right_driver.sample().relative_y == 3);
    assert(left.closes == 1 && right.closes == 1);
}

void verify_left_fault_does_not_block_right() {
    clear_model(left);
    clear_model(right);
    Tps43Iqs5xxDriver left_driver({ i2c1, 0x74, 6, 7, 9, 400000 });
    Tps43Iqs5xxDriver right_driver({ i2c0, 0x74, 4, 5, 8, 400000 });
    assert(left_driver.initialize() && right_driver.initialize());
    left.rdy = right.rdy = true;
    left.stall = true;

    bool right_published = false;
    for (int i = 0; i < 1000 && !right_published; ++i) {
        tick(left_driver, right_driver);
        assert(!left_driver.service(fake_now));
        right_published = right_driver.service(fake_now);
    }
    assert(right_published && right_driver.sample().relative_y == 3);

    fake_now += 20001;
    left_driver.poll();
    assert(left_driver.timing().transfer_failures == 1);
    assert(left_driver.timing().transfer_timeouts == 1);
    assert(right_driver.timing().transfer_failures == 0);
}

void verify_coherent_normalized_snapshot() {
    clear_model(left);
    clear_model(right);
    Tps43Iqs5xxDriver left_driver({ i2c1, 0x74, 6, 7, 9, 400000 });
    Tps43Iqs5xxDriver right_driver({ i2c0, 0x74, 4, 5, 8, 400000 });
    assert(left_driver.initialize() && right_driver.initialize());
    left.rdy = right.rdy = true;

    // Complete both hardware acquisitions before the one logical-cycle handoff.
    for (int i = 0; i < 1000 && (left.closes == 0 || right.closes == 0); ++i) {
        tick(left_driver, right_driver);
    }
    tick(left_driver, right_driver);
    RecordingProcessor processor;
    NullActionSink sink;
    DualTps43Coordinator coordinator(left_driver, right_driver, processor, sink);
    coordinator.service(fake_now);

    const DualPadSnapshot& snapshot = processor.last_snapshot;
    assert(processor.call_count == 1);
    assert(snapshot.cycle_timestamp_us == fake_now);
    assert(snapshot.left.fresh_sample && snapshot.right.fresh_sample);
    assert(snapshot.left.touch_started && snapshot.right.touch_started);
    assert(snapshot.left.relative_x == 4 && snapshot.left.relative_y == 6);
    assert(snapshot.right.relative_x == 2 && snapshot.right.relative_y == 3);
}

}  // namespace

FakeRegister::operator uint32_t() const {
    switch (kind) {
        case Data: {
            assert(!hw->rx.empty());
            const uint8_t value = hw->rx.front();
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

void i2c_init(i2c_inst_t*, uint32_t) {
}

void reset_block_num(uint32_t block) {
    SensorModel& model = block == RESET_I2C1 ? left : right;
    model.hw->tx.clear();
    model.hw->rx.clear();
    model.hw->raw_intr_stat = model.hw->tx_abrt_source = 0;
    model.address = 0;
    model.address_bytes = model.offset = 0;
}

void unreset_block_num(uint32_t) {
}

bool gpio_get(uint8_t pin) {
    return pin == 9 ? left.rdy : right.rdy;
}

// Verifies independent two-bus publication and fault isolation; no inputs.
int main() {
    verify_independent_simultaneous_publication();
    verify_left_fault_does_not_block_right();
    verify_coherent_normalized_snapshot();
    puts("dual asynchronous driver tests passed");
}
