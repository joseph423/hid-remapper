#include <cassert>
#include <cstdint>

#include "dual_tps43_coordinator.h"

namespace {

Tps43Sample compact_sample(bool active, uint8_t finger_count, int32_t relative_x, int32_t relative_y, uint64_t timestamp_us) {
    Tps43Sample sample;
    sample.active = active;
    sample.finger_count = finger_count;
    sample.relative_x = relative_x;
    sample.relative_y = relative_y;
    sample.movement_reported = relative_x != 0 || relative_y != 0;
    sample.timestamp_us = timestamp_us;
    return sample;
}

Tps43Sample three_finger_sample(int32_t offset_x, int32_t offset_y, bool movement, uint64_t timestamp_us) {
    Tps43Sample sample = compact_sample(true, 3, 0, 0, timestamp_us);
    sample.movement_reported = movement;
    sample.contact_details_available = true;
    sample.contacts[0] = { true, static_cast<uint16_t>(10 + offset_x), static_cast<uint16_t>(20 + offset_y), 100, 1 };
    sample.contacts[2] = { true, static_cast<uint16_t>(30 + offset_x), static_cast<uint16_t>(40 + offset_y), 200, 2 };
    sample.contacts[4] = { true, static_cast<uint16_t>(50 + offset_x), static_cast<uint16_t>(60 + offset_y), 300, 3 };
    return sample;
}

}  // namespace

class MockTps43Driver : public Tps43Driver {
   public:
    void set_next_sample(Tps43Sample sample) {
        next_sample_ = sample;
    }

    void service(uint64_t now_us) override {
        service_time_us = now_us;
        current_sample_ = next_sample_;
    }

    Tps43Sample sample() const override {
        return current_sample_;
    }

    uint64_t service_time_us = 0;

   private:
    Tps43Sample next_sample_;
    Tps43Sample current_sample_;
};

class RecordingProcessor : public DualPadProcessor {
   public:
    LogicalActions process(const DualPadSnapshot& snapshot) override {
        last_snapshot = snapshot;
        call_count++;

        LogicalActions actions;
        actions.cursor_x = snapshot.right.relative_x;
        actions.cursor_y = snapshot.right.relative_y;
        return actions;
    }

    DualPadSnapshot last_snapshot;
    int call_count = 0;
};

class RecordingActionSink : public Tps43ActionSink {
   public:
    void apply(const LogicalActions& actions) override {
        last_actions = actions;
        call_count++;
    }

    LogicalActions last_actions;
    int call_count = 0;
};

int main() {
    MockTps43Driver left_driver;
    MockTps43Driver right_driver;
    RecordingProcessor processor;
    RecordingActionSink action_sink;
    DualTps43Coordinator coordinator(left_driver, right_driver, processor, action_sink);

    Tps43Sample left_one_finger = compact_sample(true, 1, 10, 20, 1000);
    Tps43Sample right_one_finger = compact_sample(true, 1, 30, 40, 1000);
    assert(!left_one_finger.contact_details_available);
    assert(!right_one_finger.contact_details_available);
    left_driver.set_next_sample(left_one_finger);
    right_driver.set_next_sample(right_one_finger);
    coordinator.service(1000);

    assert(processor.call_count == 1);
    assert(action_sink.call_count == 1);
    assert(left_driver.service_time_us == 1000);
    assert(right_driver.service_time_us == 1000);
    assert(processor.last_snapshot.left.touch_started);
    assert(processor.last_snapshot.right.touch_started);
    assert(processor.last_snapshot.left.session_id == 1);
    assert(processor.last_snapshot.right.session_id == 1);
    assert(processor.last_snapshot.cycle_timestamp_us == 1000);

    left_driver.set_next_sample(compact_sample(true, 1, 3, 5, 2000));
    right_driver.set_next_sample(compact_sample(true, 1, 7, 11, 2000));
    coordinator.service(2000);

    assert(processor.call_count == 2);
    assert(action_sink.call_count == 2);
    assert(processor.last_snapshot.left.relative_x == 3);
    assert(processor.last_snapshot.left.relative_y == 5);
    assert(processor.last_snapshot.right.relative_x == 7);
    assert(processor.last_snapshot.right.relative_y == 11);
    assert(action_sink.last_actions.cursor_x == 7);
    assert(action_sink.last_actions.cursor_y == 11);

    left_driver.set_next_sample(compact_sample(false, 0, 0, 0, 3000));
    right_driver.set_next_sample(compact_sample(true, 1, 0, 0, 3000));
    coordinator.service(3000);

    assert(processor.call_count == 3);
    assert(action_sink.call_count == 3);
    assert(processor.last_snapshot.left.touch_ended);
    assert(!processor.last_snapshot.right.touch_ended);
    assert(processor.last_snapshot.left.session_id == 1);
    assert(processor.last_snapshot.right.session_id == 1);

    // Three active contacts may occupy non-contiguous hardware slots. The
    // first valid sample establishes a centroid baseline without a delta.
    Tps43Sample right_three_finger_baseline = three_finger_sample(0, 0, false, 4000);
    assert(right_three_finger_baseline.contact_details_available);
    right_driver.set_next_sample(right_three_finger_baseline);
    coordinator.service(4000);

    assert(processor.call_count == 4);
    assert(action_sink.call_count == 4);
    assert(processor.last_snapshot.right.three_finger_centroid_valid);
    assert(processor.last_snapshot.right.three_finger_centroid_x == 30);
    assert(processor.last_snapshot.right.three_finger_centroid_y == 40);
    assert(!processor.last_snapshot.right.three_finger_delta_valid);

    Tps43Sample right_three_finger_movement = three_finger_sample(3, 5, true, 5000);
    assert(right_three_finger_movement.contact_details_available);
    right_driver.set_next_sample(right_three_finger_movement);
    coordinator.service(5000);

    assert(processor.call_count == 5);
    assert(action_sink.call_count == 5);
    assert(processor.last_snapshot.right.movement_reported);
    assert(processor.last_snapshot.right.three_finger_delta_valid);
    assert(processor.last_snapshot.right.three_finger_delta_x == 3);
    assert(processor.last_snapshot.right.three_finger_delta_y == 5);

    // Compact two-finger processing remains independent of optional contact
    // data and clears the previous three-finger centroid baseline.
    Tps43Sample compact_two_finger = compact_sample(true, 2, -4, 6, 6000);
    compact_two_finger.two_finger_tap = true;
    compact_two_finger.scroll_gesture = true;
    assert(!compact_two_finger.contact_details_available);
    right_driver.set_next_sample(compact_two_finger);
    coordinator.service(6000);

    assert(processor.call_count == 6);
    assert(action_sink.call_count == 6);
    assert(processor.last_snapshot.right.relative_x == -4);
    assert(processor.last_snapshot.right.relative_y == 6);
    assert(processor.last_snapshot.right.two_finger_tap);
    assert(processor.last_snapshot.right.scroll_gesture);
    assert(!processor.last_snapshot.right.three_finger_centroid_valid);
    assert(!processor.last_snapshot.right.three_finger_delta_valid);

    return 0;
}
