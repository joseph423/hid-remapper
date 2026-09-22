#include <cassert>
#include <cstdint>
#include <stdexcept>

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
        pending_ = true;
    }

    bool service(uint64_t now_us) override {
        service_time_us = now_us;
        if (!pending_) {
            return false;
        }
        current_sample_ = next_sample_;
        pending_ = false;
        return true;
    }

    Tps43Sample sample() const override {
        return current_sample_;
    }

    uint64_t service_time_us = 0;

   private:
    bool pending_ = false;
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

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void verify_acquisition_edges() {
    MockTps43Driver left, right;
    RecordingProcessor processor;
    RecordingActionSink sink;
    DualTps43Coordinator coordinator(left, right, processor, sink);
    coordinator.service(1);
    require(!processor.last_snapshot.right.fresh_sample && !processor.last_snapshot.right.active,
        "no acquisition must leave initial state inactive");

    right.set_next_sample(compact_sample(true, 1, 0, 3, 100));
    coordinator.service(100);
    require(sink.last_actions.cursor_y == 3, "fresh displacement missing");
    coordinator.service(200);
    const PadState retained = processor.last_snapshot.right;
    require(sink.last_actions.cursor_y == 0 && !retained.fresh_sample && retained.active &&
                retained.session_id == 1 && retained.timestamp_us == 100 && !retained.touch_started,
        "a retained acquisition must preserve touch state without replaying input");

    // Equal timestamps and payloads do not identify duplicate acquisitions.
    right.set_next_sample(compact_sample(true, 1, 0, 3, 100));
    left.set_next_sample(compact_sample(true, 1, 2, 0, 250));
    coordinator.service(250);
    require(sink.last_actions.cursor_y == 3 && processor.last_snapshot.right.fresh_sample &&
                processor.last_snapshot.right.sample_interval_us == 0 && processor.last_snapshot.left.touch_started,
        "distinct acquisitions must survive equal timestamps and payloads");
    left.set_next_sample(compact_sample(true, 1, 1, 0, 300));
    coordinator.service(300);
    require(processor.last_snapshot.left.relative_x == 1 && !processor.last_snapshot.right.fresh_sample &&
                sink.last_actions.cursor_y == 0,
        "pads must advance independently");

    Tps43Sample release;
    release.timestamp_us = 400;
    release.single_tap = release.two_finger_tap = release.scroll_gesture = true;
    right.set_next_sample(release);
    coordinator.service(400);
    require(processor.last_snapshot.right.touch_ended && processor.last_snapshot.right.single_tap &&
                processor.last_snapshot.right.two_finger_tap && processor.last_snapshot.right.scroll_gesture,
        "fresh gesture and release events missing");
    coordinator.service(500);
    const PadState& ended = processor.last_snapshot.right;
    require(!ended.touch_ended && !ended.single_tap && !ended.two_finger_tap && !ended.scroll_gesture && !ended.active,
        "release and gesture events must not replay");
    require(processor.call_count == 7 && sink.call_count == 7,
        "logical processing must continue on cycles without acquisitions");

    right.set_next_sample(three_finger_sample(0, 0, false, 600));
    coordinator.service(600);
    coordinator.service(650);
    right.set_next_sample(three_finger_sample(3, 5, true, 700));
    coordinator.service(700);
    require(processor.last_snapshot.right.three_finger_delta_valid &&
                processor.last_snapshot.right.three_finger_delta_y == 5 &&
                processor.last_snapshot.right.sample_interval_us == 100,
        "acquisition gaps must preserve the centroid baseline and acquisition interval");
    coordinator.service(750);
    require(!processor.last_snapshot.right.three_finger_delta_valid &&
                processor.last_snapshot.right.three_finger_delta_y == 0 &&
                processor.last_snapshot.right.three_finger_centroid_valid,
        "centroid displacement must not replay");
    right.set_next_sample(three_finger_sample(6, 10, true, 800));
    coordinator.service(800);
    require(processor.last_snapshot.right.three_finger_delta_y == 5,
        "centroid movement after a gap must use the last acquisition baseline");
}

DualTps43Tuning acquisition_tuning() {
    return { 200000, 200, 5, { 256, 1024, 1000, 128, 10000 },
        { 256, 768, 1000, 256, 10000 }, { 256, 256, 192, 100 } };
}

// Exercises the production coordinator and FSM with independently published
// acquisitions. tick() returns logical actions, including acquisition-free coast.
class AcquisitionHarness {
   public:
    MockTps43Driver left, right;

    LogicalActions tick(uint64_t now_us) {
        coordinator_.service(now_us);
        return sink_.last_actions;
    }

   private:
    DualTps43Fsm fsm_{ acquisition_tuning() };
    RecordingActionSink sink_;
    DualTps43Coordinator coordinator_{ left, right, fsm_, sink_ };
};

void verify_motion_acquisition_timing() {
    AcquisitionHarness sparse, extra_ticks;
    for (AcquisitionHarness* harness : { &sparse, &extra_ticks }) {
        harness->right.set_next_sample(compact_sample(true, 1, 0, 0, 10000));
        harness->tick(10000);
        harness->right.set_next_sample(compact_sample(true, 1, 10, 0, 20000));
        require(harness->tick(20000).cursor_x == 25, "first filtered velocity gain must match acquisition timing");
    }
    for (uint64_t time : { 21000, 25000, 29000 }) {
        require(extra_ticks.tick(time).cursor_x == 0, "missing acquisitions must not emit cursor movement");
    }
    sparse.right.set_next_sample(compact_sample(true, 1, 10, 0, 30000));
    extra_ticks.right.set_next_sample(compact_sample(true, 1, 10, 0, 30000));
    const int32_t expected = sparse.tick(30000).cursor_x;
    require(expected > 25 && extra_ticks.tick(35000).cursor_x == expected,
        "extra ticks and delayed delivery must not reset filtering or shorten the acquisition interval");
    extra_ticks.right.set_next_sample(compact_sample(true, 1, 0, 0, 40000));
    require(extra_ticks.tick(40000).cursor_x == 0, "a fresh stationary acquisition must stop cursor output");
    extra_ticks.right.set_next_sample(compact_sample(true, 1, 10, 0, 50000));
    require(extra_ticks.tick(50000).cursor_x == 25, "fresh stationary input must reset cursor history");
}

void verify_scroll_gaps_and_stationary_intent() {
    for (bool left_scroll : { false, true }) {
        for (bool stationary : { false, true }) {
            AcquisitionHarness harness;
            MockTps43Driver& driver = left_scroll ? harness.left : harness.right;
            auto sample = compact_sample(true, left_scroll ? 1 : 2, 0, 0, 10000);
            driver.set_next_sample(sample);
            harness.tick(10000);
            sample.relative_y = 20;
            sample.movement_reported = sample.scroll_gesture = true;
            sample.timestamp_us = 20000;
            driver.set_next_sample(sample);
            require(harness.tick(20000).scroll_y == 60, "setup scroll gain missing");
            require(harness.tick(25000).scroll_y == 0, "scroll acquisition must not replay");
            if (stationary) {
                sample.relative_y = 0;
                sample.movement_reported = sample.scroll_gesture = false;
                sample.timestamp_us = 30000;
                driver.set_next_sample(sample);
            }
            require(harness.tick(30000).scroll_y == 0, "stationary or missing acquisition must emit no active scroll");
            driver.set_next_sample(compact_sample(false, 0, 0, 0, 40000));
            require(harness.tick(40000).scroll_y == 0, "release must not replay scroll");
            require(harness.tick(50000).scroll_y == (stationary ? 0 : 45),
                "only a fresh stationary sample may clear release velocity; momentum must run without acquisitions");
        }
    }

    AcquisitionHarness intent;
    intent.right.set_next_sample(compact_sample(true, 1, 2, 0, 100));
    intent.tick(100);
    intent.left.set_next_sample(compact_sample(true, 1, 0, 0, 200));
    intent.tick(200);
    require(intent.tick(400).left_button == ButtonAction::Press,
        "stationary intent must advance on logical cycles without acquisitions");
}

void verify_service_order_equivalence() {
    MockTps43Driver left_a, right_a, left_b, right_b;
    RecordingProcessor processor_a, processor_b;
    RecordingActionSink sink_a, sink_b;
    DualTps43Coordinator left_first(left_a, right_a, processor_a, sink_a);
    DualTps43Coordinator right_first(left_b, right_b, processor_b, sink_b, true);
    const Tps43Sample left_sample = compact_sample(true, 1, 4, 0, 1000);
    const Tps43Sample right_sample = compact_sample(true, 2, 0, 7, 1000);
    left_a.set_next_sample(left_sample);
    right_a.set_next_sample(right_sample);
    left_b.set_next_sample(left_sample);
    right_b.set_next_sample(right_sample);
    left_first.service(1000);
    right_first.service(1000);
    require(sink_a.last_actions.cursor_x == sink_b.last_actions.cursor_x && sink_a.last_actions.cursor_y == sink_b.last_actions.cursor_y,
        "service order must not change logical actions");
    require(processor_a.last_snapshot.left.relative_x == processor_b.last_snapshot.left.relative_x &&
                processor_a.last_snapshot.right.relative_y == processor_b.last_snapshot.right.relative_y &&
                processor_a.last_snapshot.left.touch_started == processor_b.last_snapshot.left.touch_started &&
                processor_a.last_snapshot.right.touch_started == processor_b.last_snapshot.right.touch_started,
        "service order must not change the coherent snapshot");
}

}  // namespace

int main() {
    verify_acquisition_edges();
    verify_motion_acquisition_timing();
    verify_scroll_gaps_and_stationary_intent();
    verify_service_order_equivalence();
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
