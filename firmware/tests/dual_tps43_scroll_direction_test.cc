#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "dual_tps43_fsm.h"
#include "pad_state.h"

namespace {

DualTps43Tuning test_tuning() {
    DualTps43Tuning tuning{};
    tuning.tap_max_duration_us = 200000;
    tuning.stationary_intent_threshold_us = 100000;
    tuning.neutral_activation_threshold = 20;
    tuning.cursor_base_scale_q8 = 128;
    tuning.scroll_base_scale_q8 = 256;
    tuning.scroll_momentum = { false, 50, 100 };
    return tuning;
}

Tps43Sample inactive() {
    return {};
}

Tps43Sample left_move(int32_t x, int32_t y) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 1;
    sample.relative_x = x;
    sample.relative_y = y;
    sample.movement_reported = x != 0 || y != 0;
    return sample;
}

Tps43Sample right_scroll(int32_t x, int32_t y) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 2;
    sample.relative_x = x;
    sample.relative_y = y;
    sample.movement_reported = x != 0 || y != 0;
    sample.scroll_gesture = true;
    return sample;
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class Harness {
   public:
    explicit Harness(DualTps43Tuning tuning = test_tuning())
        : fsm_(tuning) {
    }

    LogicalActions step(Tps43Sample left, Tps43Sample right) {
        now_us_ += 8000;
        left.timestamp_us = now_us_;
        right.timestamp_us = now_us_;
        DualPadSnapshot snapshot;
        snapshot.left = left_tracker_.update(left);
        snapshot.right = right_tracker_.update(right);
        snapshot.cycle_timestamp_us = now_us_;
        return fsm_.process(snapshot);
    }

   private:
    uint64_t now_us_ = 0;
    PadStateTracker left_tracker_;
    PadStateTracker right_tracker_;
    DualTps43Fsm fsm_;
};

int failures = 0;
int passes = 0;

void run_case(const char* name, const std::function<void()>& test) {
    try {
        test();
        ++passes;
        std::cout << "PASS " << name << '\n';
    } catch (const std::exception& error) {
        ++failures;
        std::cerr << "FAIL " << name << ": " << error.what() << '\n';
    }
}

}  // namespace

int main() {
    run_case("Left classifies and locks vertical", [] {
        Harness harness;
        const LogicalActions first = harness.step(left_move(1, 5), inactive());
        require(first.scroll_x == 0 && first.scroll_y == 0, "classification should buffer startup motion");
        const LogicalActions classified = harness.step(left_move(1, 2), inactive());
        require(classified.scroll_x == 0 && classified.scroll_y == 7,
            "vertical classification should preserve accumulated Y and discard X");
        const LogicalActions locked = harness.step(left_move(8, 1), inactive());
        require(locked.scroll_x == 0 && locked.scroll_y == 1,
            "the selected vertical mode should remain locked through the touch");
    });

    run_case("Right classifies and locks horizontal", [] {
        Harness harness;
        const LogicalActions first = harness.step(inactive(), right_scroll(5, 1));
        require(first.scroll_x == 0 && first.scroll_y == 0, "classification should buffer startup motion");
        const LogicalActions classified = harness.step(inactive(), right_scroll(3, 0));
        require(classified.scroll_x == 8 && classified.scroll_y == 0,
            "horizontal classification should preserve accumulated X and discard Y");
        const LogicalActions locked = harness.step(inactive(), right_scroll(1, 8));
        require(locked.scroll_x == 1 && locked.scroll_y == 0,
            "the selected horizontal mode should remain locked through the touch");
    });

    run_case("Diagonal mode preserves free two-axis movement", [] {
        Harness harness;
        require(harness.step(inactive(), right_scroll(3, -3)).scroll_x == 0,
            "the initial diagonal vector should wait until the startup threshold");
        const LogicalActions classified = harness.step(inactive(), right_scroll(1, -1));
        require(classified.scroll_x == 4 && classified.scroll_y == -4,
            "classification should emit the buffered diagonal vector unchanged");
        const LogicalActions horizontal_sample = harness.step(inactive(), right_scroll(4, 0));
        require(horizontal_sample.scroll_x == 4 && horizontal_sample.scroll_y == 0,
            "diagonal mode should preserve a later horizontal-only sample without reclassifying");
        const LogicalActions vertical_sample = harness.step(inactive(), right_scroll(1, -5));
        require(vertical_sample.scroll_x == 1 && vertical_sample.scroll_y == -5,
            "diagonal mode should preserve a later vertical-dominant ratio without reclassifying");
    });

    run_case("User thresholds control one-time classification", [] {
        DualTps43Tuning tuning = test_tuning();
        tuning.scroll_direction_classification.classification_distance_counts = 7;
        tuning.scroll_direction_classification.axis_dominance_ratio = 3;
        Harness harness(tuning);
        const LogicalActions classified = harness.step(inactive(), right_scroll(2, 5));
        require(classified.scroll_x == 2 && classified.scroll_y == 5,
            "the configured 7-count threshold and 3:1 ratio should classify this vector as free diagonal");
    });

    run_case("Classifier can be disabled for A/B test", [] {
        DualTps43Tuning tuning = test_tuning();
        tuning.scroll_direction_classification.enabled = false;
        Harness harness(tuning);
        const LogicalActions passthrough = harness.step(inactive(), right_scroll(2, 8));
        require(passthrough.scroll_x == 2 && passthrough.scroll_y == 8,
            "disabled classification should pass both axes immediately without startup buffering");
    });

    run_case("A short scroll flushes unchanged on release", [] {
        Harness harness;
        const LogicalActions buffered = harness.step(left_move(2, 1), inactive());
        require(buffered.scroll_x == 0 && buffered.scroll_y == 0,
            "sub-threshold startup movement should be buffered while the touch remains held");
        const LogicalActions released = harness.step(inactive(), inactive());
        require(released.scroll_x == 2 && released.scroll_y == 1,
            "release should deliver a short gesture without direction shaping");
    });

    std::cout << passes << " passed, " << failures << " failed\n";
    return failures == 0 ? 0 : 1;
}
