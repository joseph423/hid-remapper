#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "dual_tps43_fsm.h"
#include "pad_state.h"

namespace {

DualTps43Tuning motion_tuning() {
    DualTps43Tuning tuning;
    tuning.tap_max_duration_us = 200000;
    tuning.stationary_intent_threshold_us = 200000;
    tuning.neutral_activation_threshold = 50;
    tuning.cursor_gain = { 256, 1024, 1000, 256, 10000 };
    tuning.scroll_gain = { 256, 768, 1000, 256, 10000 };
    tuning.scroll_momentum = { 256, 256, 192, 100 };
    return tuning;
}

Tps43Sample inactive() {
    return {};
}

Tps43Sample one_finger(int32_t x = 0, int32_t y = 0) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 1;
    sample.relative_x = x;
    sample.relative_y = y;
    sample.movement_reported = x != 0 || y != 0;
    return sample;
}

Tps43Sample two_finger(int32_t x = 0, int32_t y = 0) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 2;
    sample.relative_x = x;
    sample.relative_y = y;
    sample.movement_reported = x != 0 || y != 0;
    sample.scroll_gesture = sample.movement_reported;
    return sample;
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_no_cursor_or_buttons(const LogicalActions& actions) {
    require(actions.cursor_x == 0 && actions.cursor_y == 0, "scroll momentum must not move the cursor");
    require(actions.left_button == ButtonAction::None, "scroll momentum must not act on the Left button");
    require(actions.right_button == ButtonAction::None, "scroll momentum must not act on the Right button");
}

// Drives the public FSM boundary with normalized samples and controlled time.
class Harness {
   public:
    // Creates an isolated motion test sequence with the supplied tuning.
    explicit Harness(DualTps43Tuning tuning = motion_tuning())
        : fsm_(tuning) {
    }

    // Advances logical time, normalizes both samples, and returns one cycle's
    // logical actions.
    LogicalActions step(Tps43Sample left, Tps43Sample right, uint64_t advance_us) {
        now_us_ += advance_us;
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
        std::cout << "PASS " << name << '\n';
        passes++;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        failures++;
    }
}

}  // namespace

int main() {
    run_case("MOTION-01 slow-versus-fast-cursor-gain", [] {
        Harness slow;
        slow.step(inactive(), one_finger(), 10000);
        const LogicalActions slow_action = slow.step(inactive(), one_finger(10, 0), 100000);

        Harness fast;
        fast.step(inactive(), one_finger(), 10000);
        const LogicalActions fast_action = fast.step(inactive(), one_finger(10, 0), 10000);

        require(slow_action.cursor_x > 0, "slow cursor input must remain usable");
        require(fast_action.cursor_x > slow_action.cursor_x,
            "faster cursor input must receive greater gain for equal displacement");
        require(slow_action.scroll_x == 0 && fast_action.scroll_x == 0,
            "cursor gain must not create scroll output");
    });

    run_case("MOTION-02 slow-versus-fast-active-scroll-gain", [] {
        Harness slow;
        slow.step(inactive(), two_finger(), 10000);
        const LogicalActions slow_action = slow.step(inactive(), two_finger(0, 10), 100000);

        Harness fast;
        fast.step(inactive(), two_finger(), 10000);
        const LogicalActions fast_action = fast.step(inactive(), two_finger(0, 10), 10000);

        require(slow_action.scroll_y > 0, "slow active scrolling must remain usable");
        require(fast_action.scroll_y > slow_action.scroll_y,
            "faster active scrolling must receive greater gain for equal displacement");
        require(slow_action.cursor_y == 0 && fast_action.cursor_y == 0,
            "active scroll gain must not create cursor output");

        DualTps43Tuning filtered_tuning = motion_tuning();
        filtered_tuning.scroll_gain.velocity_filter_weight_q8 = 128;
        Harness filtered(filtered_tuning);
        filtered.step(inactive(), two_finger(), 10000);
        const LogicalActions first_filtered = filtered.step(inactive(), two_finger(0, 10), 10000);
        const LogicalActions second_filtered = filtered.step(inactive(), two_finger(0, 10), 10000);
        require(second_filtered.scroll_y > first_filtered.scroll_y,
            "partial active-scroll filtering must retain velocity history across equal fast samples");
    });

    run_case("MOTION-03 cursor-stops-with-input", [] {
        Harness harness;
        harness.step(inactive(), one_finger(), 10000);
        require(harness.step(inactive(), one_finger(10, 0), 10000).cursor_x > 0,
            "setup cursor movement must produce output");

        const LogicalActions stopped = harness.step(inactive(), one_finger(), 10000);
        require(stopped.cursor_x == 0 && stopped.cursor_y == 0,
            "cursor output must stop on the first cycle without movement");
        require(stopped.scroll_x == 0 && stopped.scroll_y == 0,
            "stopping cursor input must not start scroll momentum");
    });

    run_case("MOTION-04 scroll-momentum-decay-reaches-zero", [] {
        Harness harness;
        harness.step(inactive(), two_finger(), 10000);
        require(harness.step(inactive(), two_finger(0, 20), 10000).scroll_y > 0,
            "setup scroll movement must produce output");

        const LogicalActions release = harness.step(inactive(), inactive(), 10000);
        require(release.scroll_x == 0 && release.scroll_y == 0,
            "release cycle must seed momentum without replaying movement");
        require_no_cursor_or_buttons(release);

        int32_t previous_magnitude = std::numeric_limits<int32_t>::max();
        bool saw_momentum = false;
        bool reached_zero = false;
        for (int cycle = 0; cycle < 40; cycle++) {
            const LogicalActions action = harness.step(inactive(), inactive(), 10000);
            require_no_cursor_or_buttons(action);
            require(action.scroll_x == 0, "single-axis momentum must not create cross-axis scroll");

            // The first outputs are the exact black-box result of applying the
            // configured 0.75 velocity decay before each 10 ms momentum step.
            const int32_t expected_initial_outputs[] = { 45, 33, 26 };
            if (cycle < 3) {
                require(action.scroll_y == expected_initial_outputs[cycle],
                    "momentum output must follow the configured velocity-decay recurrence");
            }

            const int32_t magnitude = std::abs(action.scroll_y);
            require(magnitude <= previous_magnitude,
                "the configured momentum output must not increase before cutoff");
            if (magnitude > 0) {
                require(!reached_zero,
                    "the configured momentum output must remain zero after reaching cutoff");
                saw_momentum = true;
            } else {
                reached_zero = true;
            }
            previous_magnitude = magnitude;
        }

        require(saw_momentum, "release velocity must produce post-release scroll output");
        require(reached_zero, "momentum must decay to the configured cutoff");
    });

    run_case("MOTION-05 left-scroll-release-seeds-momentum", [] {
        Harness harness;
        require(harness.step(one_finger(0, 20), inactive(), 10000).scroll_y > 0,
            "Left-pad movement must produce active scrolling");
        require(harness.step(inactive(), inactive(), 10000).scroll_y == 0,
            "Left release must seed without replaying movement");

        const LogicalActions momentum = harness.step(inactive(), inactive(), 10000);
        require(momentum.scroll_y > 0, "Left-scroll release velocity must produce momentum");
        require_no_cursor_or_buttons(momentum);
    });

    run_case("MOTION-06 zero-filtered-release-velocity-does-not-coast", [] {
        DualTps43Tuning tuning = motion_tuning();
        // A full-current filter makes this case specifically about a zero
        // filtered release velocity rather than retained filter history.
        tuning.scroll_momentum.release_velocity_filter_weight_q8 = 256;
        Harness harness(tuning);
        harness.step(inactive(), two_finger(), 10000);
        require(harness.step(inactive(), two_finger(0, 20), 10000).scroll_y > 0,
            "setup scroll movement must produce output");

        const LogicalActions stationary = harness.step(inactive(), two_finger(), 10000);
        require(stationary.scroll_x == 0 && stationary.scroll_y == 0,
            "active stationary fingers must stop active scroll output");
        require(harness.step(inactive(), inactive(), 10000).scroll_y == 0,
            "a stationary sample with a full-current filter must clear release velocity");
        require(harness.step(inactive(), inactive(), 10000).scroll_y == 0,
            "momentum must remain stopped after a stationary release");

        DualTps43Tuning partial_tuning = motion_tuning();
        partial_tuning.scroll_momentum.release_velocity_filter_weight_q8 = 128;
        Harness partial(partial_tuning);
        partial.step(inactive(), two_finger(), 10000);
        require(partial.step(inactive(), two_finger(0, 20), 10000).scroll_y > 0,
            "partial-filter setup scroll movement must produce output");
        require(partial.step(inactive(), two_finger(), 10000).scroll_y == 0,
            "a stationary sample must still stop active scroll output with a partial filter");
        require(partial.step(inactive(), inactive(), 10000).scroll_y == 0,
            "partial filtered release must seed without replaying movement");
        require(partial.step(inactive(), inactive(), 10000).scroll_y > 0,
            "a partial release-velocity filter must retain history for momentum launch");
    });

    run_case("MOTION-07 new-touch-cancels-momentum", [] {
        Harness harness;
        harness.step(inactive(), two_finger(), 10000);
        require(harness.step(inactive(), two_finger(0, 20), 10000).scroll_y > 0,
            "setup scroll movement must produce output");
        require(harness.step(inactive(), inactive(), 10000).scroll_y == 0,
            "release cycle must seed without replaying movement");
        require(harness.step(inactive(), inactive(), 10000).scroll_y > 0,
            "setup must establish active momentum");

        const LogicalActions new_touch = harness.step(inactive(), one_finger(), 10000);
        require(new_touch.scroll_x == 0 && new_touch.scroll_y == 0,
            "a new touch must cancel momentum before producing movement");
        require_no_cursor_or_buttons(new_touch);
        require(harness.step(inactive(), one_finger(), 10000).scroll_y == 0,
            "cancelled momentum must not restart while the new touch remains stationary");

        Harness same_cycle;
        same_cycle.step(inactive(), two_finger(), 10000);
        require(same_cycle.step(inactive(), two_finger(0, 20), 10000).scroll_y > 0,
            "same-cycle setup scroll movement must produce output");
        const LogicalActions replacement_touch = same_cycle.step(one_finger(), inactive(), 10000);
        require(replacement_touch.scroll_x == 0 && replacement_touch.scroll_y == 0,
            "a replacement touch in the release cycle must prevent momentum from starting");
        require(same_cycle.step(one_finger(), inactive(), 10000).scroll_y == 0,
            "prevented same-cycle momentum must remain stopped");
    });

    run_case("MOTION-08 velocity-filter-state-affects-gain", [] {
        DualTps43Tuning tuning = motion_tuning();
        tuning.cursor_gain.velocity_filter_weight_q8 = 128;
        Harness harness(tuning);
        harness.step(inactive(), one_finger(), 10000);

        const LogicalActions first = harness.step(inactive(), one_finger(10, 0), 10000);
        const LogicalActions second = harness.step(inactive(), one_finger(10, 0), 10000);
        require(second.cursor_x > first.cursor_x,
            "filtered velocity history must raise gain across consecutive fast samples");
    });

    run_case("MOTION-09 fallback-interval-handles-nonadvancing-time", [] {
        DualTps43Tuning tuning = motion_tuning();
        tuning.cursor_gain.fallback_sample_interval_us = 20000;
        Harness harness(tuning);
        harness.step(inactive(), one_finger(), 10000);
        const LogicalActions action = harness.step(inactive(), one_finger(10, 0), 0);
        require(action.cursor_x == 25,
            "non-advancing time must use the configured 20 ms fallback interval for cursor gain");
        require(action.scroll_x == 0 && action.scroll_y == 0,
            "fallback cursor timing must not create scroll output");
    });

    run_case("MOTION-10 fractional-momentum-displacement-accumulates", [] {
        DualTps43Tuning tuning = motion_tuning();
        tuning.scroll_gain = { 256, 256, 1, 256, 10000 };
        tuning.scroll_momentum = { 256, 256, 255, 0 };
        Harness harness(tuning);
        harness.step(inactive(), two_finger(), 10000);
        require(harness.step(inactive(), two_finger(0, 1), 10000).scroll_y == 1,
            "setup must establish a one-unit active-scroll sample");
        require(harness.step(inactive(), inactive(), 10000).scroll_y == 0,
            "release cycle must seed momentum without replaying movement");

        const LogicalActions first = harness.step(inactive(), inactive(), 10000);
        const LogicalActions second = harness.step(inactive(), inactive(), 10000);
        require(first.scroll_y == 0, "one sub-unit momentum displacement must remain fractional");
        require(second.scroll_y == 1,
            "consecutive sub-unit momentum displacements must accumulate into output");
        require_no_cursor_or_buttons(first);
        require_no_cursor_or_buttons(second);
    });

    std::cout << "Motion result: " << passes << " passed, " << failures << " failed\n";
    return failures == 0 ? 0 : 1;
}
