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
    tuning.left_assisted_drag_axis_threshold = 2;
    tuning.subthreshold_cursor_threshold = 2;
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

// Supplies complete non-contiguous contact slots so normalization establishes
// a real centroid baseline before the entry movement.
Tps43Sample three_finger(uint16_t offset) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 3;
    sample.contact_details_available = true;
    sample.movement_reported = offset != 0;
    for (int slot : { 0, 2, 4 }) {
        sample.contacts[slot] = { true, static_cast<uint16_t>(100 + offset + slot), 100, 10, 1 };
    }
    return sample;
}

void require_no_scroll(const LogicalActions& actions) {
    if (actions.scroll_x != 0 || actions.scroll_y != 0) {
        throw std::runtime_error("Drag must not emit retained scroll motion");
    }
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
    run_case("MOTION-18 adaptive-cursor-filter-uses-speed-bands-only-on-cursor", [] {
        DualTps43Tuning tuning = motion_tuning();
        tuning.cursor_gain = { 256, 256, 4000, 256, 15000 };
        tuning.subthreshold_cursor_threshold = 1;
        tuning.cursor_temporal_filter_enabled = true;
        tuning.cursor_filter_slow_speed_limit_counts_per_second = 80;
        tuning.cursor_filter_fast_speed_limit_counts_per_second = 300;
        tuning.cursor_filter_slow_weight_percent = 20;
        tuning.cursor_filter_normal_weight_percent = 10;
        tuning.cursor_filter_fast_weight_percent = 0;

        Harness slow(tuning);
        slow.step(inactive(), one_finger(), 13000);
        const LogicalActions slow_first = slow.step(inactive(), one_finger(1, 0), 13000);
        const LogicalActions slow_second = slow.step(inactive(), one_finger(1, 0), 13000);
        require(slow_first.cursor_x_q8 == 204,
            "very slow input at or below the slow limit must use the configured slow-band weight");
        require(slow_second.cursor_x_q8 == 244,
            "very slow input must continue blending with the previous filtered output");

        Harness normal(tuning);
        normal.step(inactive(), one_finger(), 8000);
        const LogicalActions normal_first = normal.step(inactive(), one_finger(1, 0), 8000);
        require(normal_first.cursor_x_q8 == 230,
            "normal input above the slow and at or below the fast limit must use its configured weight");

        Harness fast(tuning);
        fast.step(inactive(), one_finger(), 8000);
        const LogicalActions fast_first = fast.step(inactive(), one_finger(3, 0), 8000);
        require(fast_first.cursor_x_q8 == 768,
            "fast input above the fast limit must use its configured weight");

        const LogicalActions scroll = slow.step(two_finger(), two_finger(), 8000);
        require(scroll.cursor_x_q8 == 0 && scroll.cursor_y_q8 == 0,
            "scroll-only input must not create cursor output");
        const LogicalActions scrolling = slow.step(two_finger(0, 1), two_finger(), 8000);
        require(scrolling.scroll_y_q8 != 0, "cursor filtering must leave active scroll available");

        tuning.cursor_filter_slow_weight_percent = 100;
        Harness extreme(tuning);
        extreme.step(inactive(), one_finger(), 13000);
        const LogicalActions extreme_slow = extreme.step(inactive(), one_finger(1, 0), 13000);
        require(extreme_slow.cursor_x_q8 == 0,
            "100% previous-output weight should make the slow-band effect unmistakable");

        tuning.cursor_filter_slow_speed_limit_counts_per_second = 150;
        tuning.cursor_filter_fast_speed_limit_counts_per_second = 400;
        tuning.cursor_filter_slow_weight_percent = 30;
        tuning.cursor_filter_normal_weight_percent = 40;
        tuning.cursor_filter_fast_weight_percent = 50;
        Harness customized(tuning);
        customized.step(inactive(), one_finger(), 8000);
        const LogicalActions customized_slow = customized.step(inactive(), one_finger(1, 0), 8000);
        require(customized_slow.cursor_x_q8 == 179,
            "custom slow cutoff and weight must be applied to the slow band");

        Harness customized_normal(tuning);
        customized_normal.step(inactive(), one_finger(), 8000);
        const LogicalActions normal_boundary = customized_normal.step(inactive(), one_finger(2, 0), 8000);
        require(normal_boundary.cursor_x_q8 == 307,
            "custom normal-band weight must be independent of the slow-band weight");

        Harness customized_fast(tuning);
        customized_fast.step(inactive(), one_finger(), 8000);
        const LogicalActions fast_boundary = customized_fast.step(inactive(), one_finger(4, 0), 8000);
        require(fast_boundary.cursor_x_q8 == 512,
            "custom fast-band weight must be independently applied above the fast cutoff");
    });

    run_case("MOTION-19 cursor-filter-disable-and-stop-reset", [] {
        DualTps43Tuning tuning = motion_tuning();
        tuning.cursor_gain = { 256, 256, 4000, 256, 15000 };
        tuning.subthreshold_cursor_threshold = 1;
        tuning.cursor_temporal_filter_enabled = true;
        tuning.cursor_filter_slow_weight_percent = 20;
        tuning.cursor_filter_normal_weight_percent = 10;
        tuning.cursor_filter_fast_weight_percent = 0;
        Harness filtered(tuning);
        filtered.step(inactive(), one_finger(), 13000);
        const LogicalActions first = filtered.step(inactive(), one_finger(1, 0), 13000);
        require(first.cursor_x_q8 == 204, "enabled slow band must apply its adaptive filter strength");

        const LogicalActions stopped = filtered.step(inactive(), one_finger(), 13000);
        require(stopped.cursor_x_q8 == 0 && stopped.cursor_y_q8 == 0,
            "zero-motion sample must reset without emitting a filter tail");
        const LogicalActions restarted = filtered.step(inactive(), one_finger(1, 0), 13000);
        require(restarted.cursor_x_q8 == 204,
            "movement after a stop must start with clean filter state");

        const LogicalActions lifted = filtered.step(inactive(), inactive(), 13000);
        require(lifted.cursor_x_q8 == 0 && lifted.cursor_y_q8 == 0,
            "finger lift must discard filter residual without emitting cursor movement");
        filtered.step(inactive(), one_finger(), 13000);
        const LogicalActions after_lift = filtered.step(inactive(), one_finger(1, 0), 13000);
        require(after_lift.cursor_x_q8 == 204,
            "a new touch after lift must start with clean filter state");

        tuning.cursor_temporal_filter_enabled = false;
        Harness unfiltered(tuning);
        unfiltered.step(inactive(), one_finger(), 13000);
        const LogicalActions passthrough = unfiltered.step(inactive(), one_finger(1, 0), 13000);
        require(passthrough.cursor_x_q8 == 256,
            "disabled filter must preserve the original scaled cursor delta");
    });

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

    run_case("MOTION-14 accumulates-subthreshold-cursor-deltas-per-axis", [] {
        Harness harness;
        harness.step(inactive(), one_finger(), 10000);
        Tps43Sample first_delta = one_finger(1, 1);
        first_delta.movement_reported = false;
        const LogicalActions first_action = harness.step(inactive(), first_delta, 10000);
        require(first_action.cursor_x == 0 && first_action.cursor_y == 0,
            "one-count diagonal delta must wait for confirmation");

        Tps43Sample second_delta = one_finger(1, 1);
        second_delta.movement_reported = false;
        const LogicalActions second_action = harness.step(inactive(), second_delta, 10000);
        require(second_action.cursor_x > 0 && second_action.cursor_y > 0,
            "consistent diagonal single-count deltas must combine and reach the cursor");
        require(second_action.scroll_x == 0 && second_action.scroll_y == 0,
            "accumulated cursor input must not become scroll output");
    });

    run_case("MOTION-15 cancels-reversals-and-forwards-classified-cursor-deltas", [] {
        Harness larger_delta;
        larger_delta.step(inactive(), one_finger(), 10000);
        Tps43Sample positive_delta = one_finger(1, 0);
        positive_delta.movement_reported = false;
        const LogicalActions pending_action = larger_delta.step(inactive(), positive_delta, 10000);
        require(pending_action.cursor_x == 0, "first unclassified single-count delta must remain pending");
        Tps43Sample reverse_delta = one_finger(-1, 0);
        reverse_delta.movement_reported = false;
        const LogicalActions reversed_action = larger_delta.step(inactive(), reverse_delta, 10000);
        require(reversed_action.cursor_x == 0,
            "opposing single-count deltas must cancel instead of causing drift");

        Harness classified_delta;
        classified_delta.step(inactive(), one_finger(), 10000);
        Tps43Sample sensor_classified_delta = one_finger(1, 0);
        sensor_classified_delta.movement_reported = true;
        const LogicalActions classified_action = classified_delta.step(inactive(), sensor_classified_delta, 10000);
        require(classified_action.cursor_x > 0,
            "movement-classified single-count delta must preserve existing cursor behavior");
    });

    run_case("MOTION-16 expires-stale-subthreshold-cursor-deltas", [] {
        Harness harness;
        harness.step(inactive(), one_finger(), 10000);
        Tps43Sample first_delta = one_finger(1, 0);
        first_delta.movement_reported = false;
        harness.step(inactive(), first_delta, 10000);
        Tps43Sample later_delta = one_finger(1, 0);
        later_delta.movement_reported = false;
        const LogicalActions later_action = harness.step(inactive(), later_delta, 60000);
        require(later_action.cursor_x == 0,
            "single-count deltas separated beyond the expiry window must not combine");
    });

    run_case("MOTION-17 honors-configured-subthreshold-cursor-threshold", [] {
        DualTps43Tuning tuning = motion_tuning();
        tuning.subthreshold_cursor_threshold = 3;
        Harness harness(tuning);
        harness.step(inactive(), one_finger(), 10000);

        for (int i = 0; i < 2; i++) {
            Tps43Sample delta = one_finger(1, 0);
            delta.movement_reported = false;
            const LogicalActions action = harness.step(inactive(), delta, 8000);
            require(action.cursor_x == 0, "threshold 3 must hold the first two unclassified counts");
        }

        Tps43Sample third_delta = one_finger(1, 0);
        third_delta.movement_reported = false;
        const LogicalActions action = harness.step(inactive(), third_delta, 8000);
        require(action.cursor_x > 0, "threshold 3 must emit once the accumulated axis reaches 3");
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

    run_case("MOTION-13 fractional-motion-reaches-output-boundary", [] {
        DualTps43Tuning tuning = motion_tuning();
        tuning.cursor_gain = { 128, 128, 1000, 256, 10000 };
        tuning.scroll_gain = { 4, 4, 1000, 256, 10000 };
        tuning.scroll_momentum = { 0, 0, 0, 0 };

        Harness cursor(tuning);
        cursor.step(inactive(), one_finger(), 10000);
        const LogicalActions first_cursor = cursor.step(inactive(), one_finger(1, 0), 10000);
        const LogicalActions second_cursor = cursor.step(inactive(), one_finger(1, 0), 10000);
        require(first_cursor.cursor_x == 0 && first_cursor.cursor_x_q8 == 128,
            "cursor output must retain a sub-unit Q8 displacement");
        require(second_cursor.cursor_x == 1 && second_cursor.cursor_x_q8 == 128,
            "cursor Q8 displacement must remain the new per-cycle delta");

        Harness scroll(tuning);
        scroll.step(inactive(), two_finger(), 10000);
        const LogicalActions first_scroll = scroll.step(inactive(), two_finger(0, 1), 10000);
        const LogicalActions second_scroll = scroll.step(inactive(), two_finger(0, 1), 10000);
        require(first_scroll.scroll_y == 0 && first_scroll.scroll_y_q8 == 4,
            "scroll output must retain a sub-unit Q8 displacement");
        require(second_scroll.scroll_y == 0 &&
                    first_scroll.scroll_y_q8 + second_scroll.scroll_y_q8 == 8,
            "scroll Q8 displacement must remain available before whole-step emission");
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

    run_case("MOTION-11 scroll-history-cannot-escape-right-latch", [] {
        for (uint16_t weight : { 128, 256 }) {
            DualTps43Tuning tuning = motion_tuning();
            tuning.scroll_momentum.release_velocity_filter_weight_q8 = weight;
            tuning.scroll_momentum.stop_velocity_logical_units_per_second = 0;
            Harness harness(tuning);
            harness.step(inactive(), two_finger(), 10000);
            require(harness.step(inactive(), two_finger(10, 20), 10000).scroll_y > 0,
                "setup must establish scroll history");
            require_no_scroll(harness.step(inactive(), three_finger(0), 10000));
            const LogicalActions entry = harness.step(inactive(), three_finger(10), 10000);
            require_no_scroll(entry);
            require(entry.left_button == ButtonAction::Press && entry.cursor_x == 40,
                "drag entry must press and retain velocity-scaled centroid movement");
            const LogicalActions continued = harness.step(inactive(), three_finger(20), 10000);
            require_no_scroll(continued);
            require(continued.cursor_x == 40 && continued.left_button == ButtonAction::None,
                "continued centroid movement must retain the latch");
            for (int cycle = 0; cycle < 5; cycle++) {
                const LogicalActions inactive_cycle = harness.step(inactive(), inactive(), 10000);
                require_no_scroll(inactive_cycle);
                require_no_cursor_or_buttons(inactive_cycle);
            }
            require_no_scroll(harness.step(one_finger(0, 10), two_finger(0, 10), 10000));
            const LogicalActions cursor = harness.step(inactive(), one_finger(10, 0), 10000);
            require_no_scroll(cursor);
            require(cursor.cursor_x == 40 && cursor.left_button == ButtonAction::None,
                "one-finger cursor must continue while latched");
            require_no_scroll(harness.step(inactive(), inactive(), 10000));
            require_no_scroll(harness.step(inactive(), one_finger(), 10000));
            Tps43Sample tap;
            tap.single_tap = true;
            const LogicalActions drop = harness.step(inactive(), tap, 10000);
            require_no_scroll(drop);
            require(drop.left_button == ButtonAction::Release && drop.right_button == ButtonAction::None,
                "distinct tap must drop without an extra click");
            const LogicalActions after_drop = harness.step(inactive(), inactive(), 10000);
            require_no_scroll(after_drop);
            require_no_cursor_or_buttons(after_drop);

            // Clearing drag history must not disable a later ordinary scroll.
            harness.step(inactive(), two_finger(), 10000);
            require(harness.step(inactive(), two_finger(0, 20), 10000).scroll_y > 0,
                "fresh scrolling after drop must remain usable");
            harness.step(inactive(), inactive(), 10000);
            require(harness.step(inactive(), inactive(), 10000).scroll_y > 0,
                "fresh ordinary release must still launch momentum");
        }
    });

    run_case("MOTION-12 scroll-history-cannot-escape-left-assisted-drag", [] {
        for (uint16_t weight : { 128, 256 }) {
            DualTps43Tuning tuning = motion_tuning();
            tuning.scroll_momentum.release_velocity_filter_weight_q8 = weight;
            tuning.scroll_momentum.stop_velocity_logical_units_per_second = 0;
            Harness harness(tuning);
            harness.step(one_finger(), inactive(), 10000);
            harness.step(one_finger(), inactive(), 200000);
            require(harness.step(one_finger(), two_finger(10, 20), 10000).scroll_y > 0,
                "setup must establish Right scroll history against stationary Left");

            // No new touch occurs at entry: changing finger count must discard
            // launch history even when a partial filter would retain velocity.
            const LogicalActions entry = harness.step(one_finger(), one_finger(10, 0), 10000);
            require_no_scroll(entry);
            require(entry.left_button == ButtonAction::Press && entry.cursor_x == 40,
                "Left-assisted entry must press and retain scaled cursor movement");
            for (int cycle = 0; cycle < 5; cycle++) {
                const LogicalActions lifted = harness.step(one_finger(), inactive(), 10000);
                require_no_scroll(lifted);
                require_no_cursor_or_buttons(lifted);
            }
            const LogicalActions retouch = harness.step(one_finger(), one_finger(10, 0), 10000);
            require_no_scroll(retouch);
            require(retouch.cursor_x == 40 && retouch.left_button == ButtonAction::None,
                "Right retouch must continue cursor movement without another press");
            require_no_scroll(harness.step(one_finger(0, 10), two_finger(0, 20), 10000));
            require_no_scroll(harness.step(one_finger(), inactive(), 10000));
            const LogicalActions drop = harness.step(inactive(), inactive(), 10000);
            require_no_scroll(drop);
            require(drop.left_button == ButtonAction::Release && drop.right_button == ButtonAction::None,
                "Left release must drop without an extra click");
            const LogicalActions after_drop = harness.step(inactive(), inactive(), 10000);
            require_no_scroll(after_drop);
            require_no_cursor_or_buttons(after_drop);

            harness.step(inactive(), two_finger(), 10000);
            require(harness.step(inactive(), two_finger(0, 20), 10000).scroll_y > 0,
                "fresh scrolling after drop must remain usable");
            harness.step(inactive(), inactive(), 10000);
            require(harness.step(inactive(), inactive(), 10000).scroll_y > 0,
                "fresh scrolling after drop must still launch momentum");
        }
    });

    std::cout << "Motion result: " << passes << " passed, " << failures << " failed\n";
    return failures == 0 ? 0 : 1;
}
