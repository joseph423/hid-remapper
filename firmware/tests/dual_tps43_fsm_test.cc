#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "dual_tps43_fsm.h"
#include "pad_state.h"

// Each named PAD case uses dual-tps43-control-spec.md as its expected-outcome
// source. ROADMAP.md defines the required case inventory but not the behavior.

namespace {

DualTps43Tuning phase5_behavior_tuning() {
    DualTps43Tuning tuning;
    tuning.tap_max_duration_us = 2000;
    tuning.stationary_intent_threshold_us = 200;
    tuning.neutral_activation_threshold = 5;
    tuning.cursor_base_scale_q8 = 512;
    tuning.scroll_base_scale_q8 = 768;
    tuning.scroll_momentum = { false, 50, 100 };
    tuning.left_assisted_drag_axis_threshold = 2;
    return tuning;
}

DualTps43Tuning production_behavior_tuning() {
    DualTps43Tuning tuning;
    tuning.tap_max_duration_us = 200000;
    tuning.stationary_intent_threshold_us = 100000;
    tuning.neutral_activation_threshold = 20;
    tuning.cursor_base_scale_q8 = 128;
    tuning.scroll_base_scale_q8 = 4;
    tuning.scroll_momentum = { false, 50, 100 };
    tuning.left_assisted_drag_axis_threshold = 2;
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

Tps43Sample one_finger_with_movement(int32_t x, int32_t y, bool movement_reported) {
    Tps43Sample sample = one_finger(x, y);
    sample.movement_reported = movement_reported;
    return sample;
}

Tps43Sample two_finger_move(int32_t x, int32_t y) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 2;
    sample.relative_x = x;
    sample.relative_y = y;
    sample.movement_reported = true;
    sample.scroll_gesture = true;
    return sample;
}

Tps43Sample single_tap_release() {
    Tps43Sample sample;
    sample.single_tap = true;
    return sample;
}

Tps43Sample two_finger_tap_release() {
    Tps43Sample sample;
    sample.two_finger_tap = true;
    return sample;
}

Tps43Sample three_finger(int32_t offset_x, int32_t offset_y, bool movement) {
    Tps43Sample sample;
    sample.active = true;
    sample.finger_count = 3;
    sample.movement_reported = movement;
    sample.contact_details_available = true;
    sample.contacts[0] = { true, static_cast<uint16_t>(10 + offset_x), static_cast<uint16_t>(20 + offset_y), 10, 1 };
    sample.contacts[2] = { true, static_cast<uint16_t>(30 + offset_x), static_cast<uint16_t>(40 + offset_y), 20, 2 };
    sample.contacts[4] = { true, static_cast<uint16_t>(50 + offset_x), static_cast<uint16_t>(60 + offset_y), 30, 3 };
    return sample;
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_no_action(const LogicalActions& actions) {
    require(actions.cursor_x == 0 && actions.cursor_y == 0, "unexpected cursor movement");
    require(actions.scroll_x == 0 && actions.scroll_y == 0, "unexpected scroll movement");
    require(actions.left_button == ButtonAction::None, "unexpected left-button action");
    require(actions.right_button == ButtonAction::None, "unexpected right-button action");
}

void require_no_button_action(const LogicalActions& actions) {
    require(actions.left_button == ButtonAction::None && actions.right_button == ButtonAction::None,
        "unexpected button action");
}

// Drives the public FSM boundary with normalized samples and controlled time.
class Harness {
   public:
    explicit Harness(DualTps43Tuning tuning = phase5_behavior_tuning())
        : fsm_(tuning) {
    }

    // Advances logical time, normalizes both samples, and returns one cycle's
    // logical actions.
    LogicalActions step(Tps43Sample left, Tps43Sample right, uint64_t advance_us = 100) {
        now_us_ += advance_us;
        left.timestamp_us = now_us_;
        right.timestamp_us = now_us_;

        DualPadSnapshot snapshot;
        snapshot.left = left_tracker_.update(left);
        snapshot.right = right_tracker_.update(right);
        snapshot.cycle_timestamp_us = now_us_;
        return fsm_.process(snapshot);
    }

    // Reinitializes both input normalization and gesture policy as the runtime
    // configuration path does when a new tuning profile becomes active.
    void reset_with_tuning(DualTps43Tuning tuning) {
        left_tracker_.reset();
        right_tracker_.reset();
        fsm_.set_tuning(tuning);
    }

   private:
    uint64_t now_us_ = 0;
    PadStateTracker left_tracker_;
    PadStateTracker right_tracker_;
    DualTps43Fsm fsm_ = DualTps43Fsm(phase5_behavior_tuning());
};

void enter_left_scroll(Harness& harness) {
    const LogicalActions actions = harness.step(one_finger(0, 2), inactive());
    require(actions.scroll_y == 6, "Left-scroll entry must include the triggering movement");
}

void enter_left_assisted_drag(Harness& harness) {
    require_no_action(harness.step(one_finger(), inactive()));
    require_no_action(harness.step(one_finger(), inactive(), 200));
    const LogicalActions actions = harness.step(one_finger(), one_finger(2, 1));
    require(actions.left_button == ButtonAction::Press, "Left-assisted Drag must press the Left button");
    require(actions.cursor_x == 4 && actions.cursor_y == 2, "Drag entry must retain current Right movement");
}

void enter_right_latched_drag(Harness& harness) {
    require_no_action(harness.step(inactive(), three_finger(0, 0, false)));
    const LogicalActions actions = harness.step(inactive(), three_finger(2, 1, true));
    require(actions.left_button == ButtonAction::Press, "Right-latched Drag must press the Left button");
    require(actions.cursor_x == 4 && actions.cursor_y == 2, "Right-latched entry must use the centroid delta");
}

int failures = 0;
int passes = 0;

void run_case(const char* id, const std::function<void()>& test) {
    try {
        test();
        std::cout << "PASS " << id << '\n';
        passes++;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << id << ": " << error.what() << '\n';
        failures++;
    }
}

}  // namespace

int main() {
    run_case("PAD-01", [] {
        Harness harness;
        const LogicalActions actions = harness.step(inactive(), one_finger(3, -2));
        require(actions.cursor_x == 6 && actions.cursor_y == -4, "Right 1-finger movement must move the cursor");
    });

    run_case("PAD-02", [] {
        Harness harness;
        require_no_action(harness.step(inactive(), one_finger()));
        const LogicalActions actions = harness.step(inactive(), single_tap_release());
        require(actions.left_button == ButtonAction::Click, "Right 1-finger tap must Left-click");
    });

    run_case("PAD-02A", [] {
        Harness harness;
        for (int i = 0; i < 2; i++) {
            require_no_action(harness.step(inactive(), one_finger()));
            const LogicalActions actions = harness.step(inactive(), single_tap_release());
            require(actions.left_button == ButtonAction::Click, "each distinct Right tap session must Left-click");
        }
    });

    run_case("PAD-03", [] {
        Harness harness;
        const LogicalActions actions = harness.step(inactive(), two_finger_move(-2, 3));
        require(actions.scroll_x == -6 && actions.scroll_y == 9, "Right 2-finger movement must scroll");
    });

    run_case("PAD-04", [] {
        Harness harness;
        require_no_action(harness.step(inactive(), Tps43Sample{ true, 2 }));
        const LogicalActions actions = harness.step(inactive(), two_finger_tap_release());
        require(actions.right_button == ButtonAction::Click, "Right 2-finger tap must Right-click");
    });

    run_case("PAD-05", [] {
        Harness inactive_right;
        require(inactive_right.step(one_finger(1, 2), inactive()).scroll_y == 6,
            "Left movement must scroll while Right is inactive");

        Harness stationary_right;
        require_no_action(stationary_right.step(inactive(), one_finger()));
        require_no_action(stationary_right.step(inactive(), one_finger(), 200));
        require(stationary_right.step(one_finger(2, -1), one_finger()).scroll_x == 6,
            "Left movement must scroll while Right is stationary");
    });

    run_case("PAD-06", [] {
        Harness harness;
        require_no_action(harness.step(inactive(), one_finger()));
        require_no_action(harness.step(inactive(), one_finger(), 200));
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions click = harness.step(single_tap_release(), one_finger());
        require(click.left_button == ButtonAction::Click, "Left tap after stationary Right must Left-click");
        require_no_action(harness.step(inactive(), single_tap_release()));
    });

    run_case("PAD-07", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), inactive()));
        require_no_action(harness.step(one_finger(), inactive(), 200));
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions click = harness.step(one_finger(), single_tap_release());
        require(click.right_button == ButtonAction::Click, "Right tap after stationary Left must Right-click");
        require_no_action(harness.step(single_tap_release(), inactive()));
    });

    run_case("PAD-08", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), inactive()));
        require_no_action(harness.step(one_finger(), inactive(), 200));
        const LogicalActions actions = harness.step(one_finger(), one_finger(2, -1));
        require(actions.left_button == ButtonAction::Press, "Right movement after stationary Left must start Drag");
        require(actions.cursor_x == 4 && actions.cursor_y == -2, "Drag entry must move the cursor");
    });

    run_case("PAD-09", [] {
        Harness harness;
        require(harness.step(inactive(), one_finger(1, 0)).cursor_x == 2, "setup Right movement missing");
        require_no_action(harness.step(one_finger(), one_finger()));
        require_no_action(harness.step(single_tap_release(), one_finger()));
        require_no_action(harness.step(inactive(), single_tap_release()));
    });

    run_case("PAD-10", [] {
        Harness harness;
        require(harness.step(inactive(), one_finger(2, 0)).cursor_x == 4, "setup Right movement missing");
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions actions = harness.step(one_finger(), one_finger(), 200);
        require(actions.left_button == ButtonAction::Press, "stationary Left after moving Right must start Drag");
    });

    run_case("PAD-10A", [] {
        Harness harness;
        require(harness.step(inactive(), one_finger(1, 0)).cursor_x == 2, "setup Right movement missing");
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions actions = harness.step(one_finger(0, 2), one_finger(3, 0));
        require(actions.scroll_y == 6 && actions.cursor_x == 6, "Left movement must select scroll with concurrent cursor");
        require(actions.left_button == ButtonAction::None, "Left movement must not start Drag");
    });

    run_case("CHAR-01 one-unit Right movement does not activate delayed Left-assisted Drag", [] {
        Harness harness(production_behavior_tuning());
        require_no_action(harness.step(inactive(), one_finger(1, 0)));
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions actions = harness.step(one_finger(), one_finger(), 100000);
        require_no_button_action(actions);
    });

    run_case("CHAR-02 repeated one-unit Right movement does not accumulate into Drag", [] {
        Harness harness(production_behavior_tuning());
        require_no_action(harness.step(inactive(), one_finger(1, 0)));
        require_no_button_action(harness.step(inactive(), one_finger(1, 0)));
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions actions = harness.step(one_finger(), one_finger(), 100000);
        require_no_button_action(actions);
    });

    run_case("CHAR-03 zero-delta Right movement flag does not activate Left-assisted Drag", [] {
        Harness harness(production_behavior_tuning());
        require_no_action(harness.step(inactive(), one_finger_with_movement(0, 0, true)));
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions actions = harness.step(one_finger(), one_finger(), 100000);
        require_no_action(actions);
    });

    run_case("CHAR-04 two-unit Right movement activates delayed Left-assisted Drag", [] {
        Harness harness(production_behavior_tuning());
        require_no_button_action(harness.step(inactive(), one_finger(2, 0)));
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions actions = harness.step(one_finger(), one_finger(), 100000);
        require(actions.left_button == ButtonAction::Press && actions.right_button == ButtonAction::None,
            "two-unit Right movement must activate delayed Drag");
    });

    run_case("CHAR-05 one-unit Right three-finger movement still activates Right-latched Drag", [] {
        Harness harness(production_behavior_tuning());
        require_no_action(harness.step(inactive(), three_finger(0, 0, false)));
        const LogicalActions actions = harness.step(inactive(), three_finger(1, 0, true));
        require(actions.left_button == ButtonAction::Press,
            "one-unit three-finger movement must characterize the current latched Drag activation");
    });

    run_case("CHAR-06 one-unit Left movement activates Left-scroll", [] {
        Harness harness;
        const LogicalActions actions = harness.step(one_finger(0, 1), inactive());
        require(actions.left_button == ButtonAction::None && actions.scroll_y == 3,
            "one-unit Left movement must characterize the current Left-scroll entry");
    });

    run_case("CHAR-07 one-unit Right scroll movement emits Right scroll", [] {
        Harness harness;
        const LogicalActions actions = harness.step(inactive(), two_finger_move(1, 0));
        require(actions.right_button == ButtonAction::None && actions.scroll_x == 3,
            "one-unit Right scroll movement must characterize the current scroll path");
    });

    run_case("CHAR-08 one-unit neutral movement remains below neutral threshold", [] {
        Harness harness(production_behavior_tuning());
        require_no_action(harness.step(one_finger(), one_finger()));
        require_no_action(harness.step(one_finger(1, 0), one_finger()));
    });

    run_case("RUNTIME-01 tuning reset clears an active Drag mode", [] {
        Harness harness(production_behavior_tuning());
        require_no_action(harness.step(one_finger(), inactive()));
        require_no_action(harness.step(one_finger(), inactive(), 100000));
        require(harness.step(one_finger(), one_finger(2, 0)).left_button == ButtonAction::Press,
            "setup must enter Left-assisted Drag");

        harness.reset_with_tuning(production_behavior_tuning());
        require_no_action(harness.step(inactive(), inactive()));
        require_no_action(harness.step(one_finger(), one_finger()));
    });

    run_case("PAD-11", [] {
        Harness harness;
        enter_left_scroll(harness);
        const LogicalActions actions = harness.step(one_finger(0, 1), one_finger(2, -2));
        require(actions.scroll_y == 3 && actions.cursor_x == 4 && actions.cursor_y == -4,
            "Left-scroll mode must allow concurrent Right cursor movement");
    });

    run_case("PAD-12", [] {
        Harness harness;
        enter_left_scroll(harness);
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions click = harness.step(one_finger(), single_tap_release());
        require(click.left_button == ButtonAction::Click, "Right tap in Left-scroll mode must Left-click");
        require(harness.step(one_finger(0, 1), inactive()).scroll_y == 3, "Left-scroll mode must remain active after click");
    });

    run_case("PAD-12A Right tap predates Left-scroll", [] {
        Harness harness;
        require_no_action(harness.step(inactive(), one_finger()));                  // 100 us
        require_no_action(harness.step(inactive(), one_finger(), 200));             // 300 us
        const LogicalActions entry = harness.step(one_finger(0, 2), one_finger());  // 400 us
        require(entry.scroll_y == 6 && entry.left_button == ButtonAction::None &&
                    entry.right_button == ButtonAction::None && entry.cursor_x == 0 && entry.cursor_y == 0,
            "Left-scroll entry must scroll without consuming or clicking the held Right tap");
        const LogicalActions click = harness.step(one_finger(0, 1), single_tap_release());  // 500 us
        require(click.left_button == ButtonAction::Click && click.right_button == ButtonAction::None &&
                    click.scroll_x == 0 && click.scroll_y == 3 && click.cursor_x == 0 && click.cursor_y == 0,
            "eligible pre-existing Right tap must Left-click while Left continues scrolling");
        const LogicalActions continued = harness.step(one_finger(0, 1), inactive());
        require(continued.scroll_y == 3 && continued.left_button == ButtonAction::None &&
                    continued.right_button == ButtonAction::None,
            "Left scrolling must continue without another click");
        require_no_action(harness.step(single_tap_release(), inactive()));
        require_no_action(harness.step(inactive(), inactive()));
    });

    run_case("PAD-14A consumed Right survives Left-scroll entry", [] {
        // A prior cross-pad click or suppressed overlap must retain consumption
        // when the same held Right session later enters Left-scroll.
        for (bool suppressed : { false, true }) {
            Harness harness;
            if (suppressed) {
                require(harness.step(inactive(), one_finger(1, 0)).cursor_x == 2, "setup cursor missing");
            } else {
                require_no_action(harness.step(inactive(), one_finger()));
            }
            require_no_action(harness.step(inactive(), one_finger(), 200));
            require_no_action(harness.step(one_finger(), one_finger()));
            const LogicalActions prior = harness.step(single_tap_release(), one_finger());
            require(prior.left_button == (suppressed ? ButtonAction::None : ButtonAction::Click),
                "setup must distinguish suppressed overlap from cross-pad click");
            require(harness.step(one_finger(0, 2), one_finger()).scroll_y == 6, "Left-scroll entry missing");
            require_no_action(harness.step(one_finger(), single_tap_release()));
            require(harness.step(one_finger(0, 1), inactive()).scroll_y == 3, "consumption must preserve scrolling");
        }
        Harness two_fingers;
        require_no_action(two_fingers.step(inactive(), one_finger()));
        require_no_action(two_fingers.step(inactive(), one_finger(), 200));
        require(two_fingers.step(one_finger(0, 2), Tps43Sample{ true, 2 }).scroll_y == 6,
            "Left-scroll entry with Right two fingers must scroll");
        require_no_action(two_fingers.step(one_finger(), one_finger()));
        require_no_action(two_fingers.step(one_finger(), single_tap_release()));
    });

    run_case("PAD-13", [] {
        Harness harness;
        require_no_action(harness.step(inactive(), one_finger()));
        require_no_action(harness.step(inactive(), one_finger(), 200));
        for (int i = 0; i < 2; i++) {
            require_no_action(harness.step(one_finger(), one_finger()));
            const LogicalActions click = harness.step(single_tap_release(), one_finger());
            require(click.left_button == ButtonAction::Click, "each Left tap must click while Right remains held");
        }
        require_no_action(harness.step(inactive(), single_tap_release()));
    });

    run_case("PAD-14", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), one_finger()));
        require_no_action(harness.step(single_tap_release(), one_finger()));
        require_no_action(harness.step(inactive(), single_tap_release()));

        Harness selected_mode;
        require_no_action(selected_mode.step(one_finger(), one_finger()));
        require_no_action(selected_mode.step(one_finger(3, 4), one_finger()));
        require_no_action(selected_mode.step(one_finger(), single_tap_release()));
    });

    run_case("PAD-15", [] {
        Harness harness;
        enter_left_assisted_drag(harness);
        require_no_action(harness.step(one_finger(), inactive()));
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions actions = harness.step(one_finger(), one_finger(3, 1));
        require(actions.cursor_x == 6 && actions.cursor_y == 2, "Right retouch movement must continue Drag");
        require(actions.left_button == ButtonAction::None, "Right retouch must not press again");
    });

    run_case("PAD-15A", [] {
        Harness harness;
        enter_left_assisted_drag(harness);
        for (int i = 0; i < 2; i++) {
            require_no_action(harness.step(one_finger(), inactive()));
            require_no_action(harness.step(one_finger(), one_finger()));
            require(harness.step(one_finger(), one_finger(i + 1, 0)).cursor_x == (i + 1) * 2,
                "each Right lift/retouch cycle must continue Drag movement");
        }
    });

    run_case("PAD-16", [] {
        Harness harness;
        enter_left_assisted_drag(harness);
        const LogicalActions actions = harness.step(inactive(), one_finger());
        require(actions.left_button == ButtonAction::Release, "releasing Left must end Left-assisted Drag");
    });

    run_case("PAD-17", [] {
        Harness harness;
        enter_left_assisted_drag(harness);
        require_no_action(harness.step(one_finger(2, 2), one_finger()));
        require_no_action(harness.step(one_finger(), single_tap_release()));
        require_no_action(harness.step(one_finger(), two_finger_move(2, 2)));
        require_no_action(harness.step(one_finger(), two_finger_tap_release()));
        require(harness.step(one_finger(), one_finger(1, 0)).cursor_x == 2,
            "ignored gestures must not unlock Left-assisted Drag");
    });

    run_case("PAD-18", [] {
        Harness harness;
        enter_left_scroll(harness);
        require_no_action(harness.step(one_finger(), two_finger_move(2, 3)));
        require_no_action(harness.step(one_finger(), two_finger_tap_release()));
        require(harness.step(one_finger(0, 1), inactive()).scroll_y == 3,
            "Right 2-finger gesture must not unlock Left-scroll mode");
    });

    run_case("PAD-19", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(1, 1), one_finger(1, 1)));
        require_no_action(harness.step(one_finger(1, 0), one_finger(0, 1)));
    });

    run_case("PAD-20", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(3, 0), one_finger(0, 3)));
        require_no_action(harness.step(one_finger(-2, 0), one_finger(0, -2)));
        require_no_action(harness.step(one_finger(1, 0), one_finger(0, 1)));
    });

    run_case("PAD-21", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), one_finger()));
        const LogicalActions entry = harness.step(one_finger(), one_finger(3, 4));
        require(entry.left_button == ButtonAction::Press, "Right threshold must select Left-assisted Drag");
        require(entry.cursor_x == 0 && entry.cursor_y == 0, "threshold-crossing Right sample must be consumed");
        const LogicalActions movement = harness.step(one_finger(), one_finger(2, 1));
        require(movement.cursor_x == 4 && movement.cursor_y == 2, "cursor must begin on the next Right sample");
    });

    run_case("PAD-22", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), one_finger()));
        require_no_action(harness.step(one_finger(3, 4), one_finger()));
        const LogicalActions movement = harness.step(one_finger(1, 2), one_finger());
        require(movement.scroll_x == 3 && movement.scroll_y == 6, "scroll must begin after the threshold sample");
    });

    run_case("PAD-23", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), one_finger()));
        require_no_action(harness.step(one_finger(3, 4), one_finger(0, 5)));
        const LogicalActions actions = harness.step(one_finger(1, 0), one_finger(2, 0));
        require(actions.scroll_x == 3 && actions.cursor_x == 4, "same-cycle tie must select Left-scroll mode");
    });

    run_case("PAD-24", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(1, 0), one_finger(0, 1)));
        require_no_action(harness.step(single_tap_release(), single_tap_release()));
    });

    run_case("PAD-25", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), one_finger()));
        require_no_action(harness.step(inactive(), one_finger(2, 0)));
        const LogicalActions actions = harness.step(inactive(), one_finger(3, 0));
        require(actions.cursor_x == 6, "remaining Right pad must resume on a subsequent sample");
    });

    run_case("PAD-26", [] {
        Harness harness;
        require_no_action(harness.step(inactive(), three_finger(0, 0, false)));
        require_no_action(harness.step(inactive(), three_finger(0, 0, false), 300));
    });

    run_case("PAD-26A", [] {
        Harness harness;
        require_no_action(harness.step(inactive(), three_finger(0, 0, false)));
        require_no_action(harness.step(inactive(), inactive()));
    });

    run_case("PAD-27", [] {
        Harness harness;
        enter_right_latched_drag(harness);
    });

    run_case("PAD-28", [] {
        Harness harness;
        enter_right_latched_drag(harness);
        const LogicalActions actions = harness.step(inactive(), three_finger(5, 4, true));
        require(actions.cursor_x == 6 && actions.cursor_y == 6, "continued centroid movement must move the cursor");
        require(actions.left_button == ButtonAction::None, "continued movement must not press again");
    });

    run_case("PAD-29", [] {
        Harness harness;
        enter_right_latched_drag(harness);
        require_no_action(harness.step(inactive(), two_finger_move(2, 3)));
    });

    run_case("PAD-30", [] {
        Harness harness;
        enter_right_latched_drag(harness);
        require_no_action(harness.step(inactive(), Tps43Sample{ true, 2 }));
        const LogicalActions actions = harness.step(inactive(), one_finger(3, -1));
        require(actions.cursor_x == 6 && actions.cursor_y == -2, "1-finger movement must continue Right-latched Drag");
    });

    run_case("PAD-31", [] {
        Harness harness;
        enter_right_latched_drag(harness);
        require_no_action(harness.step(inactive(), one_finger()));
        require_no_action(harness.step(inactive(), inactive()));
        require_no_action(harness.step(inactive(), one_finger()));
        const LogicalActions drop = harness.step(inactive(), single_tap_release());
        require(drop.left_button == ButtonAction::Release, "only the later distinct tap may end the latch");
    });

    run_case("PAD-32", [] {
        Harness harness;
        enter_right_latched_drag(harness);
        require_no_action(harness.step(inactive(), inactive()));
        require_no_action(harness.step(inactive(), inactive()));
    });

    run_case("PAD-33", [] {
        Harness harness;
        enter_right_latched_drag(harness);
        require_no_action(harness.step(inactive(), inactive()));
        const LogicalActions actions = harness.step(inactive(), one_finger(4, 1));
        require(actions.cursor_x == 8 && actions.cursor_y == 2, "retouched 1-finger movement must continue latch");
    });

    run_case("PAD-34", [] {
        Harness harness;
        enter_right_latched_drag(harness);
        require_no_action(harness.step(inactive(), inactive()));
        require_no_action(harness.step(inactive(), one_finger()));
        const LogicalActions drop = harness.step(inactive(), single_tap_release());
        require(drop.left_button == ButtonAction::Release, "distinct 1-finger tap must drop the latch");
        require(drop.left_button != ButtonAction::Click, "drop tap must be consumed");
    });

    run_case("PAD-35", [] {
        Harness harness;
        enter_right_latched_drag(harness);

        // Exercise every otherwise-unassigned normalized event: stationary and
        // moving Left 1-, 2-, and 3-finger input; Left single/two-finger taps;
        // Right stationary 1-/2-finger input; Right 2-finger movement and tap;
        // and a Right 1-finger tap from the original session without the
        // required preceding inactive interval.
        require_no_action(harness.step(one_finger(), three_finger(2, 1, false)));
        require_no_action(harness.step(one_finger(2, 3), three_finger(2, 1, false)));
        require_no_action(harness.step(single_tap_release(), one_finger()));
        require_no_action(harness.step(Tps43Sample{ true, 2 }, single_tap_release()));
        require_no_action(harness.step(two_finger_move(1, 1), Tps43Sample{ true, 2 }));
        require_no_action(harness.step(two_finger_tap_release(), two_finger_move(2, 2)));
        require_no_action(harness.step(three_finger(0, 0, false), two_finger_tap_release()));
        require_no_action(harness.step(three_finger(1, 2, true), inactive()));
        require_no_action(harness.step(inactive(), inactive()));
    });

    run_case("PAD-36", [] {
        Harness left_scroll;
        enter_left_scroll(left_scroll);
        require_no_action(left_scroll.step(one_finger(), three_finger(0, 0, false)));
        require_no_action(left_scroll.step(one_finger(), three_finger(2, 1, true)));

        Harness left_drag;
        enter_left_assisted_drag(left_drag);
        require_no_action(left_drag.step(one_finger(), three_finger(0, 0, false)));
        require_no_action(left_drag.step(one_finger(), three_finger(2, 1, true)));
    });

    run_case("PAD-37 reverse-order stationary releases", [] {
        for (bool right_first : { false, true }) {
            for (bool reports_tap : { false, true }) {
                Harness harness;
                require_no_action(harness.step(right_first ? inactive() : one_finger(),
                    right_first ? one_finger() : inactive()));
                require_no_action(harness.step(right_first ? inactive() : one_finger(),
                    right_first ? one_finger() : inactive(), 200));
                require_no_action(harness.step(one_finger(), one_finger()));
                const Tps43Sample release = reports_tap ? single_tap_release() : inactive();
                require_no_action(harness.step(right_first ? one_finger() : release,
                    right_first ? release : one_finger(), 300));
                require_no_action(harness.step(right_first ? single_tap_release() : inactive(),
                    right_first ? inactive() : single_tap_release()));
            }
        }
    });

    run_case("PAD-38 remaining touch after reverse release", [] {
        for (int followup = 0; followup < 4; followup++) {
            Harness harness;
            require_no_action(harness.step(inactive(), one_finger()));
            require_no_action(harness.step(inactive(), one_finger(), 200));
            require_no_action(harness.step(one_finger(), one_finger()));
            require_no_action(harness.step(one_finger(), single_tap_release(), 300));
            if (followup == 0) {
                require_no_action(harness.step(one_finger(), one_finger()));
                const LogicalActions click = harness.step(one_finger(), single_tap_release());
                require(click.right_button == ButtonAction::Click && click.left_button == ButtonAction::None,
                    "a new Right tap after suppression must Right-click");
                require_no_action(harness.step(single_tap_release(), inactive()));
            } else if (followup == 1) {
                const LogicalActions drag = harness.step(one_finger(), one_finger(2, 1));
                require(drag.left_button == ButtonAction::Press && drag.cursor_x == 4 && drag.cursor_y == 2,
                    "remaining Left must support Left-assisted Drag");
                require(harness.step(inactive(), one_finger()).left_button == ButtonAction::Release,
                    "Left release must still drop the drag");
            } else {
                require(harness.step(one_finger(0, 2), inactive()).scroll_y == 6,
                    "remaining Left must still start scrolling");
                if (followup == 2) {
                    require_no_action(harness.step(one_finger(), one_finger()));
                    const LogicalActions click = harness.step(one_finger(), single_tap_release());
                    require(click.left_button == ButtonAction::Click && click.right_button == ButtonAction::None,
                        "Right tap during subsequent Left-scroll must Left-click");
                    require(harness.step(one_finger(0, 1), inactive()).scroll_y == 3,
                        "Left-scroll must remain active after the click");
                } else {
                    const LogicalActions both = harness.step(one_finger(0, 1), one_finger(2, 0));
                    require(both.scroll_y == 3 && both.cursor_x == 4 && both.left_button == ButtonAction::None,
                        "subsequent Left-scroll and Right cursor must coexist");
                }
            }
        }
        Harness mirror;
        require_no_action(mirror.step(one_finger(), inactive()));
        require_no_action(mirror.step(one_finger(), inactive(), 200));
        require_no_action(mirror.step(one_finger(), one_finger()));
        require_no_action(mirror.step(single_tap_release(), one_finger(), 300));
        for (int tap = 0; tap < 2; tap++) {
            require_no_action(mirror.step(one_finger(), one_finger()));
            require(mirror.step(single_tap_release(), one_finger()).left_button == ButtonAction::Click,
                "remaining Right must support repeated new Left taps");
        }
        require_no_action(mirror.step(inactive(), single_tap_release()));
    });

    run_case("PAD-39 reverse release before later stationary threshold", [] {
        Harness harness;
        require_no_action(harness.step(one_finger(), inactive()));
        require_no_action(harness.step(one_finger(), inactive(), 200));
        require_no_action(harness.step(one_finger(), one_finger()));
        require_no_action(harness.step(inactive(), one_finger(), 50));
        require_no_action(harness.step(inactive(), single_tap_release()));

        Harness movement;
        require_no_action(movement.step(one_finger(), inactive()));
        require_no_action(movement.step(one_finger(), inactive(), 200));
        require_no_action(movement.step(one_finger(), one_finger()));
        require_no_action(movement.step(single_tap_release(), one_finger(), 300));
        require(movement.step(inactive(), one_finger(2, 0)).cursor_x == 4,
            "consuming the remaining Right release must not disable cursor movement");
    });

    std::cout << "Phase 5 matrix result: " << passes << " passed, " << failures << " failed\n";
    return failures == 0 ? 0 : 1;
}
