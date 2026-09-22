#include <cassert>
#include <cstdint>

#include "tps43_tuning_config.h"

namespace {

void assert_equal(const DualTps43Tuning& expected, const DualTps43Tuning& actual) {
    assert(expected.tap_max_duration_us == actual.tap_max_duration_us);
    assert(expected.stationary_intent_threshold_us == actual.stationary_intent_threshold_us);
    assert(expected.neutral_activation_threshold == actual.neutral_activation_threshold);
    assert(expected.left_assisted_drag_axis_threshold == actual.left_assisted_drag_axis_threshold);
    assert(expected.cursor_gain.minimum_gain_q8 == actual.cursor_gain.minimum_gain_q8);
    assert(expected.cursor_gain.maximum_gain_q8 == actual.cursor_gain.maximum_gain_q8);
    assert(expected.cursor_gain.full_gain_velocity_pad_units_per_second ==
        actual.cursor_gain.full_gain_velocity_pad_units_per_second);
    assert(expected.cursor_gain.velocity_filter_weight_q8 == actual.cursor_gain.velocity_filter_weight_q8);
    assert(expected.cursor_gain.fallback_sample_interval_us == actual.cursor_gain.fallback_sample_interval_us);
    assert(expected.scroll_gain.minimum_gain_q8 == actual.scroll_gain.minimum_gain_q8);
    assert(expected.scroll_gain.maximum_gain_q8 == actual.scroll_gain.maximum_gain_q8);
    assert(expected.scroll_gain.full_gain_velocity_pad_units_per_second ==
        actual.scroll_gain.full_gain_velocity_pad_units_per_second);
    assert(expected.scroll_gain.velocity_filter_weight_q8 == actual.scroll_gain.velocity_filter_weight_q8);
    assert(expected.scroll_gain.fallback_sample_interval_us == actual.scroll_gain.fallback_sample_interval_us);
    assert(expected.scroll_momentum.release_velocity_filter_weight_q8 ==
        actual.scroll_momentum.release_velocity_filter_weight_q8);
    assert(expected.scroll_momentum.launch_gain_q8 == actual.scroll_momentum.launch_gain_q8);
    assert(expected.scroll_momentum.decay_q8 == actual.scroll_momentum.decay_q8);
    assert(expected.scroll_momentum.stop_velocity_logical_units_per_second ==
        actual.scroll_momentum.stop_velocity_logical_units_per_second);
}

}  // namespace

int main() {
    const DualTps43Tuning defaults = production_tuning();
    assert(validate_tps43_tuning(defaults));

    uint8_t buffer[kTps43TuningBlockSize] = {};
    assert(encode_tps43_tuning(defaults, buffer, sizeof(buffer)));

    DualTps43Tuning decoded = {};
    assert(decode_tps43_tuning(buffer, sizeof(buffer), &decoded));
    assert_equal(defaults, decoded);

    uint8_t corrupted[kTps43TuningBlockSize] = {};
    for (std::size_t i = 0; i < sizeof(buffer); i++) {
        corrupted[i] = buffer[i];
    }
    corrupted[4]++;
    assert(!decode_tps43_tuning(corrupted, sizeof(corrupted), &decoded));
    assert(!decode_tps43_tuning(buffer, kTps43TuningBlockSize - 1, &decoded));

    DualTps43Tuning invalid = defaults;
    invalid.stationary_intent_threshold_us = invalid.tap_max_duration_us;
    assert(!validate_tps43_tuning(invalid));
    invalid = defaults;
    invalid.cursor_gain.velocity_filter_weight_q8 = 257;
    assert(!validate_tps43_tuning(invalid));
    invalid = defaults;
    invalid.scroll_momentum.decay_q8 = 256;
    invalid.scroll_momentum.stop_velocity_logical_units_per_second = 1;
    assert(!validate_tps43_tuning(invalid));
    assert(!encode_tps43_tuning(invalid, buffer, sizeof(buffer)));

    return 0;
}
