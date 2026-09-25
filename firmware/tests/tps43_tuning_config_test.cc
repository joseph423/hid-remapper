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
    assert(expected.subthreshold_cursor_expiry_us == actual.subthreshold_cursor_expiry_us);
    assert(expected.subthreshold_cursor_threshold == actual.subthreshold_cursor_threshold);
    assert(expected.cursor_temporal_filter_enabled == actual.cursor_temporal_filter_enabled);
    assert(expected.cursor_filter_slow_speed_limit_counts_per_second ==
           actual.cursor_filter_slow_speed_limit_counts_per_second);
    assert(expected.cursor_filter_fast_speed_limit_counts_per_second ==
           actual.cursor_filter_fast_speed_limit_counts_per_second);
    assert(expected.cursor_filter_slow_weight_percent == actual.cursor_filter_slow_weight_percent);
    assert(expected.cursor_filter_normal_weight_percent == actual.cursor_filter_normal_weight_percent);
    assert(expected.cursor_filter_fast_weight_percent == actual.cursor_filter_fast_weight_percent);
}

}  // namespace

int main() {
    const DualTps43Tuning defaults = production_tuning();
    assert(validate_tps43_tuning(defaults));
    assert(defaults.subthreshold_cursor_threshold == 2);
    assert(defaults.cursor_temporal_filter_enabled);
    assert(defaults.cursor_filter_slow_speed_limit_counts_per_second == 500);
    assert(defaults.cursor_filter_fast_speed_limit_counts_per_second == 2000);
    assert(defaults.cursor_filter_slow_weight_percent == 20);
    assert(defaults.cursor_filter_normal_weight_percent == 10);
    assert(defaults.cursor_filter_fast_weight_percent == 0);
    assert_equal(defaults, configured_tps43_tuning());

    DualTps43Tuning configured = defaults;
    configured.cursor_gain.minimum_gain_q8 = 256;
    configured.cursor_gain.maximum_gain_q8 = 256;
    set_configured_tps43_tuning(configured);
    assert_equal(configured, configured_tps43_tuning());

    DualTps43Tuning invalid_configured = configured;
    invalid_configured.scroll_gain.velocity_filter_weight_q8 = 257;
    set_configured_tps43_tuning(invalid_configured);
    assert_equal(defaults, configured_tps43_tuning());

    uint8_t buffer[kTps43TuningBlockSize] = {};
    assert(encode_tps43_tuning(defaults, buffer, sizeof(buffer)));

    DualTps43Tuning decoded = {};
    assert(decode_tps43_tuning(buffer, sizeof(buffer), &decoded));
    assert_equal(defaults, decoded);

    DualTps43Tuning saved_v21 = defaults;
    saved_v21.subthreshold_cursor_threshold = 171;
    saved_v21.cursor_gain.minimum_gain_q8 = 192;
    migrate_tps43_tuning_v21(&saved_v21);
    assert(saved_v21.subthreshold_cursor_threshold == defaults.subthreshold_cursor_threshold);
    assert(saved_v21.cursor_gain.minimum_gain_q8 == 192);

    uint8_t legacy_v1[kTps43TuningBlockV1Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v1); i++) {
        legacy_v1[i] = buffer[i];
    }
    legacy_v1[4] = 1;
    legacy_v1[6] = static_cast<uint8_t>(kTps43TuningBlockV1Size);
    legacy_v1[7] = static_cast<uint8_t>(kTps43TuningBlockV1Size >> 8);
    assert(decode_tps43_tuning(legacy_v1, sizeof(legacy_v1), &decoded));
    assert(decoded.subthreshold_cursor_expiry_us == defaults.subthreshold_cursor_expiry_us);
    assert(decoded.subthreshold_cursor_threshold == defaults.subthreshold_cursor_threshold);
    assert(decoded.cursor_temporal_filter_enabled);
    assert(decoded.cursor_filter_slow_weight_percent == 20);

    uint8_t legacy_v3[kTps43TuningBlockV3Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v3); i++) {
        legacy_v3[i] = buffer[i];
    }
    legacy_v3[4] = 3;
    legacy_v3[6] = static_cast<uint8_t>(kTps43TuningBlockV3Size);
    legacy_v3[7] = static_cast<uint8_t>(kTps43TuningBlockV3Size >> 8);
    assert(decode_tps43_tuning(legacy_v3, sizeof(legacy_v3), &decoded));
    assert(decoded.cursor_temporal_filter_enabled);
    assert(decoded.cursor_filter_slow_weight_percent == 20);

    uint8_t legacy_v4[kTps43TuningBlockV4Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v4); i++) {
        legacy_v4[i] = buffer[i];
    }
    legacy_v4[4] = 4;
    legacy_v4[6] = static_cast<uint8_t>(kTps43TuningBlockV4Size);
    legacy_v4[7] = static_cast<uint8_t>(kTps43TuningBlockV4Size >> 8);
    legacy_v4[kTps43TuningBlockV4Size - 1] = 50;
    assert(decode_tps43_tuning(legacy_v4, sizeof(legacy_v4), &decoded));
    assert(decoded.cursor_filter_slow_weight_percent == 50);
    assert(decoded.cursor_filter_normal_weight_percent == 25);
    assert(decoded.cursor_filter_fast_weight_percent == 0);
    assert(decoded.cursor_filter_slow_speed_limit_counts_per_second == 500);
    assert(decoded.cursor_filter_fast_speed_limit_counts_per_second == 2000);

    uint8_t legacy_v2[kTps43TuningBlockV2Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v2); i++) {
        legacy_v2[i] = buffer[i];
    }
    legacy_v2[4] = 2;
    legacy_v2[6] = static_cast<uint8_t>(kTps43TuningBlockV2Size);
    legacy_v2[7] = static_cast<uint8_t>(kTps43TuningBlockV2Size >> 8);
    assert(decode_tps43_tuning(legacy_v2, sizeof(legacy_v2), &decoded));
    assert(decoded.subthreshold_cursor_expiry_us == defaults.subthreshold_cursor_expiry_us);
    assert(decoded.subthreshold_cursor_threshold == defaults.subthreshold_cursor_threshold);

    DualTps43Tuning custom_threshold = defaults;
    custom_threshold.subthreshold_cursor_threshold = 5;
    uint8_t custom_block[kTps43TuningBlockSize] = {};
    assert(encode_tps43_tuning(custom_threshold, custom_block, sizeof(custom_block)));
    assert(decode_tps43_tuning(custom_block, sizeof(custom_block), &decoded));
    assert(decoded.subthreshold_cursor_threshold == 5);

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
    invalid = defaults;
    invalid.cursor_filter_fast_speed_limit_counts_per_second =
        invalid.cursor_filter_slow_speed_limit_counts_per_second;
    assert(!validate_tps43_tuning(invalid));
    invalid = defaults;
    invalid.cursor_filter_slow_weight_percent = 101;
    assert(!validate_tps43_tuning(invalid));

    tps43_runtime_tuning_set_t controls = {};
    tps43_runtime_tuning_t initial_controls = {};
    assert(get_configured_tps43_runtime_tuning(&initial_controls));
    controls.tap_max_duration_ms = initial_controls.tap_max_duration_ms;
    controls.stationary_intent_threshold_ms = initial_controls.stationary_intent_threshold_ms;
    controls.neutral_activation_threshold = initial_controls.neutral_activation_threshold;
    controls.left_assisted_drag_axis_threshold = initial_controls.left_assisted_drag_axis_threshold;
    controls.cursor_base_scale_q8 = initial_controls.cursor_base_scale_q8;
    controls.scroll_base_scale_q8 = initial_controls.scroll_base_scale_q8;
    controls.subthreshold_cursor_expiry_ms = initial_controls.subthreshold_cursor_expiry_ms;
    controls.tap_max_duration_ms = 250;
    controls.stationary_intent_threshold_ms = 125;
    controls.neutral_activation_threshold = 24;
    controls.left_assisted_drag_axis_threshold = 3;
    controls.cursor_base_scale_q8 = 192;
    controls.scroll_base_scale_q8 = 8;
    controls.subthreshold_cursor_expiry_ms = 125;
    assert(set_configured_tps43_runtime_tuning(controls));
    assert(set_configured_tps43_cursor_threshold(4));
    tps43_runtime_tuning_t round_trip_controls = {};
    assert(get_configured_tps43_runtime_tuning(&round_trip_controls));
    assert(round_trip_controls.subthreshold_cursor_threshold == 4);
    const DualTps43Tuning updated = configured_tps43_tuning();
    assert(updated.tap_max_duration_us == 250000);
    assert(updated.stationary_intent_threshold_us == 125000);
    assert(updated.neutral_activation_threshold == 24);
    assert(updated.left_assisted_drag_axis_threshold == 3);
    assert(updated.cursor_gain.minimum_gain_q8 == 192 && updated.cursor_gain.maximum_gain_q8 == 192);
    assert(updated.scroll_gain.minimum_gain_q8 == 8 && updated.scroll_gain.maximum_gain_q8 == 8);
    assert(updated.subthreshold_cursor_expiry_us == 125000);
    assert(updated.subthreshold_cursor_threshold == 4);
    assert(updated.scroll_momentum.release_velocity_filter_weight_q8 == 0);

    const DualTps43Tuning before_invalid = updated;
    controls.stationary_intent_threshold_ms = controls.tap_max_duration_ms;
    assert(!set_configured_tps43_runtime_tuning(controls));
    assert_equal(before_invalid, configured_tps43_tuning());

    controls.stationary_intent_threshold_ms = 125;
    controls.subthreshold_cursor_expiry_ms = 0;
    assert(!set_configured_tps43_runtime_tuning(controls));
    assert_equal(before_invalid, configured_tps43_tuning());

    controls.subthreshold_cursor_expiry_ms = 125;
    assert(!set_configured_tps43_cursor_threshold(0));
    assert_equal(before_invalid, configured_tps43_tuning());

    assert(set_configured_tps43_cursor_threshold(4));
    assert(set_configured_tps43_cursor_threshold(255));
    assert(configured_tps43_tuning().subthreshold_cursor_threshold == 255);

    tps43_cursor_filter_tuning_t filter_controls = {};
    assert(get_configured_tps43_cursor_filter(&filter_controls));
    assert(filter_controls.enabled == 1);
    assert(filter_controls.slow_speed_limit_counts_per_second == 500);
    assert(filter_controls.fast_speed_limit_counts_per_second == 2000);
    assert(filter_controls.slow_weight_percent == 20);
    assert(filter_controls.normal_weight_percent == 10);
    assert(filter_controls.fast_weight_percent == 0);
    filter_controls.enabled = 0;
    filter_controls.slow_speed_limit_counts_per_second = 150;
    filter_controls.fast_speed_limit_counts_per_second = 600;
    filter_controls.slow_weight_percent = 100;
    filter_controls.normal_weight_percent = 50;
    filter_controls.fast_weight_percent = 10;
    assert(set_configured_tps43_cursor_filter(filter_controls));
    tps43_cursor_filter_tuning_t filter_round_trip = {};
    assert(get_configured_tps43_cursor_filter(&filter_round_trip));
    assert(filter_round_trip.enabled == 0);
    assert(filter_round_trip.slow_speed_limit_counts_per_second == 150);
    assert(filter_round_trip.fast_speed_limit_counts_per_second == 600);
    assert(filter_round_trip.slow_weight_percent == 100);
    assert(filter_round_trip.normal_weight_percent == 50);
    assert(filter_round_trip.fast_weight_percent == 10);
    assert(!set_configured_tps43_cursor_filter({ 2, 80, 300, 20, 10, 0 }));
    assert(!set_configured_tps43_cursor_filter({ 1, 300, 100, 20, 10, 0 }));
    assert(!set_configured_tps43_cursor_filter({ 1, 80, 300, 101, 10, 0 }));

    return 0;
}
