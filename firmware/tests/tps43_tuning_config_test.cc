#include <cassert>
#include <cstdint>

#include "tps43_tuning_config.h"

namespace {

void assert_equal(const DualTps43Tuning& expected, const DualTps43Tuning& actual) {
    assert(expected.tap_max_duration_us == actual.tap_max_duration_us);
    assert(expected.stationary_intent_threshold_us == actual.stationary_intent_threshold_us);
    assert(expected.neutral_activation_threshold == actual.neutral_activation_threshold);
    assert(expected.left_assisted_drag_axis_threshold == actual.left_assisted_drag_axis_threshold);
    assert(expected.cursor_base_scale_q8 == actual.cursor_base_scale_q8);
    assert(expected.scroll_base_scale_q8 == actual.scroll_base_scale_q8);
    assert(expected.scroll_momentum.enabled == actual.scroll_momentum.enabled);
    assert(expected.scroll_momentum.launch_strength_percent == actual.scroll_momentum.launch_strength_percent);
    assert(expected.scroll_momentum.half_life_ms == actual.scroll_momentum.half_life_ms);
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
    assert(expected.active_scroll_gain.enabled == actual.active_scroll_gain.enabled);
    assert(expected.active_scroll_gain.slow_speed_limit_counts_per_second ==
           actual.active_scroll_gain.slow_speed_limit_counts_per_second);
    assert(expected.active_scroll_gain.fast_speed_limit_counts_per_second ==
           actual.active_scroll_gain.fast_speed_limit_counts_per_second);
    assert(expected.active_scroll_gain.slow_gain_percent == actual.active_scroll_gain.slow_gain_percent);
    assert(expected.active_scroll_gain.fast_gain_percent == actual.active_scroll_gain.fast_gain_percent);
    assert(expected.scroll_direction_classification.enabled == actual.scroll_direction_classification.enabled);
    assert(expected.scroll_direction_classification.classification_distance_counts ==
           actual.scroll_direction_classification.classification_distance_counts);
    assert(expected.scroll_direction_classification.axis_dominance_ratio ==
           actual.scroll_direction_classification.axis_dominance_ratio);
    assert(expected.idle_timeout_before_lp1_seconds == actual.idle_timeout_before_lp1_seconds);
    assert(expected.lp1_timeout_before_lp2_20s_units == actual.lp1_timeout_before_lp2_20s_units);
}

}  // namespace

int main() {
    const DualTps43Tuning defaults = production_tuning();
    assert(validate_tps43_tuning(defaults));
    assert(defaults.subthreshold_cursor_threshold == 2);
    assert(defaults.cursor_temporal_filter_enabled);
    assert(defaults.scroll_direction_classification.enabled);
    assert(defaults.scroll_direction_classification.classification_distance_counts == 8);
    assert(defaults.scroll_direction_classification.axis_dominance_ratio == 2);
    assert(defaults.cursor_filter_slow_speed_limit_counts_per_second == 500);
    assert(defaults.cursor_filter_fast_speed_limit_counts_per_second == 2000);
    assert(defaults.cursor_filter_slow_weight_percent == 20);
    assert(defaults.cursor_filter_normal_weight_percent == 10);
    assert(defaults.cursor_filter_fast_weight_percent == 0);
    assert(!defaults.active_scroll_gain.enabled);
    assert(defaults.active_scroll_gain.slow_speed_limit_counts_per_second == 200);
    assert(defaults.active_scroll_gain.fast_speed_limit_counts_per_second == 1000);
    assert(defaults.active_scroll_gain.slow_gain_percent == 200);
    assert(defaults.active_scroll_gain.fast_gain_percent == 50);
    assert(defaults.idle_timeout_before_lp1_seconds == 10);
    assert(defaults.lp1_timeout_before_lp2_20s_units == 1);
    assert_equal(defaults, configured_tps43_tuning());

    DualTps43Tuning configured = defaults;
    configured.cursor_base_scale_q8 = 256;
    set_configured_tps43_tuning(configured);
    assert_equal(configured, configured_tps43_tuning());

    DualTps43Tuning invalid_configured = configured;
    invalid_configured.scroll_base_scale_q8 = -1;
    set_configured_tps43_tuning(invalid_configured);
    assert_equal(defaults, configured_tps43_tuning());

    uint8_t buffer[kTps43TuningBlockSize] = {};
    assert(encode_tps43_tuning(defaults, buffer, sizeof(buffer)));

    DualTps43Tuning decoded = {};
    assert(decode_tps43_tuning(buffer, sizeof(buffer), &decoded));
    assert_equal(defaults, decoded);

    DualTps43Tuning momentum_config = defaults;
    momentum_config.scroll_momentum = { true, 75, 150 };
    assert(encode_tps43_tuning(momentum_config, buffer, sizeof(buffer)));
    assert(decode_tps43_tuning(buffer, sizeof(buffer), &decoded));
    assert_equal(momentum_config, decoded);
    // The v8 layout has the same first 105 bytes; older momentum coefficients
    // are intentionally migrated to disabled elapsed-time momentum.
    uint8_t legacy_v8[kTps43TuningBlockV8Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v8); ++i) legacy_v8[i] = buffer[i];
    legacy_v8[4] = 8;
    legacy_v8[6] = static_cast<uint8_t>(sizeof(legacy_v8));
    legacy_v8[7] = 0;
    assert(decode_tps43_tuning(legacy_v8, sizeof(legacy_v8), &decoded));
    assert(!decoded.scroll_momentum.enabled);
    assert(decoded.scroll_momentum.launch_strength_percent == 50);
    assert(decoded.scroll_momentum.half_life_ms == 100);
    assert(decoded.cursor_base_scale_q8 == momentum_config.cursor_base_scale_q8);
    assert(encode_tps43_tuning(defaults, buffer, sizeof(buffer)));

    DualTps43Tuning scroll_gain_config = defaults;
    scroll_gain_config.active_scroll_gain = { true, 300, 1500, 250, 40 };
    uint8_t scroll_gain_block[kTps43TuningBlockSize] = {};
    assert(encode_tps43_tuning(scroll_gain_config, scroll_gain_block, sizeof(scroll_gain_block)));
    assert(decode_tps43_tuning(scroll_gain_block, sizeof(scroll_gain_block), &decoded));
    assert_equal(scroll_gain_config, decoded);

    DualTps43Tuning custom_power_timeouts = defaults;
    custom_power_timeouts.idle_timeout_before_lp1_seconds = 120;
    custom_power_timeouts.lp1_timeout_before_lp2_20s_units = 9;
    uint8_t custom_power_block[kTps43TuningBlockSize] = {};
    assert(encode_tps43_tuning(custom_power_timeouts, custom_power_block, sizeof(custom_power_block)));
    assert(decode_tps43_tuning(custom_power_block, sizeof(custom_power_block), &decoded));
    assert_equal(custom_power_timeouts, decoded);

    // Older blocks stored min/max velocity gains in these slots. Retain the
    // first value as the fixed scale and ignore the retired fields.
    uint8_t old_format_scale_block[kTps43TuningBlockSize] = {};
    for (std::size_t i = 0; i < sizeof(old_format_scale_block); i++) {
        old_format_scale_block[i] = buffer[i];
    }
    old_format_scale_block[28] = 192;
    old_format_scale_block[32] = 0;
    old_format_scale_block[33] = 4;
    old_format_scale_block[46] = 12;
    old_format_scale_block[50] = 0;
    old_format_scale_block[51] = 8;
    assert(decode_tps43_tuning(old_format_scale_block, sizeof(old_format_scale_block), &decoded));
    assert(decoded.cursor_base_scale_q8 == 192 && decoded.scroll_base_scale_q8 == 12);
    uint8_t canonical_block[kTps43TuningBlockSize] = {};
    assert(encode_tps43_tuning(decoded, canonical_block, sizeof(canonical_block)));
    assert(canonical_block[28] == 192 && canonical_block[32] == 192 &&
           canonical_block[46] == 12 && canonical_block[50] == 12);

    DualTps43Tuning saved_v21 = defaults;
    saved_v21.subthreshold_cursor_threshold = 171;
    saved_v21.cursor_base_scale_q8 = 192;
    migrate_tps43_tuning_v21(&saved_v21);
    assert(saved_v21.subthreshold_cursor_threshold == defaults.subthreshold_cursor_threshold);
    assert(saved_v21.cursor_base_scale_q8 == 192);

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
    assert(!decoded.active_scroll_gain.enabled);
    assert(decoded.active_scroll_gain.slow_gain_percent == 200);
    assert(decoded.cursor_filter_slow_speed_limit_counts_per_second == 500);
    assert(decoded.cursor_filter_fast_speed_limit_counts_per_second == 2000);

    uint8_t legacy_v5[kTps43TuningBlockV5Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v5); i++) {
        legacy_v5[i] = buffer[i];
    }
    legacy_v5[4] = 5;
    legacy_v5[6] = static_cast<uint8_t>(kTps43TuningBlockV5Size);
    legacy_v5[7] = static_cast<uint8_t>(kTps43TuningBlockV5Size >> 8);
    assert(decode_tps43_tuning(legacy_v5, sizeof(legacy_v5), &decoded));
    assert(!decoded.active_scroll_gain.enabled);
    assert(decoded.active_scroll_gain.slow_gain_percent == 200);
    assert(decoded.idle_timeout_before_lp1_seconds == defaults.idle_timeout_before_lp1_seconds);
    assert(decoded.lp1_timeout_before_lp2_20s_units == defaults.lp1_timeout_before_lp2_20s_units);

    uint8_t legacy_v6[kTps43TuningBlockV6Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v6); i++) {
        legacy_v6[i] = buffer[i];
    }
    legacy_v6[4] = kTps43TuningBlockV6Version;
    legacy_v6[6] = static_cast<uint8_t>(kTps43TuningBlockV6Size);
    legacy_v6[7] = static_cast<uint8_t>(kTps43TuningBlockV6Size >> 8);
    assert(decode_tps43_tuning(legacy_v6, sizeof(legacy_v6), &decoded));
    assert(decoded.idle_timeout_before_lp1_seconds == defaults.idle_timeout_before_lp1_seconds);
    assert(decoded.lp1_timeout_before_lp2_20s_units == defaults.lp1_timeout_before_lp2_20s_units);
    assert(decoded.scroll_direction_classification.enabled == defaults.scroll_direction_classification.enabled);

    uint8_t legacy_v7[kTps43TuningBlockV7Size] = {};
    for (std::size_t i = 0; i < sizeof(legacy_v7); i++) {
        legacy_v7[i] = buffer[i];
    }
    legacy_v7[4] = 7;
    legacy_v7[6] = static_cast<uint8_t>(kTps43TuningBlockV7Size);
    legacy_v7[7] = static_cast<uint8_t>(kTps43TuningBlockV7Size >> 8);
    assert(decode_tps43_tuning(legacy_v7, sizeof(legacy_v7), &decoded));
    assert(decoded.scroll_direction_classification.enabled == defaults.scroll_direction_classification.enabled);
    assert(decoded.scroll_direction_classification.classification_distance_counts == 8);
    assert(decoded.scroll_direction_classification.axis_dominance_ratio == 2);

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
    custom_threshold.scroll_direction_classification = { false, 12, 4 };
    assert(encode_tps43_tuning(custom_threshold, custom_block, sizeof(custom_block)));
    assert(decode_tps43_tuning(custom_block, sizeof(custom_block), &decoded));
    assert_equal(custom_threshold, decoded);

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
    invalid.scroll_direction_classification.axis_dominance_ratio = 1;
    assert(!validate_tps43_tuning(invalid));
    invalid = defaults;
    invalid.cursor_base_scale_q8 = -1;
    assert(!validate_tps43_tuning(invalid));
    invalid = defaults;
    invalid.scroll_momentum.half_life_ms = 0;
    assert(!validate_tps43_tuning(invalid));
    assert(!encode_tps43_tuning(invalid, buffer, sizeof(buffer)));
    invalid = defaults;
    invalid.cursor_filter_fast_speed_limit_counts_per_second =
        invalid.cursor_filter_slow_speed_limit_counts_per_second;
    assert(!validate_tps43_tuning(invalid));
    invalid = defaults;
    invalid.cursor_filter_slow_weight_percent = 101;
    assert(!validate_tps43_tuning(invalid));

    tps43_power_mode_timeouts_t power_timeouts = {120, 9};
    assert(set_configured_tps43_power_mode_timeouts(power_timeouts));
    tps43_power_mode_timeouts_t loaded_power_timeouts = {};
    assert(get_configured_tps43_power_mode_timeouts(&loaded_power_timeouts));
    assert(loaded_power_timeouts.idle_timeout_seconds == 120);
    assert(loaded_power_timeouts.lp1_timeout_20s_units == 9);
    power_timeouts.lp1_timeout_20s_units = 0;
    assert(!set_configured_tps43_power_mode_timeouts(power_timeouts));
    power_timeouts = {255, 9};
    assert(!set_configured_tps43_power_mode_timeouts(power_timeouts));

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
    assert(updated.cursor_base_scale_q8 == 192);
    assert(updated.scroll_base_scale_q8 == 8);
    assert(updated.subthreshold_cursor_expiry_us == 125000);
    assert(updated.subthreshold_cursor_threshold == 4);
    assert(!updated.scroll_momentum.enabled);

    tps43_scroll_momentum_tuning_t momentum_controls = {};
    assert(get_configured_tps43_scroll_momentum(&momentum_controls));
    assert(momentum_controls.enabled == 0 && momentum_controls.launch_strength_percent == 50 &&
           momentum_controls.half_life_ms == 100);
    assert(set_configured_tps43_scroll_momentum({ 1, 75, 150 }));
    assert(get_configured_tps43_scroll_momentum(&momentum_controls));
    assert(momentum_controls.enabled == 1 && momentum_controls.launch_strength_percent == 75 &&
           momentum_controls.half_life_ms == 150);
    assert(!set_configured_tps43_scroll_momentum({ 2, 50, 100 }));
    assert(!set_configured_tps43_scroll_momentum({ 1, 101, 100 }));
    assert(!set_configured_tps43_scroll_momentum({ 1, 50, 0 }));

    tps43_scroll_gain_tuning_t scroll_gain = {};
    assert(get_configured_tps43_scroll_gain(&scroll_gain));
    assert(scroll_gain.enabled == 0);
    assert(scroll_gain.slow_speed_limit_counts_per_second == 200);
    scroll_gain.enabled = 1;
    scroll_gain.slow_speed_limit_counts_per_second = 300;
    scroll_gain.fast_speed_limit_counts_per_second = 1500;
    scroll_gain.slow_gain_percent = 250;
    scroll_gain.fast_gain_percent = 40;
    assert(set_configured_tps43_scroll_gain(scroll_gain));
    tps43_scroll_gain_tuning_t saved_scroll_gain = {};
    assert(get_configured_tps43_scroll_gain(&saved_scroll_gain));
    assert(saved_scroll_gain.enabled == 1);
    assert(saved_scroll_gain.slow_speed_limit_counts_per_second == 300);
    assert(saved_scroll_gain.fast_speed_limit_counts_per_second == 1500);
    assert(saved_scroll_gain.slow_gain_percent == 250);
    assert(saved_scroll_gain.fast_gain_percent == 40);
    const DualTps43Tuning updated_scroll_gain = configured_tps43_tuning();
    assert(updated_scroll_gain.active_scroll_gain.enabled);
    assert(updated_scroll_gain.active_scroll_gain.slow_gain_percent == 250);
    assert(updated_scroll_gain.active_scroll_gain.fast_gain_percent == 40);
    assert(!set_configured_tps43_scroll_gain({ 1, 1500, 300, 100, 100 }));
    assert(!set_configured_tps43_scroll_gain({ 2, 100, 200, 100, 100 }));
    assert(!set_configured_tps43_scroll_gain({ 1, 100, 200, 301, 100 }));

    const DualTps43Tuning before_invalid = configured_tps43_tuning();
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

    tps43_scroll_direction_tuning_t direction_controls = {};
    assert(get_configured_tps43_scroll_direction(&direction_controls));
    assert(direction_controls.enabled == 1);
    assert(direction_controls.classification_distance_counts == 8);
    assert(direction_controls.axis_dominance_ratio == 2);
    direction_controls = { 0, 12, 4 };
    assert(set_configured_tps43_scroll_direction(direction_controls));
    tps43_scroll_direction_tuning_t saved_direction_controls = {};
    assert(get_configured_tps43_scroll_direction(&saved_direction_controls));
    assert(saved_direction_controls.enabled == 0);
    assert(saved_direction_controls.classification_distance_counts == 12);
    assert(saved_direction_controls.axis_dominance_ratio == 4);
    assert(!set_configured_tps43_scroll_direction({ 2, 8, 2 }));
    assert(!set_configured_tps43_scroll_direction({ 1, 0, 2 }));
    assert(!set_configured_tps43_scroll_direction({ 1, 8, 1 }));

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
