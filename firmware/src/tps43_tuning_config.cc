#include "tps43_tuning_config.h"

namespace {

constexpr uint16_t kHeaderSize = 8;
void write_u16(uint8_t* buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8);
}

void write_u32(uint8_t* buffer, uint32_t value) {
    for (int i = 0; i < 4; i++) {
        buffer[i] = static_cast<uint8_t>(value >> (i * 8));
    }
}

void write_u64(uint8_t* buffer, uint64_t value) {
    for (int i = 0; i < 8; i++) {
        buffer[i] = static_cast<uint8_t>(value >> (i * 8));
    }
}

uint16_t read_u16(const uint8_t* buffer) {
    return static_cast<uint16_t>(buffer[0]) | static_cast<uint16_t>(buffer[1]) << 8;
}

uint32_t read_u32(const uint8_t* buffer) {
    uint32_t value = 0;
    for (int i = 0; i < 4; i++) {
        value |= static_cast<uint32_t>(buffer[i]) << (i * 8);
    }
    return value;
}

uint64_t read_u64(const uint8_t* buffer) {
    uint64_t value = 0;
    for (int i = 0; i < 8; i++) {
        value |= static_cast<uint64_t>(buffer[i]) << (i * 8);
    }
    return value;
}

void write_i32(uint8_t* buffer, int32_t value) {
    write_u32(buffer, static_cast<uint32_t>(value));
}

int32_t read_i32(const uint8_t* buffer) {
    return static_cast<int32_t>(read_u32(buffer));
}

bool validate_momentum(const ScrollMomentumTuning& momentum) {
    return momentum.launch_strength_percent <= 100 && momentum.half_life_ms >= 10 &&
           momentum.half_life_ms <= 1000;
}

DualTps43Tuning& configured_tuning_storage() {
    static DualTps43Tuning tuning = production_tuning();
    return tuning;
}

// Keep the v5 wire layout stable. The first historical gain field held the
// configurator's base scale; the remaining bytes are reserved compatibility slots.
void encode_scale_compat_record(uint8_t*& cursor, int32_t base_scale_q8) {
    write_i32(cursor, base_scale_q8);
    cursor += 4;
    write_i32(cursor, base_scale_q8);
    cursor += 4;
    write_u32(cursor, 4000);
    cursor += 4;
    write_u16(cursor, 256);
    cursor += 2;
    write_u32(cursor, 15000);
    cursor += 4;
}

int32_t decode_scale_compat_record(const uint8_t*& cursor) {
    const int32_t base_scale_q8 = read_i32(cursor);
    cursor += 18;
    return base_scale_q8;
}

}  // namespace

DualTps43Tuning production_tuning() {
    DualTps43Tuning tuning;
    tuning.tap_max_duration_us = 200000;
    // Keep stationary intent below the tap limit so stationary-release cases
    // can also report an eligible tap.
    tuning.stationary_intent_threshold_us = 100000;
    tuning.neutral_activation_threshold = 20;
    // Fixed base scales keep cursor and scroll output independent of speed.
    tuning.cursor_base_scale_q8 = 128;
    tuning.scroll_base_scale_q8 = 4;
    tuning.active_scroll_gain = { false, 200, 1000, 200, 50 };
    tuning.scroll_direction_classification = { true, 8, 2 };
    tuning.scroll_momentum = { false, 50, 100 };
    tuning.left_assisted_drag_axis_threshold = 2;
    tuning.subthreshold_cursor_expiry_us = 50000;
    tuning.subthreshold_cursor_threshold = 2;
    tuning.cursor_temporal_filter_enabled = true;
    tuning.cursor_filter_slow_speed_limit_counts_per_second = 500;
    tuning.cursor_filter_fast_speed_limit_counts_per_second = 2000;
    tuning.cursor_filter_slow_weight_percent = 20;
    tuning.cursor_filter_normal_weight_percent = 10;
    tuning.cursor_filter_fast_weight_percent = 0;
    tuning.idle_timeout_before_lp1_seconds = 10;
    tuning.lp1_timeout_before_lp2_20s_units = 1;
    return tuning;
}

DualTps43Tuning configured_tps43_tuning() {
    return configured_tuning_storage();
}

void set_configured_tps43_tuning(const DualTps43Tuning& tuning) {
    configured_tuning_storage() = validate_tps43_tuning(tuning) ? tuning : production_tuning();
}

bool validate_tps43_tuning(const DualTps43Tuning& tuning) {
    return tuning.tap_max_duration_us > 0 && tuning.stationary_intent_threshold_us > 0 &&
           tuning.stationary_intent_threshold_us < tuning.tap_max_duration_us &&
           tuning.neutral_activation_threshold >= 0 && tuning.left_assisted_drag_axis_threshold > 0 &&
           tuning.subthreshold_cursor_expiry_us >= 1000 && tuning.subthreshold_cursor_expiry_us <= 65535000 &&
           tuning.subthreshold_cursor_threshold >= 1 && tuning.subthreshold_cursor_threshold <= 255 &&
           tuning.cursor_filter_slow_speed_limit_counts_per_second > 0 &&
           tuning.cursor_filter_slow_speed_limit_counts_per_second <
               tuning.cursor_filter_fast_speed_limit_counts_per_second &&
           tuning.cursor_filter_fast_speed_limit_counts_per_second <= 10000 &&
           tuning.cursor_filter_slow_weight_percent <= 100 &&
           tuning.cursor_filter_normal_weight_percent <= 100 &&
           tuning.cursor_filter_fast_weight_percent <= 100 &&
           tuning.idle_timeout_before_lp1_seconds >= 1 &&
           tuning.idle_timeout_before_lp1_seconds <= 254 &&
           tuning.lp1_timeout_before_lp2_20s_units >= 1 &&
           tuning.lp1_timeout_before_lp2_20s_units <= 254 &&
           tuning.active_scroll_gain.slow_speed_limit_counts_per_second > 0 &&
           tuning.active_scroll_gain.slow_speed_limit_counts_per_second <
               tuning.active_scroll_gain.fast_speed_limit_counts_per_second &&
           tuning.active_scroll_gain.fast_speed_limit_counts_per_second <= 10000 &&
           tuning.active_scroll_gain.slow_gain_percent <= 300 &&
           tuning.active_scroll_gain.fast_gain_percent <= 300 &&
           tuning.scroll_direction_classification.classification_distance_counts >= 1 &&
           tuning.scroll_direction_classification.axis_dominance_ratio >= 2 &&
           tuning.cursor_base_scale_q8 >= 0 && tuning.scroll_base_scale_q8 >= 0 &&
           validate_momentum(tuning.scroll_momentum);
}

bool encode_tps43_tuning(const DualTps43Tuning& tuning, uint8_t* buffer, std::size_t buffer_size) {
    if (buffer == nullptr || buffer_size < kTps43TuningBlockSize || !validate_tps43_tuning(tuning)) {
        return false;
    }

    write_u32(buffer, kTps43TuningBlockMagic);
    buffer[4] = kTps43TuningBlockVersion;
    buffer[5] = 0;
    write_u16(buffer + 6, static_cast<uint16_t>(kTps43TuningBlockSize));

    uint8_t* cursor = buffer + kHeaderSize;
    write_u64(cursor, tuning.tap_max_duration_us);
    cursor += 8;
    write_u64(cursor, tuning.stationary_intent_threshold_us);
    cursor += 8;
    write_i32(cursor, tuning.neutral_activation_threshold);
    cursor += 4;
    encode_scale_compat_record(cursor, tuning.cursor_base_scale_q8);
    encode_scale_compat_record(cursor, tuning.scroll_base_scale_q8);
    // Retain the historical 10-byte slot so the rest of the persisted layout
    // stays stable; v9 settings live in the appended record.
    for (int i = 0; i < 10; ++i) *cursor++ = 0;
    write_i32(cursor, tuning.left_assisted_drag_axis_threshold);
    cursor += 4;
    write_u32(cursor, tuning.subthreshold_cursor_expiry_us);
    cursor += 4;
    *cursor++ = tuning.subthreshold_cursor_threshold;
    *cursor++ = tuning.cursor_temporal_filter_enabled ? 1 : 0;
    *cursor++ = tuning.cursor_filter_slow_weight_percent;
    write_u16(cursor, tuning.cursor_filter_slow_speed_limit_counts_per_second);
    cursor += 2;
    write_u16(cursor, tuning.cursor_filter_fast_speed_limit_counts_per_second);
    cursor += 2;
    *cursor++ = tuning.cursor_filter_normal_weight_percent;
    *cursor++ = tuning.cursor_filter_fast_weight_percent;
    *cursor++ = tuning.active_scroll_gain.enabled ? 1 : 0;
    write_u16(cursor, tuning.active_scroll_gain.slow_speed_limit_counts_per_second);
    cursor += 2;
    write_u16(cursor, tuning.active_scroll_gain.fast_speed_limit_counts_per_second);
    cursor += 2;
    write_u16(cursor, tuning.active_scroll_gain.slow_gain_percent);
    cursor += 2;
    write_u16(cursor, tuning.active_scroll_gain.fast_gain_percent);
    cursor += 2;
    *cursor++ = tuning.idle_timeout_before_lp1_seconds;
    *cursor++ = tuning.lp1_timeout_before_lp2_20s_units;
    *cursor++ = tuning.scroll_direction_classification.enabled ? 1 : 0;
    *cursor++ = tuning.scroll_direction_classification.classification_distance_counts;
    *cursor++ = tuning.scroll_direction_classification.axis_dominance_ratio;
    *cursor++ = tuning.scroll_momentum.enabled ? 1 : 0;
    write_u16(cursor, tuning.scroll_momentum.launch_strength_percent);
    cursor += 2;
    write_u16(cursor, tuning.scroll_momentum.half_life_ms);
    cursor += 2;
    return static_cast<std::size_t>(cursor - buffer) == kTps43TuningBlockSize;
}

bool decode_tps43_tuning(const uint8_t* buffer, std::size_t buffer_size, DualTps43Tuning* tuning) {
    if (buffer == nullptr || tuning == nullptr || buffer_size < kTps43TuningBlockV1Size ||
        read_u32(buffer) != kTps43TuningBlockMagic) {
        return false;
    }

    const uint8_t block_version = buffer[4];
    const uint16_t block_size = read_u16(buffer + 6);
    const bool legacy_v1 = block_version == 1 && block_size == kTps43TuningBlockV1Size &&
                           buffer_size >= kTps43TuningBlockV1Size;
    const bool legacy_v2 = block_version == 2 && block_size == kTps43TuningBlockV2Size &&
                           buffer_size >= kTps43TuningBlockV2Size;
    const bool legacy_v3 = block_version == 3 && block_size == kTps43TuningBlockV3Size &&
                           buffer_size >= kTps43TuningBlockV3Size;
    const bool legacy_v4 = block_version == 4 && block_size == kTps43TuningBlockV4Size &&
                           buffer_size >= kTps43TuningBlockV4Size;
    const bool legacy_v5 = block_version == 5 && block_size == kTps43TuningBlockV5Size &&
                           buffer_size >= kTps43TuningBlockV5Size;
    const bool legacy_v6 = block_version == kTps43TuningBlockV6Version &&
                           block_size == kTps43TuningBlockV6Size && buffer_size >= kTps43TuningBlockV6Size;
    const bool legacy_v7 = block_version == 7 && block_size == kTps43TuningBlockV7Size &&
                           buffer_size >= kTps43TuningBlockV7Size;
    const bool legacy_v8 = block_version == 8 && block_size == kTps43TuningBlockV8Size &&
                           buffer_size >= kTps43TuningBlockV8Size;
    const bool current_v9 = block_version == kTps43TuningBlockVersion && block_size == kTps43TuningBlockSize &&
                            buffer_size >= kTps43TuningBlockSize;
    if (!legacy_v1 && !legacy_v2 && !legacy_v3 && !legacy_v4 && !legacy_v5 && !legacy_v6 && !legacy_v7 && !legacy_v8 && !current_v9) {
        return false;
    }

    const uint8_t* cursor = buffer + kHeaderSize;
    DualTps43Tuning decoded = production_tuning();
    decoded.tap_max_duration_us = read_u64(cursor);
    cursor += 8;
    decoded.stationary_intent_threshold_us = read_u64(cursor);
    cursor += 8;
    decoded.neutral_activation_threshold = read_i32(cursor);
    cursor += 4;
    decoded.cursor_base_scale_q8 = decode_scale_compat_record(cursor);
    decoded.scroll_base_scale_q8 = decode_scale_compat_record(cursor);
    // Older experimental momentum values do not map to elapsed-time controls.
    // Migration leaves momentum disabled while preserving all other settings.
    cursor += 10;
    decoded.left_assisted_drag_axis_threshold = read_i32(cursor);
    cursor += 4;
    if (legacy_v2 || legacy_v3 || legacy_v4 || legacy_v5 || legacy_v6 || legacy_v7 || legacy_v8 || current_v9) {
        decoded.subthreshold_cursor_expiry_us = read_u32(cursor);
        cursor += 4;
    }
    if (legacy_v3 || legacy_v4 || legacy_v5 || legacy_v6 || legacy_v7 || legacy_v8 || current_v9) {
        decoded.subthreshold_cursor_threshold = *cursor;
        cursor++;
    }
    if (legacy_v4 || legacy_v5 || legacy_v6 || legacy_v7 || legacy_v8 || current_v9) {
        if (*cursor > 1) {
            return false;
        }
        decoded.cursor_temporal_filter_enabled = *cursor++ != 0;
        decoded.cursor_filter_slow_weight_percent = *cursor++;
        if (legacy_v5 || legacy_v6 || legacy_v7 || legacy_v8 || current_v9) {
            decoded.cursor_filter_slow_speed_limit_counts_per_second = read_u16(cursor);
            cursor += 2;
            decoded.cursor_filter_fast_speed_limit_counts_per_second = read_u16(cursor);
            cursor += 2;
            decoded.cursor_filter_normal_weight_percent = *cursor++;
            decoded.cursor_filter_fast_weight_percent = *cursor++;
        } else {
            // v4 used the slow-band weight for all speeds up to 300 counts/s,
            // halved it in its normal band, and bypassed the fast band.
            decoded.cursor_filter_normal_weight_percent =
                decoded.cursor_filter_slow_weight_percent / 2;
            decoded.cursor_filter_fast_weight_percent = 0;
        }
    }

    if (legacy_v6 || legacy_v7 || legacy_v8 || current_v9) {
        if (*cursor > 1) {
            return false;
        }
        decoded.active_scroll_gain.enabled = *cursor++ != 0;
        decoded.active_scroll_gain.slow_speed_limit_counts_per_second = read_u16(cursor);
        cursor += 2;
        decoded.active_scroll_gain.fast_speed_limit_counts_per_second = read_u16(cursor);
        cursor += 2;
        decoded.active_scroll_gain.slow_gain_percent = read_u16(cursor);
        cursor += 2;
        decoded.active_scroll_gain.fast_gain_percent = read_u16(cursor);
        cursor += 2;
    }
    if (legacy_v7 || legacy_v8 || current_v9) {
        decoded.idle_timeout_before_lp1_seconds = *cursor++;
        decoded.lp1_timeout_before_lp2_20s_units = *cursor++;
    }
    if (legacy_v8 || current_v9) {
        if (*cursor > 1) {
            return false;
        }
        decoded.scroll_direction_classification.enabled = *cursor++ != 0;
        decoded.scroll_direction_classification.classification_distance_counts = *cursor++;
        decoded.scroll_direction_classification.axis_dominance_ratio = *cursor++;
    }
    if (current_v9) {
        if (*cursor > 1) return false;
        decoded.scroll_momentum.enabled = *cursor++ != 0;
        decoded.scroll_momentum.launch_strength_percent = read_u16(cursor);
        cursor += 2;
        decoded.scroll_momentum.half_life_ms = read_u16(cursor);
        cursor += 2;
    }

    if (!validate_tps43_tuning(decoded)) {
        return false;
    }
    *tuning = decoded;
    return true;
}

void migrate_tps43_tuning_v21(DualTps43Tuning* tuning) {
    if (tuning != nullptr) {
        tuning->subthreshold_cursor_threshold = production_tuning().subthreshold_cursor_threshold;
    }
}

bool get_configured_tps43_runtime_tuning(tps43_runtime_tuning_t* controls) {
    if (controls == nullptr) {
        return false;
    }

    const DualTps43Tuning tuning = configured_tps43_tuning();
    controls->tap_max_duration_ms = static_cast<uint32_t>(tuning.tap_max_duration_us / 1000);
    controls->stationary_intent_threshold_ms = static_cast<uint32_t>(tuning.stationary_intent_threshold_us / 1000);
    controls->neutral_activation_threshold = tuning.neutral_activation_threshold;
    controls->left_assisted_drag_axis_threshold = tuning.left_assisted_drag_axis_threshold;
    controls->cursor_base_scale_q8 = tuning.cursor_base_scale_q8;
    controls->scroll_base_scale_q8 = tuning.scroll_base_scale_q8;
    controls->subthreshold_cursor_expiry_ms = static_cast<uint16_t>(tuning.subthreshold_cursor_expiry_us / 1000);
    controls->subthreshold_cursor_threshold = tuning.subthreshold_cursor_threshold;
    return true;
}

bool set_configured_tps43_runtime_tuning(const tps43_runtime_tuning_set_t& controls) {
    DualTps43Tuning candidate = configured_tps43_tuning();
    candidate.tap_max_duration_us = static_cast<uint64_t>(controls.tap_max_duration_ms) * 1000;
    candidate.stationary_intent_threshold_us = static_cast<uint64_t>(controls.stationary_intent_threshold_ms) * 1000;
    candidate.neutral_activation_threshold = controls.neutral_activation_threshold;
    candidate.left_assisted_drag_axis_threshold = controls.left_assisted_drag_axis_threshold;
    candidate.cursor_base_scale_q8 = controls.cursor_base_scale_q8;
    candidate.scroll_base_scale_q8 = controls.scroll_base_scale_q8;
    candidate.subthreshold_cursor_expiry_us = static_cast<uint32_t>(controls.subthreshold_cursor_expiry_ms) * 1000;
    if (!validate_tps43_tuning(candidate)) {
        return false;
    }

    set_configured_tps43_tuning(candidate);
    return true;
}

bool set_configured_tps43_cursor_threshold(uint8_t threshold) {
    DualTps43Tuning candidate = configured_tps43_tuning();
    candidate.subthreshold_cursor_threshold = threshold;
    if (!validate_tps43_tuning(candidate)) {
        return false;
    }

    set_configured_tps43_tuning(candidate);
    return true;
}

bool get_configured_tps43_cursor_filter(tps43_cursor_filter_tuning_t* controls) {
    if (controls == nullptr) {
        return false;
    }

    const DualTps43Tuning tuning = configured_tps43_tuning();
    controls->enabled = tuning.cursor_temporal_filter_enabled ? 1 : 0;
    controls->slow_speed_limit_counts_per_second = tuning.cursor_filter_slow_speed_limit_counts_per_second;
    controls->fast_speed_limit_counts_per_second = tuning.cursor_filter_fast_speed_limit_counts_per_second;
    controls->slow_weight_percent = tuning.cursor_filter_slow_weight_percent;
    controls->normal_weight_percent = tuning.cursor_filter_normal_weight_percent;
    controls->fast_weight_percent = tuning.cursor_filter_fast_weight_percent;
    return true;
}

bool set_configured_tps43_cursor_filter(const tps43_cursor_filter_tuning_t& controls) {
    if (controls.enabled > 1 || controls.slow_weight_percent > 100 ||
        controls.normal_weight_percent > 100 || controls.fast_weight_percent > 100) {
        return false;
    }

    DualTps43Tuning candidate = configured_tps43_tuning();
    candidate.cursor_temporal_filter_enabled = controls.enabled != 0;
    candidate.cursor_filter_slow_speed_limit_counts_per_second = controls.slow_speed_limit_counts_per_second;
    candidate.cursor_filter_fast_speed_limit_counts_per_second = controls.fast_speed_limit_counts_per_second;
    candidate.cursor_filter_slow_weight_percent = controls.slow_weight_percent;
    candidate.cursor_filter_normal_weight_percent = controls.normal_weight_percent;
    candidate.cursor_filter_fast_weight_percent = controls.fast_weight_percent;
    if (!validate_tps43_tuning(candidate)) {
        return false;
    }

    set_configured_tps43_tuning(candidate);
    return true;
}

bool get_configured_tps43_scroll_gain(tps43_scroll_gain_tuning_t* controls) {
    if (controls == nullptr) {
        return false;
    }

    const DualTps43Tuning configured = configured_tps43_tuning();
    const ActiveScrollGainTuning& tuning = configured.active_scroll_gain;
    controls->enabled = tuning.enabled ? 1 : 0;
    controls->slow_speed_limit_counts_per_second = tuning.slow_speed_limit_counts_per_second;
    controls->fast_speed_limit_counts_per_second = tuning.fast_speed_limit_counts_per_second;
    controls->slow_gain_percent = tuning.slow_gain_percent;
    controls->fast_gain_percent = tuning.fast_gain_percent;
    return true;
}

bool set_configured_tps43_scroll_gain(const tps43_scroll_gain_tuning_t& controls) {
    if (controls.enabled > 1) {
        return false;
    }

    DualTps43Tuning candidate = configured_tps43_tuning();
    candidate.active_scroll_gain.enabled = controls.enabled != 0;
    candidate.active_scroll_gain.slow_speed_limit_counts_per_second =
        controls.slow_speed_limit_counts_per_second;
    candidate.active_scroll_gain.fast_speed_limit_counts_per_second =
        controls.fast_speed_limit_counts_per_second;
    candidate.active_scroll_gain.slow_gain_percent = controls.slow_gain_percent;
    candidate.active_scroll_gain.fast_gain_percent = controls.fast_gain_percent;
    if (!validate_tps43_tuning(candidate)) {
        return false;
    }

    set_configured_tps43_tuning(candidate);
    return true;
}

bool get_configured_tps43_power_mode_timeouts(tps43_power_mode_timeouts_t* timeouts) {
    if (timeouts == nullptr) {
        return false;
    }

    const DualTps43Tuning tuning = configured_tps43_tuning();
    timeouts->idle_timeout_seconds = tuning.idle_timeout_before_lp1_seconds;
    timeouts->lp1_timeout_20s_units = tuning.lp1_timeout_before_lp2_20s_units;
    return true;
}

bool set_configured_tps43_power_mode_timeouts(const tps43_power_mode_timeouts_t& timeouts) {
    DualTps43Tuning candidate = configured_tps43_tuning();
    candidate.idle_timeout_before_lp1_seconds = timeouts.idle_timeout_seconds;
    candidate.lp1_timeout_before_lp2_20s_units = timeouts.lp1_timeout_20s_units;
    if (!validate_tps43_tuning(candidate)) {
        return false;
    }

    set_configured_tps43_tuning(candidate);
    return true;
}

bool get_configured_tps43_scroll_direction(tps43_scroll_direction_tuning_t* controls) {
    if (controls == nullptr) {
        return false;
    }
    const ScrollDirectionClassificationTuning& tuning = configured_tps43_tuning().scroll_direction_classification;
    controls->enabled = tuning.enabled ? 1 : 0;
    controls->classification_distance_counts = tuning.classification_distance_counts;
    controls->axis_dominance_ratio = tuning.axis_dominance_ratio;
    return true;
}

bool set_configured_tps43_scroll_direction(const tps43_scroll_direction_tuning_t& controls) {
    if (controls.enabled > 1) {
        return false;
    }
    DualTps43Tuning candidate = configured_tps43_tuning();
    candidate.scroll_direction_classification.enabled = controls.enabled != 0;
    candidate.scroll_direction_classification.classification_distance_counts =
        controls.classification_distance_counts;
    candidate.scroll_direction_classification.axis_dominance_ratio = controls.axis_dominance_ratio;
    if (!validate_tps43_tuning(candidate)) {
        return false;
    }
    set_configured_tps43_tuning(candidate);
    return true;
}

bool get_configured_tps43_scroll_momentum(tps43_scroll_momentum_tuning_t* controls) {
    if (controls == nullptr) return false;
    const auto momentum = configured_tps43_tuning().scroll_momentum;
    controls->enabled = momentum.enabled ? 1 : 0;
    controls->launch_strength_percent = momentum.launch_strength_percent;
    controls->half_life_ms = momentum.half_life_ms;
    return true;
}

bool set_configured_tps43_scroll_momentum(const tps43_scroll_momentum_tuning_t& controls) {
    if (controls.enabled > 1) return false;
    auto candidate = configured_tps43_tuning();
    candidate.scroll_momentum = { controls.enabled != 0, controls.launch_strength_percent, controls.half_life_ms };
    if (!validate_tps43_tuning(candidate)) return false;
    set_configured_tps43_tuning(candidate);
    return true;
}
