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

bool validate_velocity_gain(const VelocityGainTuning& gain) {
    return gain.minimum_gain_q8 >= 0 && gain.maximum_gain_q8 >= gain.minimum_gain_q8 &&
           gain.full_gain_velocity_pad_units_per_second > 0 && gain.velocity_filter_weight_q8 <= 256 &&
           gain.fallback_sample_interval_us > 0;
}

bool validate_momentum(const ScrollMomentumTuning& momentum) {
    const bool disabled = momentum.release_velocity_filter_weight_q8 == 0 && momentum.launch_gain_q8 == 0 &&
                          momentum.decay_q8 == 0 && momentum.stop_velocity_logical_units_per_second == 0;
    if (disabled) {
        return true;
    }
    return momentum.release_velocity_filter_weight_q8 <= 256 && momentum.launch_gain_q8 <= 256 &&
           momentum.decay_q8 < 256 && momentum.stop_velocity_logical_units_per_second > 0;
}

DualTps43Tuning& configured_tuning_storage() {
    static DualTps43Tuning tuning = production_tuning();
    return tuning;
}

void encode_velocity_gain(uint8_t*& cursor, const VelocityGainTuning& gain) {
    write_i32(cursor, gain.minimum_gain_q8);
    cursor += 4;
    write_i32(cursor, gain.maximum_gain_q8);
    cursor += 4;
    write_u32(cursor, gain.full_gain_velocity_pad_units_per_second);
    cursor += 4;
    write_u16(cursor, gain.velocity_filter_weight_q8);
    cursor += 2;
    write_u32(cursor, gain.fallback_sample_interval_us);
    cursor += 4;
}

VelocityGainTuning decode_velocity_gain(const uint8_t*& cursor) {
    VelocityGainTuning gain;
    gain.minimum_gain_q8 = read_i32(cursor);
    cursor += 4;
    gain.maximum_gain_q8 = read_i32(cursor);
    cursor += 4;
    gain.full_gain_velocity_pad_units_per_second = read_u32(cursor);
    cursor += 4;
    gain.velocity_filter_weight_q8 = read_u16(cursor);
    cursor += 2;
    gain.fallback_sample_interval_us = read_u32(cursor);
    cursor += 4;
    return gain;
}

}  // namespace

DualTps43Tuning production_tuning() {
    DualTps43Tuning tuning;
    tuning.tap_max_duration_us = 200000;
    // Keep stationary intent below the tap limit so stationary-release cases
    // can also report an eligible tap.
    tuning.stationary_intent_threshold_us = 100000;
    tuning.neutral_activation_threshold = 20;
    // Fixed gains keep physical behavior deterministic while preserving usable
    // cursor and scroll output.
    tuning.cursor_gain = { 128, 128, 4000, 256, 15000 };
    tuning.scroll_gain = { 4, 4, 4000, 256, 15000 };
    tuning.scroll_momentum = { 0, 0, 0, 0 };
    tuning.left_assisted_drag_axis_threshold = 2;
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
           validate_velocity_gain(tuning.cursor_gain) && validate_velocity_gain(tuning.scroll_gain) &&
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
    encode_velocity_gain(cursor, tuning.cursor_gain);
    encode_velocity_gain(cursor, tuning.scroll_gain);
    write_u16(cursor, tuning.scroll_momentum.release_velocity_filter_weight_q8);
    cursor += 2;
    write_u16(cursor, tuning.scroll_momentum.launch_gain_q8);
    cursor += 2;
    write_u16(cursor, tuning.scroll_momentum.decay_q8);
    cursor += 2;
    write_u32(cursor, tuning.scroll_momentum.stop_velocity_logical_units_per_second);
    cursor += 4;
    write_i32(cursor, tuning.left_assisted_drag_axis_threshold);
    cursor += 4;
    return static_cast<std::size_t>(cursor - buffer) == kTps43TuningBlockSize;
}

bool decode_tps43_tuning(const uint8_t* buffer, std::size_t buffer_size, DualTps43Tuning* tuning) {
    if (buffer == nullptr || tuning == nullptr || buffer_size < kTps43TuningBlockSize ||
        read_u32(buffer) != kTps43TuningBlockMagic || buffer[4] != kTps43TuningBlockVersion ||
        read_u16(buffer + 6) != kTps43TuningBlockSize) {
        return false;
    }

    const uint8_t* cursor = buffer + kHeaderSize;
    DualTps43Tuning decoded;
    decoded.tap_max_duration_us = read_u64(cursor);
    cursor += 8;
    decoded.stationary_intent_threshold_us = read_u64(cursor);
    cursor += 8;
    decoded.neutral_activation_threshold = read_i32(cursor);
    cursor += 4;
    decoded.cursor_gain = decode_velocity_gain(cursor);
    decoded.scroll_gain = decode_velocity_gain(cursor);
    decoded.scroll_momentum.release_velocity_filter_weight_q8 = read_u16(cursor);
    cursor += 2;
    decoded.scroll_momentum.launch_gain_q8 = read_u16(cursor);
    cursor += 2;
    decoded.scroll_momentum.decay_q8 = read_u16(cursor);
    cursor += 2;
    decoded.scroll_momentum.stop_velocity_logical_units_per_second = read_u32(cursor);
    cursor += 4;
    decoded.left_assisted_drag_axis_threshold = read_i32(cursor);

    if (!validate_tps43_tuning(decoded)) {
        return false;
    }
    *tuning = decoded;
    return true;
}
