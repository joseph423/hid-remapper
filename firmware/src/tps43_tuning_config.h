#ifndef TPS43_TUNING_CONFIG_H_
#define TPS43_TUNING_CONFIG_H_

#include <cstddef>
#include <cstdint>

#include "dual_tps43_fsm.h"
#include "types.h"

// The block is serialized field-by-field in little-endian order. It must not
// depend on C++ structure padding or compiler ABI details.
constexpr uint32_t kTps43TuningBlockMagic = 0x54343354;
constexpr uint8_t kTps43TuningBlockVersion = 5;
constexpr std::size_t kTps43TuningBlockV1Size = 78;
constexpr std::size_t kTps43TuningBlockV2Size = 82;
constexpr std::size_t kTps43TuningBlockV3Size = 83;
constexpr std::size_t kTps43TuningBlockV4Size = 85;
constexpr std::size_t kTps43TuningBlockSize = 91;

// Returns the approved production defaults used by the current physical
// profile. The returned value is independent of persisted configuration.
DualTps43Tuning production_tuning();

// Returns the validated tuning loaded from persistence, or the production
// defaults before a persisted configuration has been loaded.
DualTps43Tuning configured_tps43_tuning();

// Replaces the persisted tuning candidate only when it is valid. Invalid input
// falls back to the production defaults and is never retained.
void set_configured_tps43_tuning(const DualTps43Tuning& tuning);

// Validates field relationships and coefficient ranges without applying the
// tuning to an active FSM.
bool validate_tps43_tuning(const DualTps43Tuning& tuning);

// Encodes a validated tuning block. Returns false for an invalid tuning or an
// insufficient output buffer.
bool encode_tps43_tuning(const DualTps43Tuning& tuning, uint8_t* buffer, std::size_t buffer_size);

// Decodes and validates one complete tuning block. The destination is not
// modified when the header, size, or tuning values are invalid.
bool decode_tps43_tuning(const uint8_t* buffer, std::size_t buffer_size, DualTps43Tuning* tuning);

// Version 21's cursor threshold came from the report CRC, so migrate it to the default.
void migrate_tps43_tuning_v21(DualTps43Tuning* tuning);

// Copies the safe runtime-editable subset into the HID protocol structure.
bool get_configured_tps43_runtime_tuning(tps43_runtime_tuning_t* controls);

// Updates only the safe runtime-editable subset, preserving cursor-filter and
// momentum settings. Returns false without changing the profile
// when the resulting full tuning is invalid.
bool set_configured_tps43_runtime_tuning(const tps43_runtime_tuning_set_t& controls);

// Updates the cursor threshold carried by its dedicated one-byte SET command.
bool set_configured_tps43_cursor_threshold(uint8_t threshold);

// Reads/writes the isolated temporal cursor-filter A/B setting.
bool get_configured_tps43_cursor_filter(tps43_cursor_filter_tuning_t* controls);
bool set_configured_tps43_cursor_filter(const tps43_cursor_filter_tuning_t& controls);

#endif
