#ifndef TPS43_TUNING_CONFIG_H_
#define TPS43_TUNING_CONFIG_H_

#include <cstddef>
#include <cstdint>

#include "dual_tps43_fsm.h"

// The block is serialized field-by-field in little-endian order. It must not
// depend on C++ structure padding or compiler ABI details.
constexpr uint32_t kTps43TuningBlockMagic = 0x54343354;
constexpr uint8_t kTps43TuningBlockVersion = 1;
constexpr std::size_t kTps43TuningBlockSize = 78;

// Returns the approved production defaults used by the current physical
// profile. The returned value is independent of persisted configuration.
DualTps43Tuning production_tuning();

// Validates field relationships and coefficient ranges without applying the
// tuning to an active FSM.
bool validate_tps43_tuning(const DualTps43Tuning& tuning);

// Encodes a validated tuning block. Returns false for an invalid tuning or an
// insufficient output buffer.
bool encode_tps43_tuning(const DualTps43Tuning& tuning, uint8_t* buffer, std::size_t buffer_size);

// Decodes and validates one complete tuning block. The destination is not
// modified when the header, size, or tuning values are invalid.
bool decode_tps43_tuning(const uint8_t* buffer, std::size_t buffer_size, DualTps43Tuning* tuning);

#endif
