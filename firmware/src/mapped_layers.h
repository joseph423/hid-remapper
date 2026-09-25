#ifndef MAPPED_LAYERS_H
#define MAPPED_LAYERS_H

#include <cstdint>

#include "types.h"

// Computes one usage's effective mapping-layer mask without building a heap-backed index.
// This runs during configuration rebuilds, where temporary allocation headroom is limited.
template <typename MappingRange, typename ExpressionArray>
uint8_t mapped_layers_for_usage(
    uint32_t usage,
    const MappingRange& mappings,
    const ExpressionArray& expressions,
    uint32_t layers_usage_page,
    uint32_t expression_usage_page,
    uint8_t sticky_flag,
    uint8_t layer_count,
    uint8_t expression_count) {
    uint8_t mapped_layers = 0;

    for (const auto& mapping : mappings) {
        uint8_t layer_mask = mapping.layer_mask;
        if ((mapping.target_usage & 0xFFFF0000) == layers_usage_page) {
            uint16_t layer = mapping.target_usage & 0xFFFF;
            uint8_t layer_bit = (1U << layer) & ((1U << layer_count) - 1);
            if (mapping.flags & sticky_flag) {
                layer_mask &= ~(1U << layer);
                if (mapping.source_usage == usage) {
                    mapped_layers |= layer_bit;
                }
            } else {
                layer_mask |= layer_bit;
            }
        }

        if (mapping.source_usage == usage) {
            mapped_layers |= layer_mask;
        }

        if ((mapping.source_usage & 0xFFFF0000) != expression_usage_page) {
            continue;
        }

        uint16_t expression_number = mapping.source_usage & 0xFFFF;
        if (expression_number == 0 || expression_number > expression_count) {
            continue;
        }
        for (const auto& elem : expressions[expression_number - 1]) {
            if (elem.op == Op::PUSH_USAGE && elem.val == usage) {
                mapped_layers |= layer_mask;
            }
        }
    }

    return mapped_layers;
}

#endif
