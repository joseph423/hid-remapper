#include <cassert>
#include <cstdint>
#include <vector>

#include "mapped_layers.h"

namespace {

constexpr uint32_t kLayersUsagePage = 0xFFF10000;
constexpr uint32_t kExpressionUsagePage = 0xFFF30000;
constexpr uint8_t kStickyFlag = 1 << 0;
constexpr uint8_t kLayerCount = 4;
constexpr uint8_t kExpressionCount = 2;

mapping_config11_t mapping(uint32_t source, uint32_t target, uint8_t layers, uint8_t flags = 0) {
    mapping_config11_t result{};
    result.source_usage = source;
    result.target_usage = target;
    result.layer_mask = layers;
    result.flags = flags;
    return result;
}

uint8_t mapped(uint32_t usage,
               const std::vector<mapping_config11_t>& mappings,
               const std::vector<expr_elem_t> (&expressions)[kExpressionCount]) {
    return mapped_layers_for_usage(usage, mappings, expressions, kLayersUsagePage,
                                   kExpressionUsagePage, kStickyFlag, kLayerCount,
                                   kExpressionCount);
}

void test_direct_source_masks_are_combined() {
    constexpr uint32_t kUsage = 0x00070004;
    std::vector<mapping_config11_t> mappings{
        mapping(kUsage, 0x00090001, 0b0010),
        mapping(kUsage, 0x00090002, 0b0100),
        mapping(0x00070005, 0x00090003, 0b1000),
    };
    std::vector<expr_elem_t> expressions[kExpressionCount];

    assert(mapped(kUsage, mappings, expressions) == 0b0110);
}

void test_layer_targets_preserve_sticky_and_nonsticky_rules() {
    constexpr uint32_t kNonstickySource = 0x00070010;
    constexpr uint32_t kStickySource = 0x00070011;
    std::vector<mapping_config11_t> mappings{
        mapping(kNonstickySource, kLayersUsagePage | 2, 0b0001),
        mapping(kStickySource, kLayersUsagePage | 2, 0b0011, kStickyFlag),
    };
    std::vector<expr_elem_t> expressions[kExpressionCount];

    assert(mapped(kNonstickySource, mappings, expressions) == 0b0101);
    assert(mapped(kStickySource, mappings, expressions) == 0b0111);
}

void test_expression_push_usage_inherits_mapping_layers() {
    constexpr uint32_t kExpressionSource = kExpressionUsagePage | 1;
    constexpr uint32_t kPushedUsage = 0x00070020;
    constexpr uint32_t kLiteralOnlyUsage = 0x00070021;
    std::vector<mapping_config11_t> mappings{
        mapping(kExpressionSource, 0x00090001, 0b1010),
    };
    std::vector<expr_elem_t> expressions[kExpressionCount];
    expr_elem_t pushed_usage{};
    pushed_usage.op = Op::PUSH_USAGE;
    pushed_usage.val = kPushedUsage;
    expr_elem_t literal{};
    literal.op = Op::PUSH;
    literal.val = kLiteralOnlyUsage;
    expressions[0] = {pushed_usage, literal};

    assert(mapped(kPushedUsage, mappings, expressions) == 0b1010);
    assert(mapped(kLiteralOnlyUsage, mappings, expressions) == 0);
}

}  // namespace

int main() {
    test_direct_source_masks_are_combined();
    test_layer_targets_preserve_sticky_and_nonsticky_rules();
    test_expression_push_usage_inherits_mapping_layers();
}
