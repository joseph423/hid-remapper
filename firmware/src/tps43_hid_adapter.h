#ifndef TPS43_HID_ADAPTER_H_
#define TPS43_HID_ADAPTER_H_

#include "dual_tps43_fsm.h"

// Integration boundary that translates logical actions into HID Remapper's
// central output path. Implementations must not add touch-policy decisions.
class Tps43ActionSink {
   public:
    virtual ~Tps43ActionSink() = default;
    virtual void apply(const LogicalActions& actions) = 0;
};

#endif
