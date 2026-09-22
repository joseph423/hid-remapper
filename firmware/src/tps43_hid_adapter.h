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

// Bridges logical TPS43 actions into Remapper's report-building path while
// retaining button state between coordinator cycles.
class Tps43RemapperActionSink final : public Tps43ActionSink {
   public:
    // Applies one logical action cycle without calling TinyUSB directly.
    void apply(const LogicalActions& actions) override;

    // Releases TPS43-owned buttons and removes any pending click release.
    void reset();

   private:
    static void apply_button_action(ButtonAction action, bool& held, bool& release_pending);

    bool left_button_held_ = false;
    bool right_button_held_ = false;
    bool left_release_pending_ = false;
    bool right_release_pending_ = false;
};

#endif
