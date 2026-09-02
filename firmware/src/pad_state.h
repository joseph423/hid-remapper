#ifndef PAD_STATE_H_
#define PAD_STATE_H_

#include <cstdint>

#include "tps43_driver.h"

struct PadState {
    bool active = false;
    uint8_t finger_count = 0;
    int32_t relative_x = 0;
    int32_t relative_y = 0;
    bool movement_reported = false;
    bool single_tap = false;
    bool two_finger_tap = false;
    bool scroll_gesture = false;

    // Centroid fields are normalized and sensor-independent. A centroid delta
    // is valid only across consecutive samples with valid three-finger
    // centroids; otherwise the current centroid establishes a new baseline.
    bool three_finger_centroid_valid = false;
    int32_t three_finger_centroid_x = 0;
    int32_t three_finger_centroid_y = 0;
    bool three_finger_delta_valid = false;
    int32_t three_finger_delta_x = 0;
    int32_t three_finger_delta_y = 0;
    bool touch_started = false;
    bool touch_ended = false;
    uint64_t timestamp_us = 0;

    // Increments on each inactive-to-active transition and remains stable for
    // the rest of that touch session.
    uint32_t session_id = 0;
};

class PadStateTracker {
   public:
    PadState update(const Tps43Sample& sample);
    const PadState& state() const;

   private:
    PadState state_;
};

#endif
