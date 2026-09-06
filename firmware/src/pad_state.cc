#include "pad_state.h"

PadState PadStateTracker::update(const Tps43Sample& sample, bool fresh) {
    if (!fresh) {
        // Keep the centroid baseline and session across acquisition gaps, but
        // never replay compact movement, centroid deltas, or gesture edges.
        state_.fresh_sample = false;
        state_.relative_x = state_.relative_y = 0;
        state_.movement_reported = false;
        state_.single_tap = state_.two_finger_tap = state_.scroll_gesture = false;
        state_.touch_started = state_.touch_ended = false;
        state_.three_finger_delta_valid = false;
        state_.three_finger_delta_x = state_.three_finger_delta_y = 0;
        return state_;
    }

    PadState next;
    next.fresh_sample = true;
    if (have_sample_ && sample.timestamp_us > state_.timestamp_us) {
        next.sample_interval_us = sample.timestamp_us - state_.timestamp_us;
    }
    have_sample_ = true;
    next.active = sample.active;
    next.finger_count = sample.finger_count;
    next.relative_x = sample.relative_x;
    next.relative_y = sample.relative_y;
    next.movement_reported = sample.movement_reported;
    next.single_tap = sample.single_tap;
    next.two_finger_tap = sample.two_finger_tap;
    next.scroll_gesture = sample.scroll_gesture;
    next.touch_started = sample.active && !state_.active;
    next.touch_ended = !sample.active && state_.active;
    next.timestamp_us = sample.timestamp_us;
    next.session_id = state_.session_id + (next.touch_started ? 1 : 0);

    // Leaving this condition clears centroid validity through PadState's
    // defaults, so a later three-finger sequence cannot jump from a stale
    // baseline.
    if (sample.active && sample.finger_count == 3 && sample.contact_details_available) {
        int32_t contact_x_sum = 0;
        int32_t contact_y_sum = 0;
        uint8_t active_contact_count = 0;

        for (const Tps43Contact& contact : sample.contacts) {
            if (!contact.active) {
                continue;
            }

            contact_x_sum += contact.x;
            contact_y_sum += contact.y;
            active_contact_count++;
        }

        if (active_contact_count == 3) {
            next.three_finger_centroid_valid = true;
            next.three_finger_centroid_x = contact_x_sum / active_contact_count;
            next.three_finger_centroid_y = contact_y_sum / active_contact_count;

            if (state_.three_finger_centroid_valid) {
                next.three_finger_delta_valid = true;
                next.three_finger_delta_x = next.three_finger_centroid_x - state_.three_finger_centroid_x;
                next.three_finger_delta_y = next.three_finger_centroid_y - state_.three_finger_centroid_y;
            }
        }
    }

    state_ = next;
    return state_;
}

const PadState& PadStateTracker::state() const {
    return state_;
}
