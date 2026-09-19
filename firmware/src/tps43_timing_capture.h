#ifndef TPS43_TIMING_CAPTURE_H_
#define TPS43_TIMING_CAPTURE_H_

#include <cstddef>
#include <cstdint>

#include "tps43_iqs5xx_driver.h"
#include "tps43_timing_metrics.h"

// Guides the physical timing evidence capture without changing sensor state
// or selecting production behavior-tuning values.
class Tps43TimingCapture final {
   public:
    // Prints the first operator-gated capture instruction.
    void begin();

    // Records one completed Right-pad service step and advances the active
    // stage after the requested automatic report window is complete.
    void record(
        uint64_t now_us,
        const Tps43Sample& sample,
        const Tps43ServiceTiming& timing,
        const Tps43Sample* diagnostic_contact_sample = nullptr);

    // Records bounded per-touch summaries for both physical pads. This is
    // diagnostic-only: it does not request sensor work or affect HID output.
    void record_dual_input(
        uint64_t now_us,
        const Tps43Sample& left_sample,
        const Tps43ServiceTiming& left_timing,
        const Tps43Sample& right_sample,
        const Tps43ServiceTiming& right_timing);

    // Returns whether the opt-in 10 Hz compact-sample debug stream is active.
    bool manual_debug_enabled() const;

    // Requests a raw contact read only for an armed three-finger mismatch.
    bool wants_diagnostic_contact(const Tps43Sample& sample, const Tps43ServiceTiming& timing) const;

    // Returns whether the armed legacy contact stage needs mismatch details.
    bool diagnostic_contact_requested() const;

    // Consumes non-blocking serial commands: D toggles compact-sample debug;
    // M and Enter control the staged timing capture.
    void poll_serial();

    // Returns whether the next tick should force one operator-requested read.
    bool read_requested() const;

   private:
    enum class Stage {
        OneFinger,
        TwoFinger,
        Contact,
        Concurrent,
        Complete,
    };

    void print_sample_prompt() const;
    void finish_report_stage(uint64_t now_us, const Tps43ServiceTiming& timing);
    void start_concurrent_capture(uint64_t now_us, const Tps43ServiceTiming& timing);
    void finish_concurrent_capture(uint64_t now_us, const Tps43ServiceTiming& timing);

    struct InputSession {
        bool active = false;
        uint64_t started_us = 0;
        int64_t relative_x = 0;
        int64_t relative_y = 0;
        uint32_t fresh_samples = 0;
        uint32_t movement_samples = 0;
        uint32_t maximum_delta = 0;
    };

    void record_input(
        const char* pad_name,
        const Tps43Sample& sample,
        const Tps43ServiceTiming& timing,
        InputSession& session);

    Stage stage_ = Stage::OneFinger;
    size_t captured_samples_ = 0;
    size_t stage_mismatches_ = 0;
    size_t one_finger_mismatches_ = 0;
    size_t two_finger_mismatches_ = 0;
    size_t contact_mismatches_ = 0;
    bool armed_ = false;
    uint64_t concurrent_started_us_ = 0;
    uint32_t concurrent_max_service_us_ = 0;
    uint32_t concurrent_start_failures_ = 0;
    InputSession left_input_session_;
    InputSession right_input_session_;
    bool manual_debug_enabled_ = false;
    uint64_t last_manual_debug_us_ = 0;
};

#endif
