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

    // Requests a raw contact read only for an armed three-finger mismatch.
    bool wants_diagnostic_contact(const Tps43Sample& sample, const Tps43ServiceTiming& timing) const;

    // Consumes a non-blocking serial Enter and starts the next report window.
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
};

#endif
