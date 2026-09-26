#include "tps43_cdc_stdio.h"

#include <cstddef>
#include <cstdint>

#include <tusb.h>

#include "pico/stdio.h"
#include "pico/stdio/driver.h"

namespace {

constexpr size_t kOutputBufferSize = 4096;

uint8_t output_buffer[kOutputBufferSize] = {};
size_t output_head = 0;
size_t output_tail = 0;

size_t next_output_index(size_t index) {
    return (index + 1) % kOutputBufferSize;
}

void flush_output_buffer() {
    // Most production loops have no serial output; avoid CDC work then, while
    // still delivering a queued startup error if a monitor connects later.
    if (output_head == output_tail || !tud_cdc_connected()) {
        return;
    }

    while (output_head != output_tail && tud_cdc_write_available() != 0) {
        const size_t contiguous = output_tail > output_head
                                      ? output_tail - output_head
                                      : kOutputBufferSize - output_head;
        const uint32_t available = tud_cdc_write_available();
        const uint32_t requested = static_cast<uint32_t>(contiguous < available ? contiguous : available);
        const uint32_t written = tud_cdc_write(&output_buffer[output_head], requested);
        if (written == 0) {
            break;
        }
        output_head = (output_head + written) % kOutputBufferSize;
    }
    tud_cdc_write_flush();
}

void cdc_out_chars(const char* buffer, int length) {
    if (length <= 0) {
        return;
    }

    flush_output_buffer();
    for (int i = 0; i < length; ++i) {
        const size_t next_tail = next_output_index(output_tail);
        if (next_tail == output_head) {
            break;
        }
        output_buffer[output_tail] = static_cast<uint8_t>(buffer[i]);
        output_tail = next_tail;
    }
    flush_output_buffer();
}

int cdc_in_chars(char* buffer, int length) {
    if (length <= 0 || !tud_cdc_connected() || tud_cdc_available() == 0) {
        return PICO_ERROR_NO_DATA;
    }

    const uint32_t read = tud_cdc_read(buffer, static_cast<uint32_t>(length));
    return read == 0 ? PICO_ERROR_NO_DATA : static_cast<int>(read);
}

stdio_driver_t cdc_stdio_driver = {
    .out_chars = cdc_out_chars,
    .out_flush = flush_output_buffer,
    .in_chars = cdc_in_chars,
    .set_chars_available_callback = nullptr,
    .next = nullptr,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .last_ended_with_cr = false,
    .crlf_enabled = PICO_STDIO_DEFAULT_CRLF,
#endif
};

}  // namespace

void tps43_cdc_stdio_init() {
    stdio_set_driver_enabled(&cdc_stdio_driver, true);
}

void tps43_cdc_stdio_flush() {
    flush_output_buffer();
}
