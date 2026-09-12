#ifndef TPS43_CDC_STDIO_H_
#define TPS43_CDC_STDIO_H_

// Registers the combined TinyUSB device CDC interface as the stdio source
// used by the one-sensor timing capture.
void tps43_cdc_stdio_init();

// Flushes buffered diagnostic output without waiting for the host.
void tps43_cdc_stdio_flush();

#endif
