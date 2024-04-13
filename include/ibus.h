#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>
#include <stdbool.h> /* requires C99 */

typedef void *ibus_context_t;

#define IBUS_CHANNEL_COUNT       14
typedef struct ibus_channel_s
{
   uint16_t value;
   uint16_t min;
   uint16_t max;
   uint32_t update_count;
} ibus_channel_t;

typedef struct ibus_results_s 
{
   /* statistics */
   uint32_t rx_success_count;
   uint32_t checksum_fail_count;
   uint32_t timeout_count;

   ibus_channel_t channel[IBUS_CHANNEL_COUNT];
} ibus_results_t;

/* ----------------------------------------------------------------------------
 * Initialization
 */

#if defined(__linux__)
typedef struct ibus_lowlevel_config_s
{
   /* Note that it may be necessary to access uart device files as root */
   const char *device;   /* e.g. "/dev/ttyS0" */
} ibus_lowlevel_config;

#elif defined(ESP_PLATFORM)
#include "driver/uart.h"
typedef struct ibus_lowlevel_config_s
{
    uart_port_t port;
} ibus_lowlevel_config;

#else
#pragma message("System type not detected; no UART handling included") 
#define IBUS_LOWLEVEL_NONE
#endif

#if defined(IBUS_LOWLEVEL_NONE)

/* This library supports a generic IBUS parsing implementation, where it's
   the responsibility of the calling application to receive 8-bit characters from
   a UART (the mechanism is determined and implementation by the calling application)
   with 32-bit timestamp corresponding to the time when the 8-bit character was received.
   The receive timestamp is optional (and can be set to zero for all received bytes) if
   there is no desire to monitor for receive timeouts. If receive timeout processing is
   not utilized, parser state management is significantly compromised, as delays
   (time between IBUS messages) are used by the protocol to delineate message start/end.

   The calling application is responsible for calling ibus_handle_byte() in order to
   perform IBUS message parsing.
*/
typedef struct
{
   uint64_t rx_timeout;
   uint64_t timestamp_max;
} ibus_lowlevel_config;
#endif /* IBUS_LOWLEVEL_NONE */

ibus_context_t ibus_init(ibus_lowlevel_config *config);
bool ibus_deinit(ibus_context_t context);
void ibus_handle_byte(ibus_context_t context, uint8_t val, uint64_t timestamp);

#endif /* IBUS_H */
