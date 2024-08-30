/* \copyright 2024 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ibus library interface
 */
#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>
#include <stdbool.h> /* requires C99 */

#if defined(__linux__)
   #include "ibus/sys/sys_linux.h"
#elif defined(ESP_PLATFORM)
   #include "ibus/sys/sys_esp.h"
#else
   #warning "Supported OS type not detected"
   #define IBUS_LOWLEVEL_NONE 1
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
   } uart_lowlevel_config;
#endif

#define IBUS_CHANNEL_COUNT 14
typedef struct ibus_channel_s
{
   uint16_t value;
   uint16_t min;
   uint16_t max;
   uint32_t update_count;
} ibus_channel_t;

typedef struct ibus_statistics_s
{
   uint64_t rx_byte_count;
   uint64_t rx_message_count;
   uint64_t rx_message_checksum_fail_count;
   uint64_t timeout_count;
   uint64_t min_delay;
   uint64_t max_delay;
} ibus_statistics_t;

typedef void *ibus_context_t;
typedef void (*ibus_handle_channel_update_fn)(ibus_channel_t *channels, void *cookie);

ibus_context_t ibus_init(uart_lowlevel_config *config);
bool ibus_deinit(ibus_context_t context);

/*! \brief Set a callback function which will be called each time a valid channel report
 *         message has been received.
 *  \param[in] context ibus context, received as a result of a successful ibus_init() call
 *  \param[in] handler Channel update function pointer
 *  \param[in] cookie  Opaque data pointer, passed to the channel update handler function.
 *                     This is useful for the caller to pass state information to the handler.
 */
bool ibus_set_channel_handler(ibus_context_t context, ibus_handle_channel_update_fn handler, void *cookie);

/*! \brief Obtain a copy of the current ibus statistics.
 *  \param[in]  context ibus context, received as a result of a successful ibus_init() call
 *  \param[out] stats   Pointer to structure to receive the current statistics
 */
bool ibus_get_statistics(ibus_context_t context, ibus_statistics_t *stats);

/*! \brief Reset ibus statistics 
 *  \param[in] context ibus context, received as a result of a successful ibus_init() call
 */
bool ibus_reset_statistics(ibus_context_t context);

/*! \brief Reset value/statistics for the specified ibus channel 
 *  \param[in] context ibus context, received as a result of a successful ibus_init() call
 *  \param[in] channel Index (0 - 13 valid) of channel to reset
 */
bool ibus_reset_channel(ibus_context_t context, uint8_t channel);

/*! \brief Send a byte into the ibus library implementation for processing. This function is
 *         only useful for systems which don't have a supported low-level implementation.
 *         This function is used to provide received bytes to the ibus library for processing.
 *         See description under IBUS_LOWLEVEL_NONE above.
 *  \param[in] context   ibus context, received as a result of a successful ibus_init() call
 *  \param[in] val       Received byte to process
 *  \param[in] timestamp Time at which the byte was received
 */
void ibus_handle_byte(ibus_context_t context, uint8_t val, uint64_t timestamp);

#endif /* IBUS_H */
