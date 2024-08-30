/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \license This file is released under the MIT License. See the LICENSE file for details.
 *  \brief OS/hardware portability interface
 *  \version 2.0
 * Note: any file that includes this header must first include a platform-specific header
 *       (e.g. `sys_linux.h`, `sys_esp.h`, etc.) which defines structures such as `i2c_lowlevel_config`.
 */
#ifdef _SYS_PORTABILITY_H
   #ifndef SYS_PORTABILITY_VERSION
      #define SYS_PORTABILITY_VERSION 2
   #else
      #if SYS_PORTABILITY_VERSION != 2
         #error "System portability version mismatch"
      #endif
   #endif
#else
#define _SYS_PORTABILITY_H

/* It's expected that this system portability implementation, a required dependency, may be included more
 *  than once for a single application. The definitions of each sys function are `weak` to instruct the
 *  linker to choose only one instance of the functionality even though multiple may exist at compile time.
 *  There is a version check at the top of this file to ensure (at compile time) that all instances of
 *  the system portability layer are compatible. */
#define SYS_WEAK __attribute__((weak))

#include <stdbool.h>
#include <stdint.h>

/* i2c */
typedef void *i2c_lowlevel_context;
i2c_lowlevel_context i2c_ll_init(uint8_t i2c_address, uint32_t i2c_speed, uint32_t i2c_timeout_ms,
                                 i2c_lowlevel_config *config);
bool i2c_ll_deinit(i2c_lowlevel_context ctx);
bool i2c_ll_write(i2c_lowlevel_context ctx, uint8_t *data, uint8_t length);
bool i2c_ll_write_reg(i2c_lowlevel_context ctx, uint8_t reg, uint8_t *data, uint8_t length);
bool i2c_ll_read(i2c_lowlevel_context ctx, uint8_t *data, uint8_t length);
bool i2c_ll_read_reg(i2c_lowlevel_context ctx, uint8_t reg, uint8_t *data, uint8_t length);

/* uart */
typedef void *uart_lowlevel_context;
typedef void (*uart_ll_rx_handler_fn)(uint8_t *data, uint32_t length, void *cookie);
typedef enum { SYS_UART_PARITY_NONE, SYS_UART_PARITY_EVEN, SYS_UART_PARITY_ODD } uart_ll_parity;
typedef enum { SYS_UART_STOP_BITS_ONE, SYS_UART_STOP_BITS_TWO } uart_ll_stop_bits;
typedef enum { SYS_UART_DATA_BITS_FIVE, SYS_UART_DATA_BITS_SIX, SYS_UART_DATA_BITS_SEVEN,
               SYS_UART_DATA_BITS_EIGHT } uart_ll_data_bits;
uart_lowlevel_context uart_ll_init(uint32_t baud, uart_ll_data_bits data_bits, uart_ll_stop_bits stop_bits,
                                   uart_ll_parity parity, uart_lowlevel_config *config);
bool uart_ll_deinit(uart_lowlevel_context ctx);
bool uart_ll_set_rx_handler(uart_lowlevel_context ctx, uart_ll_rx_handler_fn handler, void *cookie);
bool uart_ll_send(uart_lowlevel_context ctx, uint8_t *data, uint32_t length);

/* time */
#if defined(ESP_PLATFORM)
   #include "rom/ets_sys.h"  /* ets_delay_us */
   __inline int sys_delay_us(size_t x) { ets_delay_us(x); return 0; }
#elif defined(__linux__)
   #define sys_delay_us(x) usleep(x)
#endif
uint64_t sys_microsecond_tick(void);

/* mutex */
typedef void *mutex_lowlevel;
mutex_lowlevel sys_mutex_init(void);
bool sys_mutex_deinit(mutex_lowlevel mutex);
bool sys_mutex_lock(mutex_lowlevel mutex);
bool sys_mutex_unlock(mutex_lowlevel mutex);

#endif /* _SYS_PORTABILITY_H */
