/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \license This file is released under the MIT License. See the LICENSE file for details.
 *  \brief Linux lowlevel portability interface
 *  \version 2.0
 */
#ifndef _SYS_LINUX_H
#define _SYS_LINUX_H

#include <unistd.h>

typedef struct
{
   /* Note that it may be necessary to access i2c device files as root */
   const char *device;   /* e.g. "/dev/i2c-0" */
} i2c_lowlevel_config;

typedef struct
{
   /* Note that it may be necessary to access uart device files as root */
   const char *device;   /* e.g. "/dev/ttyS0" */
} uart_lowlevel_config;

#endif /* _SYS_LINUX_H */