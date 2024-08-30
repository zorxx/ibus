/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \license This file is released under the MIT License. See the LICENSE file for details.
 *  \brief Linux portability implementation
 *  \version 2.0
 */
#include "sys/sys_linux.h"
#include "sys.h"
#include "helpers.h"
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h> /* strdup */
#include <fcntl.h> /* open/close */
#include <time.h> /* clock_gettime */
#include <pthread.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/* --------------------------------------------------------------------------------------------------------------
 * i2c
 */

typedef struct linux_rtci2c_s
{
    char *device;
    int handle;
    uint32_t timeout;
} linux_i2c_t;

i2c_lowlevel_context SYS_WEAK i2c_ll_init(uint8_t i2c_address, uint32_t i2c_speed, uint32_t i2c_timeout_ms,
                                          i2c_lowlevel_config *config)
{
   linux_i2c_t *l;
   int result = -1;

   if(NULL == config)
      return NULL;

   l = (linux_i2c_t *) malloc(sizeof *l);
   if(NULL == l)
   {
      SERR("[%s] Failed to allocate low-level structure", __func__);
      return NULL;
   }

   l->handle = -1;
   l->timeout = i2c_timeout_ms;
   l->device = strdup(config->device);
   if(NULL == l->device)
   {
      SERR("[%s] Memory allocation error", __func__);
   }
   else
   {
      l->handle = open(l->device, O_RDWR);
      if(l->handle < 0)
      {
         SERR("[%s] Failed to open device '%s'", __func__, l->device);
      }
      else if(ioctl(l->handle, I2C_SLAVE, i2c_address) < 0)
      {
         SERR("[%s] Failed to set I2C slave address to 0x%02x", __func__, i2c_address);
      }
      else
         result = 0;
   }

   if(0 != result)
   {
      if(l->handle >= 0)
         close(l->handle);
      free(l);
      l = NULL;
   }

   SERR("[%s] result %d", __func__, result);
   return (i2c_lowlevel_context) l;
}

bool SYS_WEAK i2c_ll_deinit(i2c_lowlevel_context ctx)
{
   linux_i2c_t *l = (linux_i2c_t *) ctx;
   if(NULL == l)
      return true;

   if(l->handle >= 0)
      close(l->handle);
   if(NULL != l->device)
      free(l->device);
   free(l);

   return true;
}

bool SYS_WEAK i2c_ll_write_reg(i2c_lowlevel_context ctx, uint8_t reg, uint8_t *data, uint8_t length)
{
   linux_i2c_t *l = (linux_i2c_t *) ctx;
   struct i2c_smbus_ioctl_data args;
   union i2c_smbus_data smdata;
   int result = -EINVAL;

   if(length > I2C_SMBUS_BLOCK_MAX)
   {
      SERR("[%s] Data length overflow (%u bytes)", __func__, length);
   }
   else
   {
      smdata.block[0] = length;
      memcpy(&smdata.block[1], data, length);
      args.read_write = I2C_SMBUS_WRITE;
      args.command = reg;
      args.size = I2C_SMBUS_I2C_BLOCK_DATA;
      args.data = &smdata; 
      result = ioctl(l->handle, I2C_SMBUS, &args);
      if(0 == result)
      {
         SDBG("[%s] Success (%u bytes)", __func__, length);
         return true;
      }
   }

   SERR("[%s] Failed (result %d, errno %d)", __func__, result, errno);
   return false;
}

bool SYS_WEAK i2c_ll_write(i2c_lowlevel_context ctx, uint8_t *data, uint8_t length)
{
   linux_i2c_t *l = (linux_i2c_t *) ctx;
   int result = write(l->handle, data, length);
   if(length == result)
   {
      SDBG("[%s] Success (%u bytes)", __func__, length);
      return true;
   }

   SERR("[%s] Failed (result %d, errno %d)", __func__, result, errno);
   return false;
}

bool SYS_WEAK i2c_ll_read_reg(i2c_lowlevel_context ctx, uint8_t reg, uint8_t *data, uint8_t length)
{
   linux_i2c_t *l = (linux_i2c_t *) ctx;
   struct i2c_smbus_ioctl_data args;
   union i2c_smbus_data smdata;
   int result = -EINVAL;

   if(length > I2C_SMBUS_BLOCK_MAX)
   {
      SERR("[%s] Data length overflow (%u bytes)", __func__, length);
   }
   else
   {
      smdata.block[0] = length;
      args.read_write = I2C_SMBUS_READ;
      args.command = reg;
      args.size = I2C_SMBUS_I2C_BLOCK_DATA;
      args.data = &smdata;
      result = ioctl(l->handle, I2C_SMBUS, &args);
      if(0 == result)
      {
         SDBG("[%s] Success (%u bytes)", __func__, length);
         memcpy(data, &smdata.block[1], length);
         return true;
      }
   }

   SERR("[%s] Failed (result %d, errno %d)", __func__, result, errno);
   memset(&smdata.block[1], 0, length);
   return false;
}

bool SYS_WEAK i2c_ll_read(i2c_lowlevel_context ctx, uint8_t *data, uint8_t length)
{
   linux_i2c_t *l = (linux_i2c_t *) ctx;
   int result = read(l->handle, data, length);
   if(length == result)
   {
      SDBG("[%s] Success (%u bytes)", __func__, length);
      return true;
   }

   SERR("[%s] Failed (result %d, errno %d)", __func__, result, errno);
   memset(data,  0, length);
   return false;
}

/* --------------------------------------------------------------------------------------------------------------
 * uart
 */

#include <termios.h>

typedef struct linux_uart_s
{
    char *device;
    int handle;
    uart_ll_rx_handler_fn rx_handler;
    void *rx_cookie;
    pthread_t rx_thread;
} linux_uart_t;

#define SYS_UART_MAX_RX_LENGTH 256

static void *uart_ll_rx_thread(void *param)
{
   linux_uart_t *l = (linux_uart_t *) param;
   size_t result;
   bool done = false;
   uint8_t buffer[SYS_UART_MAX_RX_LENGTH];

   SDBG("[%s] Started\n", __func__);
   while(!done)
   {
      result = read(l->handle, buffer, sizeof buffer);
      if(result < 0)
      {
         SERR("[%s] Failed to read from '%s'\n", __func__, l->device);
         usleep(10000); /* prevent tight loops */
      }
      if(NULL != l->rx_handler)
         l->rx_handler(buffer, (uint32_t) result, l->rx_cookie);
   }

   return NULL;
}

static inline int baud_convert(uint32_t baud)
{
   switch(baud)
   {
      case 0:       return B0;
      case 50:      return B50;
      case 75:      return B75;
      case 110:     return B110;
      case 134:     return B134;
      case 150:     return B150;
      case 200:     return B200;
      case 300:     return B300;
      case 600:     return B600;
      case 1200:    return B1200;
      case 1800:    return B1800;
      case 2400:    return B2400;
      case 4800:    return B4800;
      case 9600:    return B9600;
      case 19200:   return B19200;
      case 38400:   return B38400;
      case 57600:   return B57600;
      case 115200:  return B115200;
      case 230400:  return B230400;
      case 460800:  return B460800;
      case 500000:  return B500000;
      case 576000:  return B576000;
      case 921600:  return B921600;
      case 1000000: return B1000000;
      case 1152000: return B1152000;
      case 1500000: return B1500000;
      case 2000000: return B2000000;
      case 2500000: return B2500000;
      case 3000000: return B3000000;
      case 3500000: return B3500000;
      case 4000000: return B4000000;
      default: return -1;
   }
}

uart_lowlevel_context SYS_WEAK uart_ll_init(uint32_t baud, uart_ll_data_bits data_bits, uart_ll_stop_bits stop_bits,
                                            uart_ll_parity parity, uart_lowlevel_config *config)
{
   linux_uart_t *l;
   bool success = false;
   struct termios tty;

   if(NULL == config)
      return NULL;

   l = (linux_uart_t *) malloc(sizeof *l);
   if(NULL == l)
   {
      SERR("[%s] Failed to allocate low-level structure", __func__);
      return NULL;
   }
   memset(l, 0, sizeof *l);
   l->device = strdup(config->device);

   l->handle = open(config->device, O_RDWR | O_NOCTTY | O_SYNC);
   if(l->handle < 0)
   {
      SERR("[%s] Failed to open device '%s'", __func__, config->device);
   }
   else if(tcgetattr(l->handle, &tty) != 0)
   {
      SERR("[%s] Failed to read settings for device '%s'\n", __func__, config->device);
   }
   else
   {
      bool invalid_config = false;

      int b = baud_convert(baud);
      if(-1 == b)
      {
         SERR("[%s] Invalid tx baud (%u)\n", __func__, baud);
         invalid_config = true;
      }
      else
      {
         cfsetospeed(&tty, b);
         cfsetispeed(&tty, b);
      }

      /* parity (even/odd/none) */
      tty.c_cflag &= ~PARENB;
      if(parity != SYS_UART_PARITY_NONE)
      {
         tty.c_cflag |= PARENB;
         if(parity == SYS_UART_PARITY_ODD)
            tty.c_cflag |= PARODD;
      }

      /* stop bits (1 or 2) */
      switch(stop_bits)
      {
         case SYS_UART_STOP_BITS_TWO: tty.c_cflag |= CSTOPB; break;
         case SYS_UART_STOP_BITS_ONE: tty.c_cflag &= ~CSTOPB; break;
         default: break;
      }

      /* data bits */
      tty.c_cflag &= ~CSIZE;
      tty.c_cflag |= (data_bits == SYS_UART_DATA_BITS_FIVE)  ? CS5 :
                     (data_bits == SYS_UART_DATA_BITS_SIX)   ? CS6 :
                     (data_bits == SYS_UART_DATA_BITS_SEVEN) ? CS7 : CS8;

      tty.c_cflag &= ~CRTSCTS; /* disable hardware flow control */
      tty.c_cflag |= CREAD | CLOCAL; /* enable receiver and ignore modem control lines */
      tty.c_lflag = 0;
      tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* disable software flow control */
      tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* non-canonical mode */
      tty.c_oflag &= ~OPOST; /* no output post-processing */

      /* timeouts */
      tty.c_cc[VMIN] = 1; /* minimum character read count */
      tty.c_cc[VTIME] = 0; /* wait indefinitely */

      if(!invalid_config && tcsetattr(l->handle, TCSANOW, &tty) == 0)
         success = true;
      else
      {
         SERR("[%s] Failed to configure device '%s'\n", __func__, config->device);
      }
   }

   if(!success)
   {
      if(NULL != l->device)
         free(l->device);
      free(l);
      l = NULL;
   }

   return (uart_lowlevel_context) l;
}

bool SYS_WEAK uart_ll_set_rx_handler(uart_lowlevel_context ctx, uart_ll_rx_handler_fn handler, void *cookie)
{
   linux_uart_t *l = (linux_uart_t *) ctx;
   if(NULL == l)
      return false;

   //pthread_cancel(l->rx_thread);
   l->rx_handler = handler;
   l->rx_cookie = cookie;
   if(NULL != handler)
   {
      if(pthread_create(&l->rx_thread, NULL, uart_ll_rx_thread, l) != 0)
      {
         SERR("[%s] Failed to start Rx thread\n", __func__);
         return false;
      }
   }
   return true;
}

bool SYS_WEAK uart_ll_send(uart_lowlevel_context ctx, uint8_t *data, uint32_t length)
{
   linux_uart_t *l = (linux_uart_t *) ctx;
   if(NULL == l || NULL == data)
      return false;
   if(0 == length)
      return true;
   return (write(l->handle, data, length) == length);
}

bool SYS_WEAK uart_ll_deinit(uart_lowlevel_context ctx)
{
   linux_uart_t *l = (linux_uart_t *) ctx;
   if(NULL == l)
      return true;
   pthread_cancel(l->rx_thread);
   close(l->handle);
   if(NULL != l->device)
      free(l->device);
   free(l);
   return true;
}

/* --------------------------------------------------------------------------------------------------------------
 * mutex
 */

typedef struct linux_mutex_s
{
   pthread_mutex_t mutex;
} linux_mutex_t;


mutex_lowlevel SYS_WEAK sys_mutex_init(void)
{
   linux_mutex_t *ctx = malloc(sizeof *ctx);
   if(NULL == ctx)
      return NULL;
   pthread_mutex_init(&ctx->mutex, NULL);
   return ctx;
}

bool SYS_WEAK sys_mutex_deinit(mutex_lowlevel mutex)
{
   linux_mutex_t *ctx = (linux_mutex_t *) mutex;
   if(NULL == ctx)
      return true;
   free(ctx);
   return true;
}

bool SYS_WEAK sys_mutex_lock(mutex_lowlevel mutex)
{
   linux_mutex_t *ctx = (linux_mutex_t *) mutex;
   pthread_mutex_lock(&ctx->mutex);
   return true;
}

bool SYS_WEAK sys_mutex_unlock(mutex_lowlevel mutex)
{
   linux_mutex_t *ctx = (linux_mutex_t *) mutex;
   pthread_mutex_unlock(&ctx->mutex);
   return true;
}

/* --------------------------------------------------------------------------------------------------------------
 * time
 */

uint64_t SYS_WEAK sys_microsecond_tick(void)
{
   struct timespec ts;
   if(clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
   {
      SERR("[%s] Failed to query time (errno %d)\n", __func__, errno);
      memset(&ts, 0, sizeof ts); /* no reasonable recourse */
   }
   return ((uint64_t)ts.tv_nsec) / 1000 + (((uint64_t)ts.tv_sec) * 1000000UL);
}
