/* \copyright 2024 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ibus Linux implementation
 */
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <errno.h>
#include <time.h>
#include <inttypes.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "helpers.h"
#include "ibus_private.h"

#define LINUX_RX_TIMEOUT      1000 /* microseconds */
#define LINUX_TIMESTAMP_MAX   UINT64_MAX

typedef struct linux_ibus_s
{
    char *device;
    int handle;
    pthread_mutex_t lock;
    pthread_t thread;
} linux_ibus_t;

int ibus_ll_init(ibus_state_t *state, ibus_lowlevel_config *config)
{
   linux_ibus_t *l;
   int result = -1;

   l = (linux_ibus_t *) malloc(sizeof(*l));
   if(NULL == l)
   {
      IBUSERR("[%s] Failed to allocate low-level structure\n", __func__);
      return -1;
   }

   l->handle = -1;
   l->device = strdup(config->device);
   if(NULL == l->device)
   {
      IBUSERR("[%s] Memory allocation error\n", __func__);
   }
   else
   {
      l->handle = open(l->device, O_RDWR);
      if(l->handle < 0)
      {
         IBUSERR("[%s] Failed to open device '%s'\n", __func__, l->device);
      }
      else
      {
         /* TODO: serial init */
         /* TODO: thread init */
         result = 0;
      }
   }

   if(0 != result)
   {
      if(l->handle >= 0)
         close(l->handle);
      free(l);
      l = NULL;
   }

   state->lowlevel = l;
   state->rx_timeout = LINUX_RX_TIMEOUT;
   state->timestamp_max = LINUX_TIMESTAMP_MAX;
   IBUSERR("[%s] result %d\n", __func__, result);
   return result;
}

int ibus_ll_deinit(ibus_state_t *state)
{
   linux_ibus_t *l = (linux_ibus_t *) state->lowlevel; 

   if(NULL == l)
      return 0;

   if(l->handle >= 0)
      close(l->handle);
   if(NULL != l->device)
      free(l->device);
   free(l);

   return 0;
}