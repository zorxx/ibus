/* \copyright 2024 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ibus library Linux example application
 */
#include "ibus/ibus.h"
#include <stdio.h>

#define DBG(...) fprintf(stderr, __VA_ARGS__)
#define ERR(...) fprintf(stderr, __VA_ARGS__)

static void channel_handler(ibus_channel_t *channels, void *cookie)
{
   DBG("Channel update: %04x:%04x:%04x;%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
   channels[0].value, channels[1].value, channels[2].value, channels[3].value,
   channels[4].value, channels[5].value, channels[6].value, channels[7].value,
   channels[8].value, channels[9].value, channels[10].value, channels[11].value,
   channels[12].value, channels[13].value);
}

int main(int argc, char *argv[])
{
   ibus_context_t ctx;
   uart_lowlevel_config config = { "/dev/ttyUSB0" };

   ctx = ibus_init(&config);
   if(NULL == ctx)
   {
      ERR("Failed to initialize IBUS\n");
      return -1;
   }

   ibus_set_channel_handler(ctx, channel_handler, NULL);

   sleep(1000);

   ibus_deinit(ctx);

   return 0;
}
