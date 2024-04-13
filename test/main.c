/* \copyright 2024 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ibus library Linux example application
 */
#include <stdio.h>
#include "helpers.h"
#include "ibus.h"

int main(int argc, char *argv[])
{
   ibus_context_t ctx;
   ibus_lowlevel_config config = { "/dev/ttyUSB0" };

   ctx = ibus_init(&config);
   if(NULL == ctx)
   {
      IBUSERR("Failed to initialize IBUS\n");
      return -1;
   }

   ibus_deinit(ctx);

   return 0;
}