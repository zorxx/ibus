/* \copyright 2024 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ibus library esp-idf example application
 */
#include <freertos/FreeRTOS.h>
#include <driver/uart.h>
#include "esp_event.h"
#include "esp_log.h"
#include "ibus/ibus.h"

#define TAG "ibus_example"

static void channel_handler(ibus_channel_t *channels, void *cookie)
{
   ESP_LOGD(TAG, "Channel update: %04x:%04x:%04x;%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
   channels[0].value, channels[1].value, channels[2].value, channels[3].value,
   channels[4].value, channels[5].value, channels[6].value, channels[7].value,
   channels[8].value, channels[9].value, channels[10].value, channels[11].value,
   channels[12].value, channels[13].value);
}

/* The following definition may change,
   based on the ESP device and IBUS device wiring. */
#define IBUS_UART   UART_NUM_1
#define IBUS_TX_PIN 4
#define IBUS_RX_PIN 5

void app_main(void)
{
   ESP_ERROR_CHECK(esp_event_loop_create_default() );

   uart_lowlevel_config config;
   config.port = IBUS_UART;
   config.tx_pin = IBUS_TX_PIN;
   config.rx_pin = IBUS_RX_PIN;
   ibus_context_t *ctx = ibus_init(&config);
   if(NULL != ctx)
   {
      ibus_set_channel_handler(ctx, channel_handler, NULL);
   }

   for(;;)
      vTaskDelay(portMAX_DELAY);

   ibus_deinit(ctx);
}
