/* \copyright 2024 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ibus library esp-idf example application
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ibus.h"

/* The following definition may change,
   based on the ESP device and IBUS device wiring. */
#define ESP_UART UART_NUM_1

void app_main(void)
{
   ESP_ERROR_CHECK(nvs_flash_init());
   ESP_ERROR_CHECK(esp_netif_init());
   ESP_ERROR_CHECK(esp_event_loop_create_default() );

   ibus_lowlevel_config config;
   config.port = ESP_UART;
   ibus_context *ctx = ibus_init(&config);
   if(NULL != ctx)
   {
      ibus_deinit(ctx);
   }

   for(;;)
      vTaskDelay(portMAX_DELAY);
}
