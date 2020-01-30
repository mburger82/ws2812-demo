/* Created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Demo of driving WS2812 RGB LEDs using the RMT peripheral.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/rmt_struct.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <stdio.h>
#include "ws2812.h"

#define TAG "WS2812-DEMO"

#define LEDSTRIPE1_RMTCHANNEL	0
#define LEDSTRIPE2_RMTCHANNEL	1
#define LEDSTRIPE3_RMTCHANNEL	2
#define LEDSTRIPE4_RMTCHANNEL	3
#define LEDSTRIPE5_RMTCHANNEL	4
#define LEDSTRIPE6_RMTCHANNEL	5
#define LEDSTRIPE7_RMTCHANNEL	6
#define LEDSTRIPE8_RMTCHANNEL	7

#define LEDSTRIPE1_RMT_TX_GPIO 19
#define LEDSTRIPE2_RMT_TX_GPIO 18
#define LEDSTRIPE3_RMT_TX_GPIO 17
#define LEDSTRIPE4_RMT_TX_GPIO 16
#define LEDSTRIPE5_RMT_TX_GPIO 04
#define LEDSTRIPE6_RMT_TX_GPIO 02
#define LEDSTRIPE7_RMT_TX_GPIO 15
#define LEDSTRIPE8_RMT_TX_GPIO 13

#define delay_ms(ms) vTaskDelay((ms) / portTICK_RATE_MS)

void rainbow(void *pvParameters)
{
  uint8_t rainbowChannel = (((uint32_t)pvParameters) & 0x00FF);
  ESP_LOGI(TAG, "Start rainbow Task nr: %i", rainbowChannel);
  
  const uint8_t anim_step = 10;
  const uint8_t anim_max = 250;
  const uint8_t pixel_count = 200; // Number of your "pixels"
  const uint8_t delay = 25; // duration between color changes
  rgbVal color = makeRGBVal(anim_max, 0, 0);
  uint8_t step = 0;
  rgbVal color2 = makeRGBVal(anim_max, 0, 0);
  uint8_t step2 = 0;
  rgbVal *pixels;


  pixels = malloc(sizeof(rgbVal) * pixel_count);

  while (1) {
    color = color2;
    step = step2;

    for (uint8_t i = 0; i < pixel_count; i++) {
      pixels[i] = color;

      if (i == 1) {
        color2 = color;
        step2 = step;
      }

      switch (step) {
      case 0:
        color.g += anim_step;
        if (color.g >= anim_max)
          step++;
        break;
      case 1:
        color.r -= anim_step;
        if (color.r == 0)
          step++;
        break;
      case 2:
        color.b += anim_step;
        if (color.b >= anim_max)
          step++;
        break;
      case 3:
        color.g -= anim_step;
        if (color.g == 0)
          step++;
        break;
      case 4:
        color.r += anim_step;
        if (color.r >= anim_max)
          step++;
        break;
      case 5:
        color.b -= anim_step;
        if (color.b == 0)
          step = 0;
        break;
      }
    }

    ws2812_setColors(pixel_count, pixels, rainbowChannel);
    

    delay_ms(delay*(rainbowChannel+1));
  }
}

void app_main()
{
  nvs_flash_init();

  ws2812_init(LEDSTRIPE1_RMT_TX_GPIO, LEDSTRIPE1_RMTCHANNEL);
  ws2812_init(LEDSTRIPE2_RMT_TX_GPIO, LEDSTRIPE2_RMTCHANNEL);
  ws2812_init(LEDSTRIPE3_RMT_TX_GPIO, LEDSTRIPE3_RMTCHANNEL);
  ws2812_init(LEDSTRIPE4_RMT_TX_GPIO, LEDSTRIPE4_RMTCHANNEL);
  ws2812_init(LEDSTRIPE5_RMT_TX_GPIO, LEDSTRIPE5_RMTCHANNEL);
  ws2812_init(LEDSTRIPE6_RMT_TX_GPIO, LEDSTRIPE6_RMTCHANNEL);
  ws2812_init(LEDSTRIPE7_RMT_TX_GPIO, LEDSTRIPE7_RMTCHANNEL);
  ws2812_init(LEDSTRIPE8_RMT_TX_GPIO, LEDSTRIPE8_RMTCHANNEL);

  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE1_RMTCHANNEL, 10, NULL);
  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE2_RMTCHANNEL, 11, NULL);
  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE3_RMTCHANNEL, 12, NULL);
  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE4_RMTCHANNEL, 13, NULL);
  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE5_RMTCHANNEL, 14, NULL);
  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE6_RMTCHANNEL, 15, NULL);
  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE7_RMTCHANNEL, 16, NULL);
  xTaskCreate(rainbow, "ws2812 rainbow demo 1", 3*2048, (void*)LEDSTRIPE8_RMTCHANNEL, 17, NULL);

  return;
}
