/* Created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Uses the RMT peripheral on the ESP32 for very accurate timing of
 * signals sent to the WS2812 LEDs.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include "ws2812.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <soc/rmt_struct.h>
#include <soc/dport_reg.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include <esp_intr_alloc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <driver/rmt.h>

#define DIVIDER		4 /* Above 4, timings start to deviate*/
#define DURATION	12.5 /* minimum time of a single RMT duration
				in nanoseconds based on clock */

#define PULSE_T0H	(  350 / (DURATION * DIVIDER));
#define PULSE_T1H	(  900 / (DURATION * DIVIDER));
#define PULSE_T0L	(  900 / (DURATION * DIVIDER));
#define PULSE_T1L	(  350 / (DURATION * DIVIDER));
#define PULSE_TRS	(50000 / (DURATION * DIVIDER));

#define MAX_PULSES	32

typedef union {
  struct {
    uint32_t duration0:15;
    uint32_t level0:1;
    uint32_t duration1:15;
    uint32_t level1:1;
  };
  uint32_t val;
} rmtPulsePair;

static uint8_t *ws2812_buffer[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static unsigned int ws2812_pos[8], ws2812_len[8], ws2812_half[8];
static xSemaphoreHandle ws2812_sem[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static intr_handle_t rmt_intr_handle = NULL;
static rmtPulsePair ws2812_bits[2];

void ws2812_initRMTChannel(int rmtChannel)
{
  RMT.apb_conf.fifo_mask = 1;  //enable memory access, instead of FIFO mode.
  RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer
  RMT.conf_ch[rmtChannel].conf0.div_cnt = DIVIDER;
  RMT.conf_ch[rmtChannel].conf0.mem_size = 1;
  RMT.conf_ch[rmtChannel].conf0.carrier_en = 0;
  RMT.conf_ch[rmtChannel].conf0.carrier_out_lv = 1;
  RMT.conf_ch[rmtChannel].conf0.mem_pd = 0;

  RMT.conf_ch[rmtChannel].conf1.rx_en = 0;
  RMT.conf_ch[rmtChannel].conf1.mem_owner = 0;
  RMT.conf_ch[rmtChannel].conf1.tx_conti_mode = 0;    //loop back mode.
  RMT.conf_ch[rmtChannel].conf1.ref_always_on = 1;    // use apb clock: 80M
  RMT.conf_ch[rmtChannel].conf1.idle_out_en = 1;
  RMT.conf_ch[rmtChannel].conf1.idle_out_lv = 0;

  return;
}

void ws2812_copy(int RMTChannel)
{
  unsigned int i, j, offset, len, bit;


  offset = ws2812_half[RMTChannel] * MAX_PULSES;
  ws2812_half[RMTChannel] = !ws2812_half[RMTChannel];

  len = ws2812_len[RMTChannel] - ws2812_pos[RMTChannel];
  if (len > (MAX_PULSES / 8))
    len = (MAX_PULSES / 8);

  if (!len) {
    for (i = 0; i < MAX_PULSES; i++)
      RMTMEM.chan[RMTChannel].data32[i + offset].val = 0;
    return;
  }

  for (i = 0; i < len; i++) {
    bit = ws2812_buffer[RMTChannel][i + ws2812_pos[RMTChannel]];
    for (j = 0; j < 8; j++, bit <<= 1) {
      RMTMEM.chan[RMTChannel].data32[j + i * 8 + offset].val =
	ws2812_bits[(bit >> 7) & 0x01].val;
    }
    if (i + ws2812_pos[RMTChannel] == ws2812_len[RMTChannel] - 1)
      RMTMEM.chan[RMTChannel].data32[7 + i * 8 + offset].duration1 = PULSE_TRS;
  }

  for (i *= 8; i < MAX_PULSES; i++)
    RMTMEM.chan[RMTChannel].data32[i + offset].val = 0;

  ws2812_pos[RMTChannel] += len;
  return;
}

void ws2812_handleInterrupt(void *arg)
{
  portBASE_TYPE taskAwoken = 0;


  if (RMT.int_st.ch0_tx_thr_event) {
    ws2812_copy(0);
    RMT.int_clr.ch0_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch0_tx_end && ws2812_sem[0]) {
    xSemaphoreGiveFromISR(ws2812_sem[0], &taskAwoken);
    RMT.int_clr.ch0_tx_end = 1;
  }
  if (RMT.int_st.ch1_tx_thr_event) {
    ws2812_copy(1);
    RMT.int_clr.ch1_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch1_tx_end && ws2812_sem[1]) {
    xSemaphoreGiveFromISR(ws2812_sem[1], &taskAwoken);
    RMT.int_clr.ch1_tx_end = 1;
  }
  if (RMT.int_st.ch2_tx_thr_event) {
    ws2812_copy(2);
    RMT.int_clr.ch2_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch2_tx_end && ws2812_sem[2]) {
    xSemaphoreGiveFromISR(ws2812_sem[2], &taskAwoken);
    RMT.int_clr.ch2_tx_end = 1;
  }
  if (RMT.int_st.ch3_tx_thr_event) {
    ws2812_copy(3);
    RMT.int_clr.ch3_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch3_tx_end && ws2812_sem[3]) {
    xSemaphoreGiveFromISR(ws2812_sem[3], &taskAwoken);
    RMT.int_clr.ch3_tx_end = 1;
  }
  if (RMT.int_st.ch4_tx_thr_event) {
    ws2812_copy(4);
    RMT.int_clr.ch4_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch4_tx_end && ws2812_sem[4]) {
    xSemaphoreGiveFromISR(ws2812_sem[4], &taskAwoken);
    RMT.int_clr.ch4_tx_end = 1;
  }
  if (RMT.int_st.ch5_tx_thr_event) {
    ws2812_copy(5);
    RMT.int_clr.ch5_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch5_tx_end && ws2812_sem[5]) {
    xSemaphoreGiveFromISR(ws2812_sem[5], &taskAwoken);
    RMT.int_clr.ch5_tx_end = 1;
  }
  if (RMT.int_st.ch6_tx_thr_event) {
    ws2812_copy(6);
    RMT.int_clr.ch6_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch6_tx_end && ws2812_sem[6]) {
    xSemaphoreGiveFromISR(ws2812_sem[6], &taskAwoken);
    RMT.int_clr.ch6_tx_end = 1;
  }
  if (RMT.int_st.ch7_tx_thr_event) {
    ws2812_copy(7);
    RMT.int_clr.ch7_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch7_tx_end && ws2812_sem[7]) {
    xSemaphoreGiveFromISR(ws2812_sem[7], &taskAwoken);
    RMT.int_clr.ch7_tx_end = 1;
  }

  return;
}

void ws2812_init(int gpioNum, int RMTChannel)
{
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);

  rmt_set_pin((rmt_channel_t)RMTChannel, RMT_MODE_TX, (gpio_num_t)gpioNum);

  ws2812_initRMTChannel(RMTChannel);

  RMT.tx_lim_ch[RMTChannel].limit = MAX_PULSES;
  switch(RMTChannel) {
    case 0: {
      RMT.int_ena.ch0_tx_thr_event = 1;
      RMT.int_ena.ch0_tx_end = 1;
      break;
    }
    case 1: {
      RMT.int_ena.ch1_tx_thr_event = 1;
      RMT.int_ena.ch1_tx_end = 1;
      break;
    }
    case 2: {
      RMT.int_ena.ch2_tx_thr_event = 1;
      RMT.int_ena.ch2_tx_end = 1;
      break;
    }
    case 3: {
      RMT.int_ena.ch3_tx_thr_event = 1;
      RMT.int_ena.ch3_tx_end = 1;
      break;
    }
    case 4: {
      RMT.int_ena.ch4_tx_thr_event = 1;
      RMT.int_ena.ch4_tx_end = 1;
      break;
    }
    case 5: {
      RMT.int_ena.ch5_tx_thr_event = 1;
      RMT.int_ena.ch5_tx_end = 1;
      break;
    }
    case 6: {
      RMT.int_ena.ch6_tx_thr_event = 1;
      RMT.int_ena.ch6_tx_end = 1;
      break;
    }
    case 7: {
      RMT.int_ena.ch7_tx_thr_event = 1;
      RMT.int_ena.ch7_tx_end = 1;
      break;
    }
  }
  

  ws2812_bits[0].level0 = 1;
  ws2812_bits[0].level1 = 0;
  ws2812_bits[0].duration0 = PULSE_T0H;
  ws2812_bits[0].duration1 = PULSE_T0L;
  ws2812_bits[1].level0 = 1;
  ws2812_bits[1].level1 = 0;
  ws2812_bits[1].duration0 = PULSE_T1H;
  ws2812_bits[1].duration1 = PULSE_T1L;

  esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, ws2812_handleInterrupt, NULL, &rmt_intr_handle);

  return;
}

void ws2812_setColors(unsigned int length, rgbVal *array, int RMTChannel)
{
  unsigned int i;

  ws2812_len[RMTChannel] = (length * 3) * sizeof(uint8_t);
  ws2812_buffer[RMTChannel] = malloc(ws2812_len[RMTChannel]);

  for (i = 0; i < length; i++) {
    ws2812_buffer[RMTChannel][0 + i * 3] = array[i].g;
    ws2812_buffer[RMTChannel][1 + i * 3] = array[i].r;
    ws2812_buffer[RMTChannel][2 + i * 3] = array[i].b;
  }

  ws2812_pos[RMTChannel] = 0;
  ws2812_half[RMTChannel] = 0;

  ws2812_copy(RMTChannel);

  if (ws2812_pos[RMTChannel] < ws2812_len[RMTChannel])
    ws2812_copy(RMTChannel);

  ws2812_sem[RMTChannel] = xSemaphoreCreateBinary();

  RMT.conf_ch[RMTChannel].conf1.mem_rd_rst = 1;
  RMT.conf_ch[RMTChannel].conf1.tx_start = 1;

  xSemaphoreTake(ws2812_sem[RMTChannel], portMAX_DELAY);
  vSemaphoreDelete(ws2812_sem[RMTChannel]);
  ws2812_sem[RMTChannel] = NULL;

  free(ws2812_buffer[RMTChannel]);

  return;
}