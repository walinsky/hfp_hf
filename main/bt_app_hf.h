/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef __BT_APP_HF_H__
#define __BT_APP_HF_H__

#include <stdint.h>
#include "esp_hf_client_api.h"
#include "esp_audio_dec.h"
#include "esp_audio_enc.h"

#define BT_HF_TAG               "BT_HF"

/**
 * @brief     callback function for HF client
 */
void bt_app_hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param);
bool hfp_sbc_decoder(uint8_t *data, uint16_t data_len, esp_audio_dec_out_frame_t *out_frame);
bool hfp_sbc_encoder(uint8_t *data, uint16_t data_len, esp_audio_enc_out_frame_t *out_frame);

static void heap_monitor_task(void *pvParameters);
static void kill_hfp_audio_task(void *pvParameters);


#endif /* __BT_APP_HF_H__*/
