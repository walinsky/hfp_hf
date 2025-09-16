/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"

#include "bt_app_core.h"
#include "bt_app_hf.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
// #include "esp_pbac_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "time.h"
#include "sys/time.h"
#include "sdkconfig.h"

#include "esp_sbc_def.h"
// #include "esp_audio_dec.h"
#include "esp_audio_dec_reg.h"
// #include "esp_audio_enc.h"
#include "esp_audio_enc_reg.h"
#include "esp_sbc_dec.h"
#include "esp_sbc_enc.h"
#include "esp_hf_defs.h"

#include "bt_i2s.h"




const char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< connection state changed event */
    "AUDIO_STATE_EVT",                   /*!< audio connection state change event */
    "VR_STATE_CHANGE_EVT",                /*!< voice recognition state changed */
    "CALL_IND_EVT",                      /*!< call indication event */
    "CALL_SETUP_IND_EVT",                /*!< call setup indication event */
    "CALL_HELD_IND_EVT",                 /*!< call held indicator event */
    "NETWORK_STATE_EVT",                 /*!< network state change event */
    "SIGNAL_STRENGTH_IND_EVT",           /*!< signal strength indication event */
    "ROAMING_STATUS_IND_EVT",            /*!< roaming status indication event */
    "BATTERY_LEVEL_IND_EVT",             /*!< battery level indication event */
    "CURRENT_OPERATOR_EVT",              /*!< current operator name event */
    "RESP_AND_HOLD_EVT",                 /*!< response and hold event */
    "CLIP_EVT",                          /*!< Calling Line Identification notification event */
    "CALL_WAITING_EVT",                  /*!< call waiting notification */
    "CLCC_EVT",                          /*!< listing current calls event */
    "VOLUME_CONTROL_EVT",                /*!< audio volume control event */
    "AT_RESPONSE",                       /*!< audio volume control event */
    "SUBSCRIBER_INFO_EVT",               /*!< subscriber information event */
    "INBAND_RING_TONE_EVT",              /*!< in-band ring tone settings */
    "LAST_VOICE_TAG_NUMBER_EVT",         /*!< requested number from AG event */
    "RING_IND_EVT",                      /*!< ring indication event */
    "PKT_STAT_EVT",                      /*!< requested number of packet status event */
    "PROF_STATE_EVT",                    /*!< Indicate HF CLIENT init or deinit complete */
};

// esp_hf_client_connection_state_t
const char *c_connection_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "slc_connected",
    "disconnecting",
};

// esp_hf_client_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

/// esp_hf_vr_state_t
const char *c_vr_state_str[] = {
    "disabled",
    "enabled",
};

// esp_hf_service_availability_status_t
const char *c_service_availability_status_str[] = {
    "unavailable",
    "available",
};

// esp_hf_roaming_status_t
const char *c_roaming_status_str[] = {
    "inactive",
    "active",
};

// esp_hf_client_call_state_t
const char *c_call_str[] = {
    "NO call in progress",
    "call in progress",
};

// esp_hf_client_callsetup_t
const char *c_call_setup_str[] = {
    "NONE",
    "INCOMING",
    "OUTGOING_DIALING",
    "OUTGOING_ALERTING"
};

// esp_hf_client_callheld_t
const char *c_call_held_str[] = {
    "NONE held",
    "Held and Active",
    "Held",
};

// esp_hf_response_and_hold_status_t
const char *c_resp_and_hold_str[] = {
    "HELD",
    "HELD ACCEPTED",
    "HELD REJECTED",
};

// esp_hf_client_call_direction_t
const char *c_call_dir_str[] = {
    "outgoing",
    "incoming",
};

// esp_hf_client_call_state_t
const char *c_call_state_str[] = {
    "active",
    "held",
    "dialing",
    "alerting",
    "incoming",
    "waiting",
    "held_by_resp_hold",
};

// esp_hf_current_call_mpty_type_t
const char *c_call_mpty_type_str[] = {
    "single",
    "multi",
};

// esp_hf_volume_control_target_t
const char *c_volume_control_target_str[] = {
    "SPEAKER",
    "MICROPHONE"
};

// esp_hf_at_response_code_t
const char *c_at_response_code_str[] = {
    "OK",
    "ERROR"
    "ERR_NO_CARRIER",
    "ERR_BUSY",
    "ERR_NO_ANSWER",
    "ERR_DELAYED",
    "ERR_BLACKLILSTED",
    "ERR_CME",
};

// esp_hf_subscriber_service_type_t
const char *c_subscriber_service_type_str[] = {
    "unknown",
    "voice",
    "fax",
};

// esp_hf_client_in_band_ring_state_t
const char *c_inband_ring_state_str[] = {
    "NOT provided",
    "Provided",
};

extern esp_bd_addr_t peer_addr;
// If you want to connect a specific device, add it's address here
// esp_bd_addr_t peer_addr = {0xac, 0x67, 0xb2, 0x53, 0x77, 0xbe};

extern i2s_chan_handle_t tx_chan;
extern i2s_chan_handle_t rx_chan;

static TaskHandle_t s_hfp_heap_monitor_task_handle = NULL; 
static bool s_hfp_heap_monitor_task_running = false;

static esp_hf_sync_conn_hdl_t s_sync_conn_hdl;
static bool s_msbc_air_mode = false;
static bool s_hfp_audio_connected = false;
// QueueHandle_t s_audio_buff_queue = NULL;
// static int s_audio_buff_cnt = 0;



static int s_audio_callback_cnt = 0;
static void bt_app_hf_client_audio_data_cb(esp_hf_sync_conn_hdl_t sync_conn_hdl, esp_hf_audio_buff_t *audio_buf, bool is_bad_frame)
{
    if (!s_hfp_audio_connected) {
        esp_hf_client_audio_buff_free(audio_buf);
        return;
    }
    if (is_bad_frame) {
        esp_hf_client_audio_buff_free(audio_buf);
        return;
    }

    /* decode our incoming data and send it to i2s tx ringbuffer */
    esp_audio_dec_out_frame_t out_frame = {0};
    if (hfp_sbc_decoder(audio_buf->data, audio_buf->data_len, &out_frame)) {
        bt_i2s_hfp_write_tx_ringbuf(out_frame.buffer, out_frame.len);
        esp_hf_client_audio_buff_free(audio_buf);
    } else {
        esp_hf_client_audio_buff_free(audio_buf);
    }
    
    /* fetch our msbc encoded mic data and send it to the ag */
    uint8_t *mic_data = (uint8_t *) malloc (ESP_HF_MSBC_ENCODED_FRAME_SIZE);
    size_t mic_data_len = bt_i2s_hfp_read_rx_ringbuf(mic_data);
    if (mic_data_len == 0 || mic_data == NULL) {
        return;
    }
    esp_hf_audio_buff_t *audio_data_to_send = esp_hf_client_audio_buff_alloc((uint16_t) mic_data_len);
    memcpy (audio_data_to_send->data, mic_data, mic_data_len);
    audio_data_to_send->data_len = mic_data_len;
    free (mic_data);
    if (s_msbc_air_mode && audio_data_to_send->data_len > ESP_HF_MSBC_ENCODED_FRAME_SIZE) {
        audio_data_to_send->data_len = ESP_HF_MSBC_ENCODED_FRAME_SIZE;
    }
    if (esp_hf_client_audio_data_send(s_sync_conn_hdl, audio_data_to_send) != ESP_OK) {
        esp_hf_client_audio_buff_free(audio_data_to_send);
        ESP_LOGW(BT_HF_TAG, "%s failed to send audio data", __func__);
    }
    if (s_audio_callback_cnt % 1000 == 0) {
        esp_hf_client_pkt_stat_nums_get(sync_conn_hdl);
    }
    s_audio_callback_cnt++;
}


/* callback for HF_CLIENT */
void bt_app_hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    if (event <= ESP_HF_CLIENT_PROF_STATE_EVT) {
        ESP_LOGI(BT_HF_TAG, "APP HFP event: %s", c_hf_evt_str[event]);
    } else {
        ESP_LOGE(BT_HF_TAG, "APP HFP invalid event %d", event);
    }

    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--connection state %s, peer feats 0x%"PRIx32", chld_feats 0x%"PRIx32,
                    c_connection_state_str[param->conn_stat.state],
                    param->conn_stat.peer_feat,
                    param->conn_stat.chld_feat);
            memcpy(peer_addr,param->conn_stat.remote_bda,ESP_BD_ADDR_LEN);
            if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED) {
                // esp_pbac_connect(peer_addr);
            }
            break;
        }

        case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--audio state %s",
                    c_audio_state_str[param->audio_stat.state]);
    #if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI && CONFIG_BT_HFP_USE_EXTERNAL_CODEC
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                s_msbc_air_mode = true;
                ESP_LOGI(BT_HF_TAG, "--audio air mode: mSBC , preferred_frame_size: %d", param->audio_stat.preferred_frame_size);
            }
            else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED) {
                s_msbc_air_mode = false;
                ESP_LOGI(BT_HF_TAG, "--audio air mode: CVSD , preferred_frame_size: %d", param->audio_stat.preferred_frame_size);
            }

            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                s_sync_conn_hdl = param->audio_stat.sync_conn_handle;
                s_hfp_heap_monitor_task_running = true;
                s_hfp_audio_connected = true;
                xTaskCreate(&heap_monitor_task, "HeapMonitor", 4096, NULL, 5, &s_hfp_heap_monitor_task_handle);
                bt_i2s_hfp_start();
                esp_hf_client_register_audio_data_callback(bt_app_hf_client_audio_data_cb);
            } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
                ESP_LOGI(BT_HF_TAG, "%s ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED", __func__);
                s_sync_conn_hdl = 0;
                s_msbc_air_mode = false;
                s_hfp_audio_connected = false;
                static TaskHandle_t s_hfp_kill_audio_task_handle;
                xTaskCreate(&kill_hfp_audio_task, "Kill HPF AUDIO", 4096, NULL, 5, &s_hfp_kill_audio_task_handle);
            }
    #endif /* #if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI && CONFIG_BT_HFP_USE_EXTERNAL_CODEC */
            break;
        }

        case ESP_HF_CLIENT_BVRA_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--VR state %s",
                    c_vr_state_str[param->bvra.value]);
            break;
        }

        case ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--NETWORK STATE %s",
                    c_service_availability_status_str[param->service_availability.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--ROAMING: %s",
                    c_roaming_status_str[param->roaming.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "-- signal strength: %d",
                    param->signal_strength.value);
            break;
        }

        case ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--battery level %d",
                    param->battery_level.value);
            break;
        }

        case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--operator name: %s",
                    param->cops.name);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Call indicator %s",
                    c_call_str[param->call.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Call setup indicator %s",
                    c_call_setup_str[param->call_setup.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Call held indicator %s",
                    c_call_held_str[param->call_held.status]);
            break;
        }

        case ESP_HF_CLIENT_BTRH_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--response and hold %s",
                    c_resp_and_hold_str[param->btrh.status]);
            break;
        }

        case ESP_HF_CLIENT_CLIP_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--clip number %s",
                    (param->clip.number == NULL) ? "NULL" : (param->clip.number));
            break;
        }

        case ESP_HF_CLIENT_CCWA_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--call_waiting %s",
                    (param->ccwa.number == NULL) ? "NULL" : (param->ccwa.number));
            break;
        }

        case ESP_HF_CLIENT_CLCC_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Current call: idx %d, dir %s, state %s, mpty %s, number %s",
                    param->clcc.idx,
                    c_call_dir_str[param->clcc.dir],
                    c_call_state_str[param->clcc.status],
                    c_call_mpty_type_str[param->clcc.mpty],
                    (param->clcc.number == NULL) ? "NULL" : (param->clcc.number));
            break;
        }

        case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--volume_target: %s, volume %d",
                    c_volume_control_target_str[param->volume_control.type],
                    param->volume_control.volume);
            break;
        }

        case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--AT response event, code %d, cme %d",
                    param->at_response.code, param->at_response.cme);
            break;
        }

        case ESP_HF_CLIENT_CNUM_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--subscriber type %s, number %s",
                    c_subscriber_service_type_str[param->cnum.type],
                    (param->cnum.number == NULL) ? "NULL" : param->cnum.number);
            break;
        }

        case ESP_HF_CLIENT_BSIR_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--inband ring state %s",
                    c_inband_ring_state_str[param->bsir.state]);
            break;
        }

        case ESP_HF_CLIENT_BINP_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--last voice tag number: %s",
                    (param->binp.number == NULL) ? "NULL" : param->binp.number);
            break;
        }
        case ESP_HF_CLIENT_PKT_STAT_NUMS_GET_EVT:
        {
            // struct hf_client_pkt_status_nums {
            //     uint32_t rx_total;        /*!< the total number of packets received */
            //     uint32_t rx_correct;      /*!< the total number of packets data correctly received */
            //     uint32_t rx_err;          /*!< the total number of packets data with possible invalid */
            //     uint32_t rx_none;         /*!< the total number of packets data no received */
            //     uint32_t rx_lost;         /*!< the total number of packets data partially lost */
            //     uint32_t tx_total;        /*!< the total number of packets send */
            //     uint32_t tx_discarded;    /*!< the total number of packets send lost */
            // } pkt_nums;                   /*!< HF callback param of ESP_HF_CLIENT_PKT_STAT_NUMS_GET_EVT */

            ESP_LOGI(BT_HF_TAG, "total packets: %d, received ok: %d, received err: %d, received none: %d, received lost: %d, sent: %d, sent lost: %d", 
                param->pkt_nums.rx_total, param->pkt_nums.rx_correct, param->pkt_nums.rx_err, param->pkt_nums.rx_none, param->pkt_nums.rx_lost,
                param->pkt_nums.tx_total, param->pkt_nums.tx_discarded);
            break;
        }
        case ESP_HF_CLIENT_PROF_STATE_EVT:
        {
            if (ESP_HF_INIT_SUCCESS == param->prof_stat.state) {
                ESP_LOGI(BT_HF_TAG, "HF PROF STATE: Init Complete");
            } else if (ESP_HF_DEINIT_SUCCESS == param->prof_stat.state) {
                ESP_LOGI(BT_HF_TAG, "HF PROF STATE: Deinit Complete");
            } else {
                ESP_LOGE(BT_HF_TAG, "HF PROF STATE error: %d", param->prof_stat.state);
            }
            break;
        }
        default:
            ESP_LOGE(BT_HF_TAG, "HF_CLIENT EVT: %d", event);
            break;
    }
}

int hfp_sbc_decoder_counter = 0;
bool hfp_sbc_decoder(uint8_t *data, uint16_t data_len, esp_audio_dec_out_frame_t *out_frame)
{
    // int inbuf_sz = data_len;
    int outbuf_sz = 240;
    uint8_t *inbuf = malloc(data_len);
    uint8_t *outbuf = malloc(outbuf_sz);
    esp_audio_dec_in_raw_t in_frame = {0};
    esp_audio_dec_info_t info = {0};

    in_frame.buffer = data;
    in_frame.len = data_len;
    in_frame.consumed = 0;
    in_frame.frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE;
    out_frame->buffer = outbuf;
    out_frame->len = outbuf_sz;

    esp_sbc_dec_cfg_t hfp_dec_cfg = {
        // .ch_num = 1,
        .enable_plc = true,
        .sbc_mode = ESP_SBC_MODE_MSBC,
    };
    void *hfp_dec_handle = NULL;

    if (esp_sbc_dec_open(&hfp_dec_cfg, sizeof(esp_sbc_dec_cfg_t), &hfp_dec_handle) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(BT_HF_TAG, "%s, could not open sbc decoder", __func__);
    };
    int dec_ret = 0;
    dec_ret = esp_sbc_dec_decode(hfp_dec_handle, &in_frame, out_frame, &info);
    if (hfp_sbc_decoder_counter % 1000 == 0) {
        ESP_LOGI(BT_HF_TAG, "%s decoded #%d frame sample rate: %d, bits per sample: %d, channel(s): %d, bitrate: %d, frame size: %d",
             __func__, hfp_sbc_decoder_counter, info.sample_rate, info.bits_per_sample, info.channel, info.bitrate, info.frame_size);
    }
    hfp_sbc_decoder_counter++;
    if (dec_ret != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(BT_HF_TAG, "could not decode: %d", dec_ret);
        free(inbuf);
        free(outbuf);
        return false;
    };
    if (esp_sbc_dec_close(hfp_dec_handle) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(BT_HF_TAG, "%s, could not close sbc decoder", __func__);
    };
    free(inbuf);
    free(outbuf);
    return true;
}

int hfp_sbc_encoder_counter = 0;
bool hfp_sbc_encoder(uint8_t *data, uint16_t data_len, esp_audio_enc_out_frame_t *out_frame)
{
    int inbuf_sz = 0;
    int outbuf_sz = 0;
    esp_sbc_enc_config_t hfp_enc_cfg = ESP_SBC_MSBC_ENC_CONFIG_DEFAULT();
    void *hfp_enc_handle = NULL;
    if (esp_sbc_enc_open(&hfp_enc_cfg, sizeof(esp_sbc_enc_config_t), &hfp_enc_handle) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(BT_HF_TAG, "%s, could not open sbc encoder", __func__);
    };
    if (esp_sbc_enc_get_frame_size(hfp_enc_handle, &inbuf_sz, &outbuf_sz) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(BT_HF_TAG, "%s, could not get frame size", __func__);
        return false;
    };
    uint8_t *inbuf = calloc(1, inbuf_sz);
    uint8_t *outbuf = calloc(1, outbuf_sz);
    esp_audio_enc_in_frame_t in_frame = {0};
    in_frame.buffer = data;
    in_frame.len = data_len;
    out_frame->buffer = outbuf;
    out_frame->len = outbuf_sz;
    int enc_ret = esp_sbc_enc_process(hfp_enc_handle, &in_frame, out_frame);
    hfp_sbc_encoder_counter++;
    if (enc_ret != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(BT_HF_TAG, "%s, could not encode", __func__);
        free(inbuf);
        free(outbuf);
        return false;
    } else {
        if (hfp_sbc_encoder_counter % 1000 == 0) {
            esp_audio_enc_info_t *enc_info = malloc(sizeof *enc_info);
            esp_sbc_enc_get_info(hfp_enc_handle, enc_info);
            ESP_LOGI(BT_HF_TAG, "%s encoded #%d frame sample rate: %d, bits per sample: %d, channel(s): %d, bitrate: %d",
                    __func__, hfp_sbc_encoder_counter, enc_info->sample_rate, enc_info->bits_per_sample, enc_info->channel, enc_info->bitrate);
            free(enc_info);
        }
    }
    esp_sbc_enc_close(hfp_enc_handle);
    free(inbuf);
    free(outbuf);
    return true;
}

static void heap_monitor_task(void *pvParameters) {
    for (;;) {
        // Get the number of free bytes in the heap
        int heap_size = esp_get_free_heap_size();
        ESP_LOGI(BT_HF_TAG, "%s, heap size: %d",__func__, heap_size);   
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_hfp_heap_monitor_task_running){
            ESP_LOGI(BT_HF_TAG, "%s, deleting myself",__func__); 
            vTaskDelete(NULL);
        }
    }
}

static void kill_hfp_audio_task(void *pvParameters) {
    ESP_LOGI(BT_HF_TAG, "%s stopping I2S hfp", __func__);
    bt_i2s_hfp_stop();
    ESP_LOGI(BT_HF_TAG, "%s stopping monitor task", __func__);
    s_hfp_heap_monitor_task_running = false;
    // vTaskDelete(s_hfp_heap_monitor_task_handle);
    ESP_LOGI(BT_HF_TAG, "%s, deleting myself",__func__); 
    vTaskDelete(NULL);
}