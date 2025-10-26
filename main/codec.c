#include "codec.h"
#include "esp_log.h"
#include "esp_sbc_enc.h"
#include "esp_sbc_dec.h"
#include <string.h>

static const char *TAG = "CODEC";

// mSBC Configuration Constants
#define MSBC_SAMPLE_RATE        16000
#define MSBC_CHANNELS           1
#define MSBC_BITS_PER_SAMPLE    16
#define MSBC_FRAME_SAMPLES      120        // Changed from 240 to 120
#define MSBC_FRAME_SIZE_BYTES   (MSBC_FRAME_SAMPLES * 2)  // 240 bytes
#define MSBC_ENCODED_SIZE       120  // Increased from 60 to 120 bytes

// Static handles for encoder and decoder
static void *encoder_handle = NULL;
static void *decoder_handle = NULL;

int msbc_enc_open(void)
{
    if (encoder_handle != NULL) {
        ESP_LOGW(TAG, "Encoder already open");
        return 0;
    }

    #ifdef ESP_SBC_MSBC_ENC_CONFIG_DEFAULT
        esp_sbc_enc_config_t enc_cfg = ESP_SBC_MSBC_ENC_CONFIG_DEFAULT();
    #else
        esp_sbc_enc_config_t enc_cfg = {
            .sbc_mode = ESP_SBC_MODE_MSBC,
            .allocation_method = ESP_SBC_AM_LOUDNESS,
            .ch_mode = ESP_SBC_CH_MODE_MONO,
            .sample_rate = MSBC_SAMPLE_RATE,
            .bits_per_sample = MSBC_BITS_PER_SAMPLE,
            .bitpool = 26,
            .block_length = 15,
            .sub_bands_num = 8,
        };
    #endif

    int ret = esp_sbc_enc_open(&enc_cfg, sizeof(esp_sbc_enc_config_t), &encoder_handle);
    if (ret != 0 || encoder_handle == NULL) {
        ESP_LOGE(TAG, "Failed to open mSBC encoder, error: %d", ret);
        encoder_handle = NULL;
        return -1;
    }

    ESP_LOGI(TAG, "mSBC encoder opened successfully");
    return 0;
}

void msbc_enc_close(void)
{
    if (encoder_handle != NULL) {
        esp_sbc_enc_close(encoder_handle);
        encoder_handle = NULL;
        ESP_LOGI(TAG, "mSBC encoder closed");
    }
}

int msbc_dec_open(void)
{
    if (decoder_handle != NULL) {
        ESP_LOGW(TAG, "Decoder already open");
        return 0;
    }

    esp_sbc_dec_cfg_t dec_cfg = {
        .sbc_mode = ESP_SBC_MODE_MSBC,
        .ch_num = MSBC_CHANNELS,
        .enable_plc = 1,
    };
    
    ESP_LOGI(TAG, "Opening decoder with: mode=%d, ch_num=%d, plc=%d", 
             dec_cfg.sbc_mode, dec_cfg.ch_num, dec_cfg.enable_plc);

    int ret = esp_sbc_dec_open(&dec_cfg, sizeof(esp_sbc_dec_cfg_t), &decoder_handle);
    if (ret != 0 || decoder_handle == NULL) {
        ESP_LOGE(TAG, "Failed to open mSBC decoder, error: %d", ret);
        decoder_handle = NULL;
        return -1;
    }

    ESP_LOGI(TAG, "mSBC decoder opened successfully");
    return 0;
}

void msbc_dec_close(void)
{
    if (decoder_handle != NULL) {
        esp_sbc_dec_close(decoder_handle);
        decoder_handle = NULL;
        ESP_LOGI(TAG, "mSBC decoder closed");
    }
}

int msbc_enc_data(const uint8_t *in_data, size_t in_data_len, 
                  uint8_t *out_data, size_t *out_data_len)
{
    if (in_data == NULL || out_data == NULL || out_data_len == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for encoding");
        return -1;
    }

    if (encoder_handle == NULL) {
        ESP_LOGE(TAG, "Encoder not initialized. Call msbc_enc_open() first");
        return -1;
    }

    if (in_data_len != MSBC_FRAME_SIZE_BYTES) {
        ESP_LOGW(TAG, "Input data length %zu is not optimal for mSBC (expected %d)", 
                 in_data_len, MSBC_FRAME_SIZE_BYTES);
    }

    // Prepare input frame
    esp_audio_enc_in_frame_t in_frame = {
        .buffer = (uint8_t *)in_data,
        .len = in_data_len,
    };

    // Prepare output frame with sufficient buffer size
    esp_audio_enc_out_frame_t out_frame = {
        .buffer = out_data,
        .len = MSBC_ENCODED_SIZE,  // 120 bytes
    };

    // Encode the data
    int ret = esp_sbc_enc_process(encoder_handle, &in_frame, &out_frame);
    
    if (ret != 0) {
        ESP_LOGE(TAG, "Encoding failed, error: %d", ret);
        return -1;
    }

    *out_data_len = out_frame.len;
    ESP_LOGD(TAG, "Encoded %zu bytes to %d bytes", in_data_len, out_frame.len);
    return 0;
}

int msbc_dec_data(const uint8_t *in_data, size_t in_data_len, 
                  uint8_t *out_data, size_t *out_data_len)
{
    if (in_data == NULL || out_data == NULL || out_data_len == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for decoding");
        return -1;
    }

    if (decoder_handle == NULL) {
        ESP_LOGE(TAG, "Decoder not initialized. Call msbc_dec_open() first");
        return -1;
    }

    // Prepare input frame
    esp_audio_dec_in_raw_t in_frame = {
        .buffer = (uint8_t *)in_data,
        .len = in_data_len,
    };

    // Prepare output frame with buffer size
    esp_audio_dec_out_frame_t out_frame = {
        .buffer = out_data,
        .len = MSBC_FRAME_SIZE_BYTES * 2,  // Buffer capacity
        .decoded_size = 0,
    };

    // Prepare info structure
    esp_audio_dec_info_t dec_info = {0};

    // Decode the data
    int ret = esp_sbc_dec_decode(decoder_handle, &in_frame, &out_frame, &dec_info);
    
    if (ret != 0) {
        ESP_LOGE(TAG, "Decoding failed, error: %d", ret);
        return -1;
    }

    *out_data_len = out_frame.decoded_size;
    ESP_LOGD(TAG, "Decoded %zu bytes to %d bytes", in_data_len, out_frame.decoded_size);
    return 0;
}

/* void i2s_32bit_to_16bit_pcm(const int32_t *i2s_data, uint8_t *pcm_data, size_t num_samples)
{
    if (i2s_data == NULL || pcm_data == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for I2S conversion");
        return;
    }

    int16_t *output = (int16_t *)pcm_data;
    
    for (size_t i = 0; i < num_samples; i++) {
        // INMP441: 24-bit data left-aligned in 32-bit word
        // Shift right by 16 to get most significant 16 bits
        output[i] = (int16_t)(i2s_data[i] >> 16);
    }
} */
void i2s_32bit_to_16bit_pcm(const int32_t *i2s_data, uint8_t *pcm_data, size_t num_samples)
{
    uint8_t *input_bytes = (uint8_t *)i2s_data;
    
    for (size_t i = 0; i < num_samples; i++) {
        // Extract bytes [2] and [3] from each 32-bit word
        pcm_data[i * 2] = input_bytes[i * 4 + 2];      // mid byte
        pcm_data[i * 2 + 1] = input_bytes[i * 4 + 3];  // high byte
    }
}