#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include <xtensa_api.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "sys/lock.h"
#include "bt_i2s.h"
#include "bt_app_hf.h"
#include "codec.h"
#include "esp_timer.h"

#define BT_I2S_TAG "BT_I2S"

#define MSBC_FRAME_SAMPLES                      120  // mSBC uses 120 samples per frame (240 bytes)
#define HFP_SAMPLE_RATE                         16000
#define HFP_I2S_DATA_BIT_WIDTH                  I2S_DATA_BIT_WIDTH_16BIT
#define A2DP_STANDARD_SAMPLE_RATE               44100
#define A2DP_I2S_DATA_BIT_WIDTH                 I2S_DATA_BIT_WIDTH_16BIT
#define RINGBUF_HIGHEST_WATER_LEVEL             (32 * 1024)
#define RINGBUF_PREFETCH_WATER_LEVEL            (20 * 1024)
#define RINGBUF_HFP_TX_HIGHEST_WATER_LEVEL      (32 * MSBC_FRAME_SAMPLES * 2)
#define RINGBUF_HFP_TX_PREFETCH_WATER_LEVEL     (20 * MSBC_FRAME_SAMPLES * 2)
#define RINGBUF_HFP_RX_HIGHEST_WATER_LEVEL      (32 * ESP_HF_MSBC_ENCODED_FRAME_SIZE)
#define RINGBUF_HFP_RX_PREFETCH_WATER_LEVEL     (20 * ESP_HF_MSBC_ENCODED_FRAME_SIZE)


enum {
    RINGBUFFER_MODE_PROCESSING,    /* ringbuffer is buffering incoming audio data, I2S is working */
    RINGBUFFER_MODE_PREFETCHING,   /* ringbuffer is buffering incoming audio data, I2S is waiting */
    RINGBUFFER_MODE_DROPPING       /* ringbuffer is not buffering (dropping) incoming audio data, I2S is working */
};

enum {
    I2S_TX_MODE_NONE,   /* i2s tx isn't being used by a2dp or hfp */
    I2S_TX_MODE_A2DP,   /* i2s tx is being used by a2dp */
    I2S_TX_MODE_HFP     /* i2s tx is being used by hfp */
};

enum {
    I2S_RX_MODE_NONE,   /* i2s rx isn't being used by hfp */
    I2S_RX_MODE_HFP     /* i2s rx is being used by hfp */
};
/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/
// static QueueHandle_t s_bt_app_task_queue = NULL;                             /* handle of work queue */
// static TaskHandle_t s_bt_app_task_handle = NULL;                             /* handle of application task  */
static TaskHandle_t s_bt_i2s_a2dp_tx_task_handle = NULL;                        /* handle of a2dp I2S task */
static RingbufHandle_t s_i2s_a2dp_tx_ringbuf = NULL;                            /* handle of ringbuffer for I2S tx*/
// static SemaphoreHandle_t s_i2s_a2dp_tx_semaphore = NULL;                        /* handle of semaphore for a2dp I2S tx*/ // NOPE! Just a tx semaphore. tx is the same i2s for hfp and adp
static TaskHandle_t s_bt_i2s_hfp_rx_task_handle = NULL;                         /* handle of I2S rx task */
static bool s_bt_i2s_hfp_rx_task_running = false;                               /* running state of I2S hfp rx task */
static RingbufHandle_t s_i2s_hfp_rx_ringbuf = NULL;                             /* handle of ringbuffer for I2S hfp rx*/
static SemaphoreHandle_t s_i2s_hfp_rx_ringbuf_delete = NULL;                    /* hfp rx ringbuffer can be safely deleted after semaphore is given */
// static SemaphoreHandle_t s_i2s_hfp_rx_semaphore = NULL;                         /* handle of semaphore for hfp I2S rx */ // NOPE!
static TaskHandle_t s_bt_i2s_hfp_tx_task_handle = NULL;                         /* handle of I2S hfp tx task */
static bool s_bt_i2s_hfp_tx_task_running = false;                               /* running state of I2S hfp tx task */
static RingbufHandle_t s_i2s_hfp_tx_ringbuf = NULL;                             /* handle of ringbuffer for hfp I2S tx*/
static SemaphoreHandle_t s_i2s_hfp_tx_ringbuf_delete = NULL;                    /* hfp tx ringbuffer can be safely deleted after semaphore is given */
// static SemaphoreHandle_t s_i2s_hfp_tx_semaphore = NULL;                         /* handle of semaphore for hfp I2S tx */ // NOPE!
static uint16_t s_i2s_a2dp_tx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;    /* a2dp tx ringbuffer mode */
static uint16_t s_i2s_hfp_rx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;     /* hfp rx ringbuffer mode */
static uint16_t s_i2s_hfp_tx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;     /* hfp tx ringbuffer mode */
static uint16_t s_i2s_tx_mode = I2S_TX_MODE_NONE;
static uint16_t s_i2s_rx_mode = I2S_RX_MODE_NONE;
static SemaphoreHandle_t s_i2s_tx_semaphore = NULL;
static SemaphoreHandle_t s_i2s_rx_semaphore = NULL;
// static SemaphoreHandle_t s_i2s_tx_mode_semaphore = NULL;        /* handle of semaphore for I2S tx mode */
// static esp_timer_handle_t s_i2s_rx_timer = NULL;

/*  
    we initialize with default values here
*/
int A2DP_SAMPLE_RATE =           A2DP_STANDARD_SAMPLE_RATE; // this might be changed by a avrc event
int A2DP_CH_COUNT =              I2S_SLOT_MODE_STEREO;
bool tx_chan_running =           false; // do we need this? we can check if tx_chan is NULL
bool rx_chan_running =           false; // do we need this? we can check if rx_chan is NULL
I2S_pin_config i2sTxPinConfig = { 26, 17, 25, 0 };
I2S_pin_config i2sRxPinConfig = { 16, 27, 0, 14 };
// our channel handles
i2s_chan_handle_t tx_chan = NULL;
i2s_chan_handle_t rx_chan = NULL;
// semaphore handle for allowing a2dp to writing to i2s
// extern SemaphoreHandle_t s_i2s_tx_mode_semaphore;


/*
I2S setup and init
*/
// Functions to set our I2S pins
void bt_i2s_set_tx_I2S_pins(int bckPin, int wsPin, int doPin, int diPin) {
    i2sTxPinConfig.bck = bckPin;
    i2sTxPinConfig.ws = wsPin;
    i2sTxPinConfig.dout = doPin;
    i2sTxPinConfig.din = diPin;
    ESP_LOGI(BT_I2S_TAG, "setting tx GPIO Pins: BCK: %d WS: %d DOUT: %d DIN: %d ", i2sTxPinConfig.bck, i2sTxPinConfig.ws, i2sTxPinConfig.dout, i2sTxPinConfig.din);
}

void bt_i2s_set_rx_I2S_pins(int bckPin, int wsPin, int doPin, int diPin) {
    i2sRxPinConfig.bck = bckPin;
    i2sRxPinConfig.ws = wsPin;
    i2sRxPinConfig.dout = doPin;
    i2sRxPinConfig.din = diPin;
    ESP_LOGI(BT_I2S_TAG, "setting rx GPIO Pins: BCK: %d WS: %d DOUT: %d DIN: %d ", i2sRxPinConfig.bck, i2sRxPinConfig.ws, i2sRxPinConfig.dout, i2sRxPinConfig.din);
}

void bt_i2s_init() {
    if ((s_i2s_tx_semaphore = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE(BT_I2S_TAG, "%s, s_i2s_write_semaphore Semaphore create failed", __func__);
        return;
    }
    if ((s_i2s_rx_semaphore = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE(BT_I2S_TAG, "%s, s_i2s_read_semaphore Semaphore create failed", __func__);
        return;
    }

    if ((s_i2s_hfp_tx_ringbuf_delete = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE(BT_I2S_TAG, "%s, s_i2s_hfp_tx_ringbuf_delete Semaphore create failed", __func__);
        return;
    }
    if ((s_i2s_hfp_rx_ringbuf_delete = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE(BT_I2S_TAG, "%s, s_i2s_hfp_rx_ringbuf_delete Semaphore create failed", __func__);
        return;
    }
    //s_i2s_hfp_tx_ringbuf_delete
    bt_i2s_init_tx_chan();
    // bt_i2s_a2dp_task_init();
    // bt_i2s_a2dp_task_start_up();
    bt_i2s_init_rx_chan();
    // bt_i2s_hfp_task_init();
    // bt_i2s_hfp_task_start_up();
}


/*  
    I2S mgmnt
*/

i2s_std_clk_config_t bt_i2s_get_hfp_clk_cfg(void)
{
    i2s_std_clk_config_t hfp_clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(HFP_SAMPLE_RATE);
    ESP_LOGI(BT_I2S_TAG, "reconfiguring hfp clock to sample rate:  %d", HFP_SAMPLE_RATE);
    return hfp_clk_cfg;
}

i2s_std_slot_config_t bt_i2s_get_hfp_tx_slot_cfg(void)
{
    // decoded frame sample rate: 16000, bits per sample: 16, channel(s): 1, bitrate: 60800, frame size: 17
    // I2S_STD_PCM_SLOT_DEFAULT_CONFIG, I2S_STD_MSB_SLOT_DEFAULT_CONFIG, or I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG
    i2s_std_slot_config_t hfp_slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(HFP_I2S_DATA_BIT_WIDTH, I2S_SLOT_MODE_MONO);
    // hfp_slot_cfg.msb_right = true;
    hfp_slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    ESP_LOGI(BT_I2S_TAG, "reconfiguring hfp tx slot to data bit width:  %d", HFP_I2S_DATA_BIT_WIDTH);
    return hfp_slot_cfg;
}

i2s_std_clk_config_t bt_i2s_get_adp_clk_cfg(void)
{
    i2s_std_clk_config_t adp_clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(A2DP_SAMPLE_RATE);
    ESP_LOGI(BT_I2S_TAG, "reconfiguring adp clock to sample rate:  %d", A2DP_SAMPLE_RATE);
    return adp_clk_cfg;
}

i2s_std_slot_config_t bt_i2s_get_adp_slot_cfg(void)
{
    i2s_std_slot_config_t adp_slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(A2DP_I2S_DATA_BIT_WIDTH, I2S_SLOT_MODE_STEREO);
    ESP_LOGI(BT_I2S_TAG, "reconfiguring adp slot to data bit width:  %d", A2DP_I2S_DATA_BIT_WIDTH);
    return adp_slot_cfg;
}

// This is our tx channel (used for both ad2p sink and hfp tx)
void bt_i2s_init_tx_chan()
{
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL);
    i2s_std_config_t std_tx_cfg = {
        .clk_cfg = bt_i2s_get_adp_clk_cfg(),
        .slot_cfg = bt_i2s_get_adp_slot_cfg(),
        .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = i2sTxPinConfig.bck,
                .ws   = i2sTxPinConfig.ws,
                .dout = i2sTxPinConfig.dout,
                .din  = I2S_GPIO_UNUSED,
                .invert_flags = {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv   = false,
                },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_tx_cfg));
}

// This is our INMP441 mems microphone. left channel, so pin is low.
void bt_i2s_init_rx_chan()
{
    /* RX channel will be registered on our second I2S */
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan);
    // PHILIPS mode with MONO and 32-bit
    i2s_std_config_t std_rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(HFP_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = i2sRxPinConfig.bck,
            .ws = i2sRxPinConfig.ws,
            .dout = I2S_GPIO_UNUSED,
            .din = i2sRxPinConfig.din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_rx_cfg));
}

void bt_i2s_driver_install(void)
{
    ESP_LOGI(BT_I2S_TAG, "%s", __func__);
    bt_i2s_init_tx_chan();
    bt_i2s_init_rx_chan();
}

void bt_i2s_driver_uninstall(void)
{
    ESP_LOGI(BT_I2S_TAG, "%s", __func__);
    if (tx_chan_running)
        {
        bt_i2s_tx_channel_disable();
        ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
        ESP_LOGI(BT_I2S_TAG, "tx_chan pointer: %p", tx_chan);
        }
    if (rx_chan_running)
        {
        bt_i2s_rx_channel_disable();
        ESP_ERROR_CHECK(i2s_del_channel(rx_chan));
        ESP_LOGI(BT_I2S_TAG, "rx_chan pointer: %p", rx_chan);
        }
}

void bt_i2s_channels_disable(void)
{
    bt_i2s_rx_channel_disable();
    bt_i2s_tx_channel_disable();
}

void bt_i2s_tx_channel_enable(void)
{
    ESP_LOGI(BT_I2S_TAG, "%s", __func__);
    if (!tx_chan_running)
     {
        ESP_LOGI(BT_I2S_TAG, " -- not running; enabling now");
        ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
     }
    tx_chan_running = true;
}

void bt_i2s_tx_channel_disable(void)
{
    ESP_LOGI(BT_I2S_TAG, "%s", __func__);
    if (tx_chan_running)
     {
        ESP_LOGI(BT_I2S_TAG, " -- bt_i2s_tx_channel running; disabling now");
        ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
     }
    tx_chan_running = false;
}

void bt_i2s_rx_channel_enable(void)
{
    ESP_LOGI(BT_I2S_TAG, "%s", __func__);
    if (!rx_chan_running)
     {
        ESP_LOGI(BT_I2S_TAG, " -- not running; enabling now");
        ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
     }
    rx_chan_running = true;
}

void bt_i2s_rx_channel_disable(void)
{
    ESP_LOGI(BT_I2S_TAG, "%s", __func__);
    if (rx_chan_running)
     {
        ESP_LOGI(BT_I2S_TAG, " -- bt_i2s_rx_channel running; disabling now");
        ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
     }
    rx_chan_running = false;
}

void bt_i2s_tx_channel_reconfig_clock_slot(int sample_rate, int ch_count)
{
    A2DP_SAMPLE_RATE = sample_rate;
    A2DP_CH_COUNT = ch_count;
    bt_i2s_channels_config_adp();
}

void bt_i2s_audio_enable_adp(void)
{
    bt_i2s_channels_config_adp();
    bt_i2s_tx_channel_enable();
}

void bt_i2s_audio_disable_adp(void)
{
    bt_i2s_tx_channel_disable();
}

void bt_i2s_channels_config_adp(void)
{
    bool _isrunning = tx_chan_running; 
    i2s_std_clk_config_t clk_cfg = bt_i2s_get_adp_clk_cfg();
    i2s_std_slot_config_t slot_cfg = bt_i2s_get_adp_slot_cfg();
    bt_i2s_tx_channel_disable();
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg));
    if (_isrunning) {
        bt_i2s_tx_channel_enable();
    }
}

void bt_i2s_channels_config_hfp(void)
{
    bool _tx_is_running = tx_chan_running;

    i2s_std_clk_config_t clk_cfg = bt_i2s_get_hfp_clk_cfg();
    i2s_std_slot_config_t slot_cfg = bt_i2s_get_hfp_tx_slot_cfg();
    bt_i2s_tx_channel_disable();
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg));
    if (_tx_is_running) {
        bt_i2s_tx_channel_enable();
    }
}

/*  
    I2S tasks and buffers
*/

/*
A2DP
*/


/* 
    fetch audio data from the a2dp ringbuffer and write to i2s
 */
void bt_i2s_a2dp_tx_task_handler(void *arg)
{
    uint8_t *data = NULL;
    size_t item_size = 0;
    /**
     * The total length of DMA buffer of I2S is:
     * `dma_frame_num * dma_desc_num * i2s_channel_num * i2s_data_bit_width / 8`.
     * Transmit `dma_frame_num * dma_desc_num` bytes to DMA is trade-off.
     */
    const size_t item_size_upto = 240 * 6;
    size_t bytes_written = 0;
    for (;;) {
        if (pdTRUE == xSemaphoreTake(s_i2s_tx_semaphore, portMAX_DELAY)) {
            for (;;) {
                item_size = 0;
                /* receive data from ringbuffer and write it to I2S DMA transmit buffer */
                /* note: write_ringbuf always writes 4096 bytes to our ringbuffer */
                data = (uint8_t *)xRingbufferReceiveUpTo(s_i2s_a2dp_tx_ringbuf, &item_size, 0, item_size_upto);
                if (item_size == 0) {
                    ESP_LOGI(BT_I2S_TAG, "%s - tx ringbuffer underflowed! mode changed: RINGBUFFER_MODE_PREFETCHING", __func__);
                    s_i2s_a2dp_tx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
                    break;
                }
                if (s_i2s_tx_mode == I2S_TX_MODE_A2DP) { // we discard the data if we are not in a2dp mode
                    i2s_channel_write(tx_chan, data, item_size, &bytes_written, portMAX_DELAY);
                }
                vRingbufferReturnItem(s_i2s_a2dp_tx_ringbuf, (void *)data);
            }
        }
    }
}

/* 
    this sets up our a2dp ringbuffer, and (just) creates the a2dp tx task handler
 */
void bt_i2s_a2dp_task_init(void)
{
    // s_i2s_tx_mode_semaphore should this be here? we need to init it somewhere
    // if ((s_i2s_tx_mode_semaphore = xSemaphoreCreateBinary()) == NULL) {
    //     ESP_LOGE(BT_I2S_TAG, "%s, s_i2s_tx_mode_semaphore Semaphore create failed", __func__);
    //     return;
    // }
    // bt_app_set_i2s_tx_mode_none();
    ESP_LOGI(BT_I2S_TAG, "ringbuffer data empty! mode changed: RINGBUFFER_MODE_PREFETCHING");
    s_i2s_a2dp_tx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    if ((s_i2s_a2dp_tx_ringbuf = xRingbufferCreate(RINGBUF_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF)) == NULL) {
        ESP_LOGE(BT_I2S_TAG, "%s, ringbuffer create failed", __func__);
        return;
    }
    xTaskCreate(bt_i2s_a2dp_tx_task_handler, "BtI2Sa2dpTask", 2048, NULL, configMAX_PRIORITIES - 4, &s_bt_i2s_a2dp_tx_task_handle);
}

void bt_i2s_a2dp_task_deinit(void)
{
    if (s_bt_i2s_a2dp_tx_task_handle) {
        vTaskDelete(s_bt_i2s_a2dp_tx_task_handle);
        s_bt_i2s_a2dp_tx_task_handle = NULL;
    }
    if (s_i2s_a2dp_tx_ringbuf) {
        vRingbufferDelete(s_i2s_a2dp_tx_ringbuf);
        s_i2s_a2dp_tx_ringbuf = NULL;
    }
    //don't destroy the semaphore. do this in bt_i2s deinit
    // if (s_i2s_a2dp_tx_semaphore) {
    //     vSemaphoreDelete(s_i2s_a2dp_tx_semaphore);
    //     s_i2s_a2dp_tx_semaphore = NULL;
    // }
    // if (s_i2s_tx_mode_semaphore) {
    //     vSemaphoreDelete(s_i2s_tx_mode_semaphore);
    //     s_i2s_tx_mode_semaphore = NULL;
    // }
}

/* 
    start our a2dp tx task.
    this should be called after bt_i2s_task_init
 */
void bt_i2s_a2dp_task_start_up(void)
{
    
    bt_i2s_channels_config_adp();
    bt_i2s_tx_channel_enable();
    s_i2s_tx_mode = I2S_TX_MODE_A2DP;
}

/* 
    stop our a2dp tx task.
    this should be called before bt_i2s_task_deinit
 */
void bt_i2s_a2dp_task_shut_down(void)
{
    // if (s_bt_i2s_a2dp_tx_task_handle) {
    //     vTaskDelete(s_bt_i2s_a2dp_tx_task_handle);
    //     s_bt_i2s_a2dp_tx_task_handle = NULL;
    // }
    s_i2s_tx_mode = I2S_TX_MODE_NONE;
    bt_i2s_tx_channel_disable();
}


/* 
    this is our callback function that recieves the data
    and puts it in the tx ringbuffer
 */
void bt_i2s_a2dp_write_tx_ringbuf(const uint8_t *data, uint32_t size)
{
    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    if (s_i2s_a2dp_tx_ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        ESP_LOGW(BT_I2S_TAG, "%s - ringbuffer is full, drop this packet!", __func__);
        vRingbufferGetInfo(s_i2s_a2dp_tx_ringbuf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size <= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_I2S_TAG, "%s - ringbuffer data decreased! mode changed: RINGBUFFER_MODE_PROCESSING", __func__);
            s_i2s_a2dp_tx_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
        return;
    }
    // ESP_LOGI(BT_I2S_TAG, "%s - sending tx size %lu to the ringbuffer", __func__, size);
    done = xRingbufferSend(s_i2s_a2dp_tx_ringbuf, (void *)data, size, (TickType_t)0);

    if (!done) {
        ESP_LOGW(BT_I2S_TAG, "%s - ringbuffer overflowed, ready to decrease data! mode changed: RINGBUFFER_MODE_DROPPING", __func__);
        s_i2s_a2dp_tx_ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
    }

    if (s_i2s_a2dp_tx_ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_i2s_a2dp_tx_ringbuf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_I2S_TAG, "%s - ringbuffer data increased! mode changed: RINGBUFFER_MODE_PROCESSING", __func__);
            s_i2s_a2dp_tx_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
            if (pdFALSE == xSemaphoreGive(s_i2s_tx_semaphore)) {// we have taken the semaphore in our tx task(?)
                ESP_LOGE(BT_I2S_TAG, "%s - semphore give failed", __func__);
            }
        }
    }

    // return done ? size : 0;
}

/* 
    this sets up our hfp ringbuffers, and creates the hfp tx and rx and tasks
 */
void bt_i2s_hfp_task_init(void)
{
    s_i2s_hfp_tx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    s_i2s_tx_mode = I2S_TX_MODE_HFP;
    if ((s_i2s_hfp_tx_ringbuf = xRingbufferCreate(RINGBUF_HFP_TX_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF)) == NULL) {
        ESP_LOGE(BT_I2S_TAG, "%s, hfp tx ringbuffer create failed", __func__);
        return;
    }
    s_bt_i2s_hfp_tx_task_running = true;
    xTaskCreate(bt_i2s_hfp_tx_task_handler, "BtI2ShfpTxTask", 4096, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_hfp_tx_task_handle);
    
    s_i2s_hfp_rx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    s_i2s_rx_mode = I2S_RX_MODE_HFP;
    if ((s_i2s_hfp_rx_ringbuf = xRingbufferCreate(RINGBUF_HFP_RX_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF)) == NULL) {
        ESP_LOGE(BT_I2S_TAG, "%s, hfp rx ringbuffer create failed", __func__);
        return;
    }
    s_bt_i2s_hfp_rx_task_running = true;
    xTaskCreate(bt_i2s_hfp_rx_task_handler, "BtI2ShfpRxTask", 4096, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_hfp_rx_task_handle);
}

void bt_i2s_hfp_task_deinit(void)
{
    s_i2s_tx_mode = I2S_TX_MODE_NONE;
    s_i2s_rx_mode = I2S_RX_MODE_NONE;
    if (s_bt_i2s_hfp_tx_task_handle) {
        // vTaskDelete(s_bt_i2s_hfp_tx_task_handle);
        // task should kill itself
        s_bt_i2s_hfp_tx_task_running = false;
        s_bt_i2s_hfp_tx_task_handle = NULL;
    }
    if (pdTRUE == xSemaphoreTake(s_i2s_hfp_tx_ringbuf_delete, portMAX_DELAY)) {
        if (s_i2s_hfp_tx_ringbuf) {
            ESP_LOGI(BT_I2S_TAG, "%s, deleting s_i2s_hfp_tx_ringbuf", __func__);
            vRingbufferDelete(s_i2s_hfp_tx_ringbuf);
            s_i2s_hfp_tx_ringbuf = NULL;
        }
    }
    if (s_bt_i2s_hfp_rx_task_handle) {
        //vTaskDelete(s_bt_i2s_hfp_rx_task_handle);
        s_bt_i2s_hfp_rx_task_running = false;
        s_bt_i2s_hfp_rx_task_handle = NULL;
    }
    if (pdTRUE == xSemaphoreTake(s_i2s_hfp_rx_ringbuf_delete, portMAX_DELAY)) {
        if (s_i2s_hfp_rx_ringbuf) {
            ESP_LOGI(BT_I2S_TAG, "%s, deleting s_i2s_hfp_rx_ringbuf", __func__);
            vRingbufferDelete(s_i2s_hfp_rx_ringbuf);
            s_i2s_hfp_rx_ringbuf = NULL;
        }
    }
}

/* 
    fetch audio data from the hfp tx ringbuffer and write to i2s
 */
void bt_i2s_hfp_tx_task_handler(void *arg)
{
    uint8_t *data = NULL;
    size_t item_size = 0;
    const size_t item_size_upto = MSBC_FRAME_SAMPLES * 2;
    size_t bytes_written = 0;
    for (;;) {
        if (s_bt_i2s_hfp_tx_task_running) {
            if (s_i2s_hfp_tx_ringbuffer_mode != RINGBUFFER_MODE_PREFETCHING) {
                item_size = 0;
                /* receive data from ringbuffer and write it to I2S DMA transmit buffer */
                data = (uint8_t *)xRingbufferReceiveUpTo(s_i2s_hfp_tx_ringbuf, &item_size, 0, item_size_upto);
                if (item_size == 0) {
                    ESP_LOGI(BT_I2S_TAG, "%s - tx ringbuffer underflowed! mode changed: RINGBUFFER_MODE_PREFETCHING", __func__);
                    s_i2s_hfp_tx_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
                    goto Delay;
                }
                if (s_i2s_tx_mode == I2S_TX_MODE_HFP) { // we discard the data if we are not in hfp mode
                    // swap every pair of uint16_t as per:
                    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html#std-tx-mode
                    int16_t *send_data = (int16_t*) data;
                    // Swap every pair of samples
                    for(int i = 0; i < item_size/2; i += 2) {
                        int16_t temp = send_data[i];
                        send_data[i] = send_data[i + 1];
                        send_data[i + 1] = temp;
                    }
                    i2s_channel_write(tx_chan, send_data, item_size, &bytes_written, portMAX_DELAY);
                }
                vRingbufferReturnItem(s_i2s_hfp_tx_ringbuf, (void *)data);
            } else {
                Delay:
                    vTaskDelay(pdMS_TO_TICKS(40)); // give ringbuffer some time to prefetch
            }
        } else {
            // give semaphore so s_i2s_hfp_tx_ringbuf can be safely deleted
            xSemaphoreGive(s_i2s_hfp_tx_ringbuf_delete);
            ESP_LOGI(BT_I2S_TAG, "%s, deleting myself",__func__); 
            vTaskDelete(NULL);
        }
    }
}

/* 
    fetch audio (mic) data from i2s and put it in the rx ringbuffer
 */
void bt_i2s_hfp_rx_task_handler(void *arg)
{
    int32_t *i2s_buffer = malloc(MSBC_FRAME_SAMPLES * sizeof(int32_t));
    uint8_t *pcm_buffer = malloc(MSBC_FRAME_SAMPLES * 2);
    uint8_t *encoded_buffer = malloc(ESP_HF_MSBC_ENCODED_FRAME_SIZE);
    
    if (!i2s_buffer || !pcm_buffer || !encoded_buffer) {
        ESP_LOGE(BT_I2S_TAG, "Failed to allocate buffers");
    }
    
    size_t bytes_read;    
    
    while (1) {
        if (s_bt_i2s_hfp_rx_task_running) {
            esp_err_t ret = i2s_channel_read(rx_chan, i2s_buffer, 
                                            MSBC_FRAME_SAMPLES * sizeof(int32_t), 
                                            &bytes_read, portMAX_DELAY);
            
            if (ret == ESP_OK && bytes_read > 0) {
                i2s_32bit_to_16bit_pcm(i2s_buffer, pcm_buffer, MSBC_FRAME_SAMPLES);
                
                size_t encoded_len;
                if (msbc_enc_data(pcm_buffer, MSBC_FRAME_SAMPLES * 2, 
                                encoded_buffer, &encoded_len) == 0) {
                    bt_i2s_hfp_write_rx_ringbuf(encoded_buffer, ESP_HF_MSBC_ENCODED_FRAME_SIZE);
                }
            }
        } else { /* if (s_bt_i2s_hfp_rx_task_running) */
            free(i2s_buffer);
            free(pcm_buffer);
            free(encoded_buffer);
            // give semaphore so s_i2s_hfp_rx_ringbuf can be safely deleted
            xSemaphoreGive(s_i2s_hfp_rx_ringbuf_delete);
            ESP_LOGI(BT_I2S_TAG, "%s, deleting myself",__func__); 
            vTaskDelete(NULL);
        } /* if (s_bt_i2s_hfp_rx_task_running) */
    }
}

/* 
    this is called from hfp and recieves the decoded audio data
    and puts it in the tx ringbuffer
 */
void bt_i2s_hfp_write_tx_ringbuf(const uint8_t *data, uint32_t size)
{
    if (s_i2s_hfp_tx_ringbuf == NULL) {// ringbuffer hasn't been set up yet
        return;
    }
    
    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    if (s_i2s_hfp_tx_ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        ESP_LOGW(BT_I2S_TAG, "%s - hfp tx ringbuffer is full, drop this packet!", __func__);
        vRingbufferGetInfo(s_i2s_hfp_tx_ringbuf, NULL, NULL, NULL, NULL, &item_size);

        if (item_size <= RINGBUF_HFP_TX_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_I2S_TAG, "%s - hfp tx ringbuffer data decreased! (%d) mode changed: RINGBUFFER_MODE_PROCESSING", __func__, item_size);
            s_i2s_hfp_tx_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
        return;
    }

    done = xRingbufferSend(s_i2s_hfp_tx_ringbuf, (void *)data, size, (TickType_t)0);
    // ESP_LOGI(BT_I2S_TAG, "%s - hfp tx ringbuffer size: %d", __func__, item_size);

    if (!done) {
        ESP_LOGW(BT_I2S_TAG, "%s - hfp tx ringbuffer overflowed, ready to decrease data! mode changed: RINGBUFFER_MODE_DROPPING", __func__);
        s_i2s_hfp_tx_ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
    }

    if (s_i2s_hfp_tx_ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_i2s_hfp_tx_ringbuf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= RINGBUF_HFP_TX_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_I2S_TAG, "%s - hfp tx ringbuffer data increased! (%d) mode changed: RINGBUFFER_MODE_PROCESSING", __func__, item_size);
            s_i2s_hfp_tx_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        } else {
            // ESP_LOGI(BT_I2S_TAG, "%s - hfp tx ringbuffer size (%d) mode still: RINGBUFFER_MODE_PREFETCHING", __func__, item_size);
        }
    }
}

size_t i2s_hfp_rx_ringbuffer_total = 0;
size_t i2s_hfp_rx_ringbuffer_dropped = 0;
size_t i2s_hfp_rx_ringbuffer_sent = 0;
/* 
    this is our called from our i2s hfp rx task and recieves the (mic) audio data
    and puts it in the rx ringbuffer
 */
void bt_i2s_hfp_write_rx_ringbuf(unsigned char *data, uint32_t size)
{
    if (!s_i2s_hfp_rx_ringbuf) {
        return;
    }
    i2s_hfp_rx_ringbuffer_total += 1;
    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    if (s_i2s_hfp_rx_ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        // ESP_LOGW(BT_I2S_TAG, "%s - hfp rx ringbuffer is full, drop this packet!", __func__);
        vRingbufferGetInfo(s_i2s_hfp_rx_ringbuf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size <= RINGBUF_HFP_RX_HIGHEST_WATER_LEVEL) {
            // ESP_LOGI(BT_I2S_TAG, "%s - hfp rx ringbuffer data decreased! mode changed: RINGBUFFER_MODE_PROCESSING", __func__);
            s_i2s_hfp_rx_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
        i2s_hfp_rx_ringbuffer_dropped += 1;
        return;
    }
    done = xRingbufferSend(s_i2s_hfp_rx_ringbuf, (const char*)data, size, (TickType_t)0);

    if (!done) {
        // ESP_LOGW(BT_I2S_TAG, "%s - hfp rx ringbuffer overflowed, ready to decrease data! mode changed: RINGBUFFER_MODE_DROPPING", __func__);
        s_i2s_hfp_rx_ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
        i2s_hfp_rx_ringbuffer_dropped += 1;
    } else {
        i2s_hfp_rx_ringbuffer_sent += 1;
    }

    if (s_i2s_hfp_rx_ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_i2s_hfp_rx_ringbuf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= RINGBUF_HFP_RX_PREFETCH_WATER_LEVEL) {
            // ESP_LOGI(BT_I2S_TAG, "%s - hfp rx ringbuffer data increased! mode changed: RINGBUFFER_MODE_PROCESSING", __func__);
            s_i2s_hfp_rx_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
    }
    // Log every 1000 calls
    if (i2s_hfp_rx_ringbuffer_total % 1000 == 0) {
        ESP_LOGI(BT_I2S_TAG, "%s - calls: %d sent: %d dropped: %d", 
            __func__, i2s_hfp_rx_ringbuffer_total, i2s_hfp_rx_ringbuffer_sent, i2s_hfp_rx_ringbuffer_dropped);
    }
}

/* 
    this is called from hfp client for getting the (mic) audio data from the rx ringbuffer
 */
size_t bt_i2s_hfp_read_rx_ringbuf(uint8_t *mic_data)
{
    // ringbuffer needs to exist
    if (!s_i2s_hfp_rx_ringbuf) {
        return 0;
    }
    size_t item_size = 0;
    if (s_i2s_hfp_rx_ringbuffer_mode != RINGBUFFER_MODE_PREFETCHING) {
        uint8_t *ringbuf_data = xRingbufferReceiveUpTo(s_i2s_hfp_rx_ringbuf, &item_size, 10000, ESP_HF_MSBC_ENCODED_FRAME_SIZE);
        // ESP_LOGI(BT_I2S_TAG, "%s - read %d bytes from ringbuffer, expected %d", __func__, item_size, ESP_HF_MSBC_ENCODED_FRAME_SIZE);
        
        memcpy(mic_data, ringbuf_data, item_size);
        vRingbufferReturnItem(s_i2s_hfp_rx_ringbuf, (void *)ringbuf_data);
    }
    return item_size;
}

void bt_i2s_hfp_start()
{
    s_i2s_tx_mode = I2S_TX_MODE_HFP;
    bt_i2s_channels_config_hfp();
    bt_i2s_tx_channel_enable();
    bt_i2s_rx_channel_enable();
    msbc_dec_open();
    msbc_enc_open();
    bt_i2s_hfp_task_init();
}

void bt_i2s_hfp_stop()
{
    bt_i2s_hfp_task_deinit();
    bt_i2s_channels_disable();
    msbc_dec_close();
    msbc_enc_close();
    // bt_i2s_channels_config_adp();
    s_i2s_tx_mode = I2S_TX_MODE_NONE;
}
