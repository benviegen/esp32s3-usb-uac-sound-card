#include <stdio.h>
#include "usb_device_uac.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include <stdint.h>
#include "esp_timer.h"

i2s_chan_handle_t tx_handle;

i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

//TODO : adjust clock and slot config to match USB audio exactly
// check current ticks and see how many samples should have been send to keep up, and how many were actually sent
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html#_CPPv421i2s_channel_tune_rate17i2s_chan_handle_tPK19i2s_tuning_config_tP17i2s_tuning_info_t
// use for tuning based on usb input rate

i2s_std_config_t std_cfg = {
    // still not exactly correctly matched, but closer (depends on port)
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(47995), //the 48khz of the esp is faster than the 48khz of the pc 
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = GPIO_NUM_4,
        .ws = GPIO_NUM_5,
        .dout = GPIO_NUM_6,
        .din = I2S_GPIO_UNUSED,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
        },
    },
};

static volatile float volume_factor = 1.0f; // default: full volume (gain)
static volatile bool muted = false;
static const char *TAG = "usb-i2s";

/* Timestamp of last uac output callback (microseconds from esp_timer_get_time) */
static int64_t last_output_cb_time_us = 0; 




static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    int16_t *samples = (int16_t *)buf; // 16-bit audio
    size_t sample_count = len / sizeof(int16_t);

    /* Estimate incoming sample rate by measuring time between callbacks */
    static int64_t fps_accum_us = 0;
    static uint64_t frames_accum = 0;

    int64_t now_us = esp_timer_get_time();
    if (last_output_cb_time_us != 0) {
        int64_t delta_us = now_us - last_output_cb_time_us;
        if (delta_us > 0) {
            size_t frames = sample_count / 2; /* stereo frames in this callback */
            frames_accum += frames;
            fps_accum_us += delta_us;

            if (fps_accum_us >= 1000000) { /* report once per second */
                double avg_fps = (double)frames_accum * 1e6 / (double)fps_accum_us;
                ESP_LOGI(TAG, "avg fps: %.2f", avg_fps);
                frames_accum = 0;
                fps_accum_us = 0;
            }
        }
    }
    last_output_cb_time_us = now_us;

    /* If muted or gain is zero, write silence. Otherwise apply gain.
       We operate on per-sample (int16_t) data; sample_count is number of int16 samples. */
    if (muted || volume_factor <= 0.0f) {
        memset(buf, 0, len);
    } else {
        float gain = volume_factor;
        for (size_t i = 0; i < sample_count; i++) {
            /* multiply sample by gain and clamp to int16 range */
            // int diff = samples[i] - prev_sample;
            
            // if (diff > 100 || diff < -100) {
            //     ESP_LOGI(TAG, "i%u: d%d, c%d, p%d", (unsigned)i, diff, (int)samples[i], prev_sample);
            // }
            
            // prev_sample = samples[i];

            int32_t temp = (int32_t)(samples[i] * gain);

            if (temp > INT16_MAX) temp = INT16_MAX;
            if (temp < INT16_MIN) temp = INT16_MIN;

            samples[i] = (int16_t)temp;

        }
    }

    size_t bytes_written = 0;
    i2s_channel_write(tx_handle, buf, len, &bytes_written, 1);

    return ESP_OK;
}


static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    return ESP_OK;
}

static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
    muted = (mute != 0);
    ESP_LOGI(TAG, "set_mute_cb: mute=%u", (unsigned)mute);
}

static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    /* Map USB 0..255 volume to a perceptual gain (linear in dB).
       0 -> silence, 255 -> 0 dB (gain=1.0). We map linearly in dB
       from min_db to 0dB so that perceived loudness is more natural. */
    const float min_db = -10.0f; /* floor dB for zero-like */
    if (volume == 0) {
        volume_factor = 0.0f;
        ESP_LOGI(TAG, "set_volume_cb: volume=0 -> mute-equivalent, gain=0");
    } else {
        float norm = (float)volume / 100.0f; /* 0..1 */
        float db = min_db + norm * (0.0f - min_db); /* interp from min_db to 0dB */
        volume_factor = powf(10.0f, db / 20.0f);
        if (volume_factor > 1.0f) volume_factor = 1.0f;
        if (volume_factor < 0.0f) volume_factor = 0.0f;
        ESP_LOGI(TAG, "set_volume_cb: volume=%u -> db=%.2f dB -> gain=%.6f", (unsigned)volume, db, (double)volume_factor);
    }
}


void app_main(void)
{
    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .cb_ctx = NULL,
    };
    uac_device_init(&config);
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);
    i2s_channel_init_std_mode(tx_handle, &std_cfg);

    /* Before writing data, start the TX channel first */
    i2s_channel_enable(tx_handle);
}