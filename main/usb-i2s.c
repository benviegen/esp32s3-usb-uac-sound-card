#include <stdio.h>
#include "usb_device_uac.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include <stdint.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

i2s_chan_handle_t tx_handle;

i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

//TODO : adjust clock and slot config to match USB audio exactly
// check current ticks and see how many samples should have been send to keep up, and how many were actually sent
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html#_CPPv421i2s_channel_tune_rate17i2s_chan_handle_tPK19i2s_tuning_config_tP17i2s_tuning_info_t
// use for tuning based on usb input rate

i2s_std_config_t std_cfg = {
    // still not exactly correctly matched, but closer
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(47995), //the 48khz of the esp is faster than the 48khz of the pc by about 0.02% so use 47990 to compensate
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

/* Tuning request state. The USB callback will set these and the background
   task will perform the actual I2S tuning using i2s_channel_tune_rate(). */
static volatile bool tuning_requested = false;
static volatile int32_t tuning_delta_mclk = 0;

/* Background task prototype */
static void i2s_tuning_task(void *arg);





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

                /* If measured sample rate drifts from configured rate, schedule tuning.
                   We use a small relative threshold to avoid jittery updates. */
                double configured_rate = (double)std_cfg.clk_cfg.sample_rate_hz;
                double rel_err = (avg_fps - configured_rate) / configured_rate;
                const double REL_TOL = 1e-4; /* 0.01% threshold */
                if (fabs(rel_err) > REL_TOL) {
                    /* Query current MCLK and compute desired change (Hz). Use ADDSUB mode. */
                    i2s_tuning_info_t info;
                    esp_err_t qerr = i2s_channel_tune_rate(tx_handle, NULL, &info);
                    if (qerr == ESP_OK) {
                        double desired_mclk = (double)info.curr_mclk_hz * (avg_fps / configured_rate);
                        int32_t delta = (int32_t)round(desired_mclk - (double)info.curr_mclk_hz);
                        if (delta != 0) {
                            tuning_delta_mclk = delta;
                            tuning_requested = true;
                            ESP_LOGI(TAG, "request tuning: avg_fps=%.2f configured=%.2f delta_mclk=%d", avg_fps, configured_rate, delta);
                        }
                    } else {
                        ESP_LOGW(TAG, "could not query current I2S MCLK (err=%d), will request tuning anyway", qerr);
                        /* schedule best-effort tuning using measured ratio and current configured sample rate */
                        /* compute an approximate delta based on configured sample rate and known mclk multiple */
                        /* Fallback: assume MCLK = configured_rate * default mclk_multiple (256) */
                        const int default_mclk_mult = 256; /* assume mclk multiple 256 for fallback */
                        int32_t assumed_curr_mclk = (int32_t)(configured_rate * (double)default_mclk_mult);
                        double desired_mclk = (double)assumed_curr_mclk * (avg_fps / configured_rate);
                        int32_t delta = (int32_t)round(desired_mclk - (double)assumed_curr_mclk);
                        if (delta != 0) {
                            tuning_delta_mclk = delta;
                            tuning_requested = true;
                            ESP_LOGI(TAG, "fallback request tuning: avg_fps=%.2f configured=%.2f delta_mclk=%d", avg_fps, configured_rate, delta);
                        }
                    }
                }

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

/* Background task that performs the I2S tuning when requested by the USB
   audio callback. It tries to call i2s_channel_tune_rate directly; if tuning
   requires stopping the channel it will stop/start it temporarily. */
static void i2s_tuning_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!tuning_requested) {
            continue;
        }

        /* consume the request */
        tuning_requested = false;
        int32_t delta = tuning_delta_mclk;
        if (delta == 0) {
            ESP_LOGI(TAG, "tuning requested but delta is 0, skipping");
            continue;
        }

        i2s_tuning_config_t cfg = {
            .tune_mode = I2S_TUNING_MODE_ADDSUB,
            .tune_mclk_val = delta,
            .max_delta_mclk = 100000, /* clamp range in Hz */
            .min_delta_mclk = -100000,
        };

        i2s_tuning_info_t result;
        esp_err_t err = i2s_channel_tune_rate(tx_handle, &cfg, &result);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Tuning applied: curr_mclk=%d delta_mclk=%d water_mark=%u", result.curr_mclk_hz, result.delta_mclk_hz, result.water_mark);
            continue;
        }

        /* If tuning failed due to running state or unsupported, try by stopping channel */
        ESP_LOGW(TAG, "i2s_channel_tune_rate returned %d, attempting tuning with channel stop", err);
        i2s_channel_disable(tx_handle);
        err = i2s_channel_tune_rate(tx_handle, &cfg, &result);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Tuning applied after stopping channel: curr_mclk=%d delta_mclk=%d", result.curr_mclk_hz, result.delta_mclk_hz);
        } else {
            ESP_LOGE(TAG, "Tuning failed even after stopping channel: %d", err);
        }
        i2s_channel_enable(tx_handle);
    }
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
    const float min_db = -80.0f; /* floor dB for zero-like */
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

    /* Start background tuning task (checks tuning_requested) */
    xTaskCreate(i2s_tuning_task, "i2s_tune", 2048, NULL, 5, NULL);
}