#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb_device_uac.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"


/*
 * ============================================================
 * Configuration
 * ============================================================
 */

/*
 * I2S output:
 *
 *   BCLK = GPIO 4
 *   WS   = GPIO 5
 *   DATA = GPIO 6
 *
 * Audio:
 *
 *   48 kHz
 *   stereo
 *   16-bit
 */
#define I2S_BCLK_GPIO      GPIO_NUM_4
#define I2S_WS_GPIO        GPIO_NUM_5
#define I2S_DOUT_GPIO      GPIO_NUM_6

#define AUDIO_SAMPLE_RATE  48000


/*
 * ============================================================
 * Globals
 * ============================================================
 */

static const char *TAG = "usb-i2s";

static i2s_chan_handle_t tx_handle = NULL;

static volatile float volume_factor = 1.0f;
static volatile bool muted = false;


/*
 * ============================================================
 * I2S configuration
 * ============================================================
 */

static esp_err_t init_i2s(void)
{
    /*
     * Create I2S TX channel.
     *
     * IMPORTANT:
     *
     * The I2S channel remains running continuously.
     *
     * We do NOT use i2s_channel_tune_rate().
     * We do NOT disable/re-enable I2S to synchronize USB.
     *
     * USB feedback is responsible for making the PC follow
     * the actual ESP32 audio clock.
     */
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_AUTO,
            I2S_ROLE_MASTER
        );

    esp_err_t err = i2s_new_channel(
        &chan_cfg,
        &tx_handle,
        NULL
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "i2s_new_channel() failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * Standard I2S configuration.
     *
     * We intentionally use 48000 here.
     *
     * The ESP32 clock will not necessarily generate an
     * absolutely exact 48000 Hz physical sample rate.
     *
     * That is OK.
     *
     * The USB asynchronous feedback mechanism compensates
     * for the difference between the ESP32 clock and the PC
     * clock.
     */
    i2s_std_config_t std_cfg = {
        .clk_cfg =
            I2S_STD_CLK_DEFAULT_CONFIG(
                AUDIO_SAMPLE_RATE
            ),

        .slot_cfg =
            I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_16BIT,
                I2S_SLOT_MODE_STEREO
            ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,

            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_WS_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };


    err = i2s_channel_init_std_mode(
        tx_handle,
        &std_cfg
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "i2s_channel_init_std_mode() failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * Start I2S.
     *
     * From this point onward the I2S clock remains running.
     */
    err = i2s_channel_enable(tx_handle);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "i2s_channel_enable() failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    ESP_LOGI(
        TAG,
        "I2S started: %d Hz, stereo, 16-bit",
        AUDIO_SAMPLE_RATE
    );

    return ESP_OK;
}


/*
 * ============================================================
 * USB UAC output callback
 * ============================================================
 *
 * This is called when USB audio data is available.
 *
 * IMPORTANT:
 *
 * Do not attempt to tune the I2S clock here.
 *
 * The I2S clock remains fixed and continuously running.
 *
 * The USB Audio asynchronous feedback endpoint tells the
 * computer how quickly the ESP32 is actually consuming data.
 *
 * Therefore the PC gradually changes the number of samples
 * it sends per USB frame.
 *
 * This prevents the FIFO from slowly overflowing or
 * underrunning due to the two independent clocks.
 */
static esp_err_t uac_device_output_cb(
    uint8_t *buf,
    size_t len,
    void *arg)
{
    (void)arg;

    if (buf == NULL || len == 0) {
        return ESP_OK;
    }


    /*
     * USB audio is:
     *
     *   stereo
     *   16-bit
     *
     * Therefore every sample is one int16_t and every stereo
     * frame contains two int16_t values.
     */
    int16_t *samples = (int16_t *)buf;

    size_t sample_count =
        len / sizeof(int16_t);


    /*
     * --------------------------------------------------------
     * Mute / volume
     * --------------------------------------------------------
     */

    if (muted || volume_factor <= 0.0f) {

        memset(
            buf,
            0,
            len
        );

    } else {

        float gain = volume_factor;

        for (size_t i = 0; i < sample_count; i++) {

            int32_t temp =
                (int32_t)(
                    (float)samples[i] * gain
                );

            /*
             * Saturate to signed 16-bit.
             */
            if (temp > INT16_MAX) {
                temp = INT16_MAX;
            }

            if (temp < INT16_MIN) {
                temp = INT16_MIN;
            }

            samples[i] = (int16_t)temp;
        }
    }


    /*
     * --------------------------------------------------------
     * Write USB audio data to I2S
     * --------------------------------------------------------
     *
     * Use a blocking write.
     *
     * We do NOT restart or retune I2S here.
     */
    size_t bytes_written = 0;

    esp_err_t err = i2s_channel_write(
        tx_handle,
        buf,
        len,
        &bytes_written,
        portMAX_DELAY
    );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "i2s_channel_write() failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * This should normally never happen with a blocking write.
     */
    if (bytes_written != len) {

        ESP_LOGW(
            TAG,
            "I2S short write: %u/%u bytes",
            (unsigned)bytes_written,
            (unsigned)len
        );
    }

    return ESP_OK;
}


/*
 * ============================================================
 * USB UAC input callback
 * ============================================================
 *
 * We are not implementing a USB microphone in this project.
 */
static esp_err_t uac_device_input_cb(
    uint8_t *buf,
    size_t len,
    size_t *bytes_read,
    void *arg)
{
    (void)buf;
    (void)len;
    (void)arg;

    if (bytes_read != NULL) {
        *bytes_read = 0;
    }

    return ESP_OK;
}


/*
 * ============================================================
 * USB mute callback
 * ============================================================
 */
static void uac_device_set_mute_cb(
    uint32_t mute,
    void *arg)
{
    (void)arg;

    muted = (mute != 0);

    ESP_LOGI(
        TAG,
        "USB mute: %s",
        muted ? "ON" : "OFF"
    );
}


/*
 * ============================================================
 * USB volume callback
 * ============================================================
 *
 * The UAC component provides volume in the range used by
 * your existing implementation.
 *
 * We convert it to a linear gain.
 */
static void uac_device_set_volume_cb(
    uint32_t volume,
    void *arg)
{
    (void)arg;

    /*
     * The UAC component reports volume as a 0..100 value mapped
     * from the host's dB control range.
     *
     * Map that back onto a wider audio gain range so the slider
     * produces an audible change.
     */
    const float min_db = -60.0f;
    const float max_db = 0.0f;


    if (volume == 0) {

        volume_factor = 0.0f;

        ESP_LOGI(
            TAG,
            "USB volume: 0 -> mute"
        );

        return;
    }


    /*
     * Your original implementation assumes 0..100.
     */
    float norm =
        (float)volume / 100.0f;


    if (norm > 1.0f) {
        norm = 1.0f;
    }


    /*
     * Convert 0..100 to the configured dB span.
     */
    float db =
        min_db +
        norm * (max_db - min_db);


    /*
     * dB -> linear amplitude.
     */
    volume_factor =
        powf(
            10.0f,
            db / 20.0f
        );


    if (volume_factor > 1.0f) {
        volume_factor = 1.0f;
    }

    if (volume_factor < 0.0f) {
        volume_factor = 0.0f;
    }


    ESP_LOGI(
        TAG,
        "USB volume=%u -> %.2f dB -> gain=%.6f",
        (unsigned)volume,
        db,
        (double)volume_factor
    );
}


/*
 * ============================================================
 * USB UAC initialization
 * ============================================================
 */
static esp_err_t init_usb_uac(void)
{
    uac_device_config_t config = {
        .output_cb =
            uac_device_output_cb,

        .input_cb =
            uac_device_input_cb,

        .set_mute_cb =
            uac_device_set_mute_cb,

        .set_volume_cb =
            uac_device_set_volume_cb,

        .cb_ctx = NULL,
    };


    esp_err_t err =
        uac_device_init(&config);


    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "uac_device_init() failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "USB UAC initialized"
    );

    return ESP_OK;
}


/*
 * ============================================================
 * Main
 * ============================================================
 */
void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Starting USB UAC -> I2S audio"
    );


    /*
     * Start I2S first.
     *
     * This ensures the I2S clock is already running before
     * USB audio starts sending samples.
     */
    ESP_ERROR_CHECK(
        init_i2s()
    );


    /*
     * Start USB UAC.
     *
     * The usb_device_uac component handles the USB Audio
     * feedback mechanism.
     */
    ESP_ERROR_CHECK(
        init_usb_uac()
    );


    ESP_LOGI(
        TAG,
        "USB UAC -> I2S ready"
    );


    /*
     * app_main() can return.
     *
     * I2S and USB run from their respective driver tasks.
     */
}
