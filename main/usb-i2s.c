#include <stdio.h>
#include "usb_device_uac.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

i2s_chan_handle_t tx_handle;

i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
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

static float volume_factor = 1.0; // default: full volume

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    int16_t *samples = (int16_t *)buf; // 16-bit audio
    size_t sample_count = len / 2;     // 2 bytes per sample

    for(size_t i = 0; i < sample_count; i++) {
        int32_t temp = (int32_t)samples[i] * volume_factor; // scale
        if(temp > 32767) temp = 32767;
        if(temp < -32768) temp = -32768;
        samples[i] = (int16_t)temp;
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
}

static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    // Assuming volume is 0..255
    volume_factor = (float)volume / 255.0f;  
    if(volume_factor > 1.0f) volume_factor = 1.0f;
    if(volume_factor < 0.0f) volume_factor = 0.0f;
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