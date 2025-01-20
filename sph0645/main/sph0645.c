/* I2S Digital Microphone Recording Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

static const char* TAG = "pdm_rec_example";
#define CONFIG_EXAMPLE_SAMPLE_RATE 16000      // Example sample rate in Hz
#define CONFIG_EXAMPLE_BIT_SAMPLE 16          // 16-bit per sample
#define CONFIG_EXAMPLE_I2S_CLK_GPIO 26        // I2S Clock GPIO pin
#define CONFIG_EXAMPLE_I2S_DATA_GPIO 25       // I2S Data GPIO pin
#define CONFIG_EXAMPLE_I2S_CH 0               // I2S channel 0
#define CONFIG_EXAMPLE_REC_TIME 10
#define NUM_CHANNELS        (1)               // For mono recording only!
#define SAMPLE_SIZE         (CONFIG_EXAMPLE_BIT_SAMPLE * 1024)
#define BYTE_RATE           (CONFIG_EXAMPLE_SAMPLE_RATE * (CONFIG_EXAMPLE_BIT_SAMPLE / 8)) * NUM_CHANNELS

static int16_t i2s_readraw_buff[SAMPLE_SIZE];

void init_microphone(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM,
        .sample_rate = CONFIG_EXAMPLE_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 8,
        .dma_buf_len = 200,
        .use_apll = 0,
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = CONFIG_EXAMPLE_I2S_CLK_GPIO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = CONFIG_EXAMPLE_I2S_DATA_GPIO,
    };

    ESP_ERROR_CHECK(i2s_driver_install(CONFIG_EXAMPLE_I2S_CH, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(CONFIG_EXAMPLE_I2S_CH, &pin_config));
    ESP_ERROR_CHECK(i2s_set_clk(CONFIG_EXAMPLE_I2S_CH, CONFIG_EXAMPLE_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO));
}

void read_microphone(void)
{
    size_t bytes_read = 0;
    esp_err_t ret = i2s_read(CONFIG_EXAMPLE_I2S_CH, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 100);
    if (ret == ESP_OK && bytes_read > 0) {
        ESP_LOGI(TAG, "Data read: %d bytes", bytes_read);

        // Print samples
        int num_samples = bytes_read / sizeof(int16_t); // Calculate the number of samples
        ESP_LOGI(TAG, "Number of samples: %d", num_samples);

        for (int i = 0; i < num_samples; i++) {
            printf("Sample[%d]: %d\n", i, i2s_readraw_buff[i]);
        }
    } else {
        ESP_LOGE(TAG, "Failed to read data!");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "PDM microphone recording Example start");
    init_microphone();
    ESP_LOGI(TAG, "Reading from mic!");
    read_microphone();
    ESP_ERROR_CHECK(i2s_driver_uninstall(CONFIG_EXAMPLE_I2S_CH));
}

