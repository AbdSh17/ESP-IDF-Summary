

/*
This code will read a value from POT using ADC then light a LED based on this value usign PWM
while if the pot is half it's value wil light a right LED else a left LED

Topics Covered:
GPIOs
ADC
PWM
*/

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "driver/ledc.h"

#define PWM_GPIO 5
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ 5000                // 5 kHz
#define PWM_RES LEDC_TIMER_12_BIT    // 12-bit resolution
#define PWM_MAX_DUTY ((1 << 12) - 1) // 0..8191

#define POT_PIN = ADC_CHANNEL_3

#define LED1 6
#define LED2 7

// float pid_update(PID_t *, float, float, float);

static const char *TAG = "ADC";

void app_main(void)
{
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1));

    adc_oneshot_chan_cfg_t cfg1 = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_CHANNEL_3, &cfg1));

    // Configure LEDC PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_channel_config_t ledc_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    gpio_config_t io_out_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_6) | (1ULL << GPIO_NUM_7),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_out_conf);

    int value = 0;

    while (1)
    {
        adc_oneshot_read(adc1, ADC_CHANNEL_3, &value);
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, value);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);

        if (value < PWM_MAX_DUTY / 2)
        {
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 0);
        }

        else
        {
            gpio_set_level(LED1, 0);
            gpio_set_level(LED2, 1);
        }

        ESP_LOGI(TAG, "ADC Raw Value: %d", value);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}