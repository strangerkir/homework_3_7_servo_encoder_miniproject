//Ussing PCNT


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define ENC_A   GPIO_NUM_9
#define ENC_B   GPIO_NUM_10
#define ENC_SW  GPIO_NUM_11


#define PCNT_HIGH_LIMIT   1000
#define PCNT_LOW_LIMIT   -1000


#define PULSES_PER_DETENT  4
#define DETENTS_PER_REV    20
#define STEPS_PER_REV      (PULSES_PER_DETENT * DETENTS_PER_REV)  

#define LOOP_PERIOD_MS     100

#define SERVO_GPIO 16

#define SERVO_MAX_ANGLE 175
#define SERVO_MIN_ANGLE 5

#define SERVO_MAX_DUTY (1u << 14)
#define SERVO_PERIOD_US 20000u

#define SERVO_MIN_US 400
#define SERVO_MAX_US 2600

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_14_BIT

#define POT_CHANNEL ADC_CHANNEL_4


static const char *TAG = "ENC";

static pcnt_unit_handle_t pcnt_unit = NULL;


static void encoder_init(void)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit  = PCNT_HIGH_LIMIT,
        .low_limit   = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &pcnt_unit));


    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_cfg));

  
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = ENC_A,
        .level_gpio_num = ENC_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_cfg, &chan_a));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,    
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));  
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,       /* B = HIGH    */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));  /* B = LOW   */

    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = ENC_B,
        .level_gpio_num = ENC_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_cfg, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));


    gpio_config_t sw_cfg = {
        .pin_bit_mask = (1ULL << ENC_SW),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&sw_cfg));

    ESP_LOGI(TAG, "PCNT X4 готовий. A=%d B=%d SW=%d, %d кроків на оберт",
             ENC_A, ENC_B, ENC_SW, STEPS_PER_REV);
}

static void servo_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t c = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

void servo_set_us(uint32_t us) {
    if(us < SERVO_MIN_US) {
        us = SERVO_MIN_US;
    }

    if(us > SERVO_MAX_US) {
        us = SERVO_MAX_US;
    }

    uint32_t duty = (uint32_t)((uint32_t)(us * SERVO_MAX_DUTY) / SERVO_PERIOD_US);

    ESP_LOGI("Main App: ", "Setting Servo Duty %d", duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void servo_set_angle(float angle) {
    if(angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }

    if(angle < SERVO_MIN_ANGLE) {
        angle = SERVO_MIN_ANGLE;
    }

    uint32_t us = SERVO_MIN_US + (uint32_t)((angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
    ESP_LOGI("check", " us = %d", us);

    servo_set_us(us);

}

void app_main(void)
{
    encoder_init();
    servo_init();

    int     last_count = 0;
    int64_t last_us    = esp_timer_get_time();
    int     sw_prev    = 1;

    while (1) {
        int count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &count));

        int64_t now_us = esp_timer_get_time();
        int     delta  = count - last_count;
        int64_t dt_us  = now_us - last_us;


        float rpm = 0.0f;
        if (dt_us > 0) {
            rpm = ((float)delta / STEPS_PER_REV) * (60.0f * 1000000.0f / (float)dt_us);
        }


        float angle = (count % STEPS_PER_REV) * (360.0f / STEPS_PER_REV);
        if (angle < 0.0f) {
            angle += 360.0f;
        }

        const char *dir = (delta > 0) ? "CW " : (delta < 0) ? "CCW" : "-  ";


        if (delta != 0) {
            servo_set_angle(angle);

            ESP_LOGI(TAG, "%s кроки=%6d  клац=%5d  кут=%6.1f  RPM=%7.1f",
                     dir,
                     count,
                     count / PULSES_PER_DETENT,
                     angle,
                     rpm);
        }

        last_count = count;
        last_us    = now_us;

        int sw = gpio_get_level(ENC_SW);
        if (sw == 0 && sw_prev == 1) {
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
            last_count = 0;
            ESP_LOGW(TAG, "нуль встановлено");
        }
        sw_prev = sw;


        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}