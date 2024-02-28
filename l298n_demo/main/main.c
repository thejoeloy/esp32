#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// Define GPIO pins for motor control
#define MOTOR1A_GPIO 12
#define MOTOR1B_GPIO 14
#define MOTOR2A_GPIO 26
#define MOTOR2B_GPIO 27

// Define PWM channels for speed control
#define MOTOR1_PWM_CHANNEL 0
#define MOTOR2_PWM_CHANNEL 1

void init_motor_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<MOTOR1A_GPIO) | (1ULL<<MOTOR1B_GPIO) | (1ULL<<MOTOR2A_GPIO) | (1ULL<<MOTOR2B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
}

void init_pwm() {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,  // PWM frequency
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = MOTOR1_PWM_CHANNEL,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = MOTOR1_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel1);

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = MOTOR2_PWM_CHANNEL,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = MOTOR2_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel2);
}

void set_motor_speed(int motor, uint32_t speed) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor, speed);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor);
}

void set_motor_direction(gpio_num_t motorA, gpio_num_t motorB, int direction) {
    gpio_set_level(motorA, (direction == 1) ? 1 : 0);
    gpio_set_level(motorB, (direction == -1) ? 1 : 0);
}

void app_main(void) {
    printf("Hello world!\n");

    // Initialize GPIO pins for motor control
    init_motor_gpio();

    // Initialize PWM for motor speed control
    init_pwm();

    // Example: Set Motor 1 speed to 50% in the forward direction
    set_motor_speed(MOTOR1_PWM_CHANNEL, 8191); // 50% duty cycle (13-bit resolution)
    set_motor_direction(MOTOR1A_GPIO, MOTOR1B_GPIO, 1); // 1 for forward, -1 for backward

    // Example: Set Motor 2 speed to 75% in the backward direction
    set_motor_speed(MOTOR2_PWM_CHANNEL, 12287); // 75% duty cycle (13-bit resolution)
    set_motor_direction(MOTOR2A_GPIO, MOTOR2B_GPIO, -1); // 1 for forward, -1 for backward

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds

    // Stop the motors
    set_motor_speed(MOTOR1_PWM_CHANNEL, 0);
    set_motor_speed(MOTOR2_PWM_CHANNEL, 0);

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

