#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "esp_log.h"
#include <driver/ledc.h>

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10-bit PWM resolution (1024 steps)
#define LEDC_FREQUENCY 1000             // Frequency in Hz (1 kHz)

// Define the 4 PWM GPIO pins
#define MOTOR1_PWM_PIN CONFIG_ESP32_MOTOR1_PWM
#define MOTOR2_PWM_PIN CONFIG_ESP32_MOTOR2_PWM

#define MOTOR1_DIR_PIN CONFIG_ESP32_MOTOR1_DIR
#define MOTOR2_DIR_PIN CONFIG_ESP32_MOTOR2_DIR

// LEDC channels for each motor
#define MOTOR1_CHANNEL LEDC_CHANNEL_0
#define MOTOR2_CHANNEL LEDC_CHANNEL_1

// Define a struct for each motor's PWM configuration
typedef struct {
  int gpio_num;
  ledc_mode_t speed_mode;
  ledc_channel_t channel;
  ledc_timer_t timer_sel;
  uint32_t duty;
  int hpoint;
} MotorPWMConfig;

// Define a struct to hold all motors' configurations and the LEDC timer
// configuration
typedef struct {
  ledc_timer_config_t timer_config;
  MotorPWMConfig motors[2];
} MotorControlConfig;

#ifdef __cplusplus
extern "C" {
#endif

void ledc_pwm_motor_control_init(MotorControlConfig *config);

#ifdef __cplusplus
}
#endif
#endif
