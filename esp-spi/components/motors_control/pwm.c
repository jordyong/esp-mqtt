#include "pwm.h"

void ledc_pwm_motor_control_init(MotorControlConfig *config){
    // Configure the LEDC timer using the timer configuration from the struct
    ledc_timer_config(&config->timer_config);

    // Configure each motor channel using the configurations from the struct
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num       = config->motors[i].gpio_num,
            .speed_mode     = config->motors[i].speed_mode,
            .channel        = config->motors[i].channel,
            .timer_sel      = config->motors[i].timer_sel,
            .duty           = config->motors[i].duty,
            .hpoint         = config->motors[i].hpoint
        };
        ledc_channel_config(&ledc_channel);
    }
}