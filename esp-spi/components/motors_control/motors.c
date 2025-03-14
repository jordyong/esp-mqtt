#include "motors.h"
#include "pwm.h"

void setDutyCycle(int M1, int M2) {
  ledc_set_duty(LEDC_MODE, MOTOR1_CHANNEL, M1);
  ledc_update_duty(LEDC_MODE, MOTOR1_CHANNEL);

  ledc_set_duty(LEDC_MODE, MOTOR2_CHANNEL, M2);
  ledc_update_duty(LEDC_MODE, MOTOR2_CHANNEL);
}

void setDirection(int direction) {
  switch (direction) {
  case MOVESTOP:
    setDutyCycle(0, 0);
    break;
  case MOVEFRONT:
    gpio_set_level(MOTOR1_DIR_PIN, 0);
    gpio_set_level(MOTOR2_DIR_PIN, 0);
    setDutyCycle(256, 256);
    break;
  case MOVELEFT:
    gpio_set_level(MOTOR1_DIR_PIN, 1);
    gpio_set_level(MOTOR2_DIR_PIN, 0);
    setDutyCycle(256, 256);
    break;
  case MOVERIGHT:
    gpio_set_level(MOTOR1_DIR_PIN, 0);
    gpio_set_level(MOTOR2_DIR_PIN, 1);
    setDutyCycle(256, 256);
    break;
  default:
    setDutyCycle(0, 0);
  }
}

void motorsInit() {
  MotorControlConfig motor_config = {
      .timer_config = {.speed_mode = LEDC_MODE,
                       .duty_resolution = LEDC_DUTY_RES,
                       .timer_num = LEDC_TIMER,
                       .freq_hz = LEDC_FREQUENCY,
                       .clk_cfg = LEDC_AUTO_CLK,
                       .deconfigure = 0},
      .motors = {{.gpio_num = MOTOR1_PWM_PIN,
                  .speed_mode = LEDC_MODE,
                  .channel = MOTOR1_CHANNEL,
                  .timer_sel = LEDC_TIMER,
                  .duty = 0,
                  .hpoint = 0},
                 {.gpio_num = MOTOR2_PWM_PIN,
                  .speed_mode = LEDC_MODE,
                  .channel = MOTOR2_CHANNEL,
                  .timer_sel = LEDC_TIMER,
                  .duty = 0,
                  .hpoint = 0}}};

  ledc_pwm_motor_control_init(&motor_config);

  gpio_set_direction(MOTOR1_DIR_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR2_DIR_PIN, GPIO_MODE_OUTPUT);

  setDirection(MOVESTOP);
}
