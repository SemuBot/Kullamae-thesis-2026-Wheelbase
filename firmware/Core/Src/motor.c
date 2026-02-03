/*
 * motor.c - Motor Control Layer
 *
 */

#include "motor.h"
#include "tim.h"
#include <math.h>

motor_st motor1 = {
    .htim = &htim1,
    .channel = TIM_CHANNEL_1,
    .dir_port = motor1_dir_GPIO_Port,
    .dir_pin = motor1_dir_Pin,
    .led_port = motor1_led_GPIO_Port,
    .led_pin = motor1_led_Pin,
    .current_duty = 0.0f,
    .target_duty = 0.0f
};

motor_st motor2 = {
    .htim = &htim3,
    .channel = TIM_CHANNEL_1,
    .dir_port = motor2_dir_GPIO_Port,
    .dir_pin = motor2_dir_Pin,
    .led_port = motor2_led_GPIO_Port,
    .led_pin = motor2_led_Pin,
    .current_duty = 0.0f,
    .target_duty = 0.0f
};

motor_st motor3 = {
    .htim = &htim4,
    .channel = TIM_CHANNEL_1,
    .dir_port = motor3_dir_GPIO_Port,
    .dir_pin = motor3_dir_Pin,
    .led_port = motor3_led_GPIO_Port,
    .led_pin = motor3_led_Pin,
    .current_duty = 0.0f,
    .target_duty = 0.0f
};


void Motor_SetDuty(motor_st *motor, float duty)
{
    // Clamp duty cycle
    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;

    motor->target_duty = duty;
}


void Motor_Update(motor_st *motor)
{
    const float RAMP_RATE = 5.0f;  // % per update

    // Ramp current duty towards target
    if (fabsf(motor->current_duty - motor->target_duty) < RAMP_RATE) {
        motor->current_duty = motor->target_duty;
    } else if (motor->current_duty < motor->target_duty) {
        motor->current_duty += RAMP_RATE;
    } else {
        motor->current_duty -= RAMP_RATE;
    }

    // Apply to hardware
    Motor_ApplyPWM(motor, motor->current_duty);
}

void Motor_ApplyPWM(motor_st *motor, float duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->htim);
    GPIO_PinState direction;
    float abs_duty;

    if (duty >= 0) {
        direction = GPIO_PIN_SET;    // Forward
        abs_duty = duty;
    } else {
        direction = GPIO_PIN_RESET;  // Reverse
        abs_duty = -duty;
    }

    // Clamp
    if (abs_duty > 100.0f) abs_duty = 100.0f;
    if (abs_duty < 0.0f) abs_duty = 0.0f;

    // Set direction
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, direction);

    // LED indicator
    if (abs_duty > 0.5f) {
        HAL_GPIO_WritePin(motor->led_port, motor->led_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(motor->led_port, motor->led_pin, GPIO_PIN_RESET);
    }

    // Calculate PWM compare value
    uint32_t compare = (uint32_t)((arr * abs_duty) / 100.0f);

    // Apply PWM
    if (abs_duty > 0.5f) {
        // Start PWM
        if (motor->htim->Instance == TIM1) {
            __HAL_TIM_MOE_ENABLE(motor->htim);
        }
        HAL_TIM_PWM_Start(motor->htim, motor->channel);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, compare);
    } else {
        // Stop PWM
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
        HAL_TIM_PWM_Stop(motor->htim, motor->channel);
    }
}

/**

void Motor_Stop(motor_st *motor)
{
    motor->target_duty = 0.0f;
    motor->current_duty = 0.0f;

    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    HAL_GPIO_WritePin(motor->led_port, motor->led_pin, GPIO_PIN_RESET);
}


void Motor_StopAll(void)
{
    Motor_Stop(&motor1);
    Motor_Stop(&motor2);
    Motor_Stop(&motor3);
}

void Motor_UpdateAll(void)
{
    Motor_Update(&motor1);
    Motor_Update(&motor2);
    Motor_Update(&motor3);
}
