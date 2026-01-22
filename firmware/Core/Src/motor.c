/*
 * motor.c
 *
 *  Created on: Nov 18, 2024
 *      Author: medved
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "motor.h"
#include "drv8353.h"
#include "tim.h"
#include "spi.h"
#include "stm32f3xx_hal.h"
#include "usbd_cdc_if.h"

static uint8_t duty_cycle_limit;

// DRV8353 handles for each motor
DRV8353_Handle drv_motor1;
DRV8353_Handle drv_motor2;
DRV8353_Handle drv_motor3;

motor_st motor1 = {
    .dir_port = motor1_dir_GPIO_Port,
    .dir_pin = motor1_dir_Pin,
    .pwm_timer = &htim1,
    .pwm_port = motor1_pwm_GPIO_Port,
    .pwm_pin = motor1_pwm_Pin,
    .led_port = motor1_led_GPIO_Port,
    .led_pin = motor1_led_Pin,
    .duty_cycle = 0.0,
    .duty_cycle_limit = DUTY_CYCLE_LIMIT,
    .drv_handle = &drv_motor1
};

motor_st motor2 = {
    .dir_port = motor2_dir_GPIO_Port,
    .dir_pin = motor2_dir_Pin,
    .pwm_timer = &htim3,
    .pwm_port = motor2_pwm_GPIO_Port,
    .pwm_pin = motor2_pwm_Pin,
    .led_port = motor2_led_GPIO_Port,
    .led_pin = motor2_led_Pin,
    .duty_cycle = 0.0,
    .duty_cycle_limit = DUTY_CYCLE_LIMIT,
    .drv_handle = &drv_motor2
};

motor_st motor3 = {
    .dir_port = motor3_dir_GPIO_Port,
    .dir_pin = motor3_dir_Pin,
    .pwm_timer = &htim4,
    .pwm_port = motor3_pwm_GPIO_Port,
    .pwm_pin = motor3_pwm_Pin,
    .led_port = motor3_led_GPIO_Port,
    .led_pin = motor3_led_Pin,
    .duty_cycle = 0.0,
    .duty_cycle_limit = DUTY_CYCLE_LIMIT,
    .drv_handle = &drv_motor3
};

/**
 * @brief Initialize motor and its DRV8353 driver
 * @param motor_data: Pointer to motor structure
 * @retval None
 */
void motor_init(motor_st *motor_data) {
    HAL_StatusTypeDef status;
    char msg[128];
    int len;

    duty_cycle_limit = DUTY_CYCLE_LIMIT_DEFAULT;

    // Initialize GPIO for direction
    HAL_GPIO_WritePin(motor_data->dir_port, motor_data->dir_pin, GPIO_PIN_SET);

    // Initialize DRV8353 driver if handle exists
    if (motor_data->drv_handle != NULL) {
        // Initialize the driver
        status = DRV8353_Init(motor_data->drv_handle);

        if (status == HAL_OK) {
            len = snprintf(msg, sizeof(msg), "DRV8353 initialized successfully\r\n");
            CDC_Transmit_FS((uint8_t *)msg, len);


            if (status == HAL_OK) {
                len = snprintf(msg, sizeof(msg), "DRV8353 configured successfully\r\n");
                CDC_Transmit_FS((uint8_t *)msg, len);

                // Enable the driver
                status = DRV8353_Enable(motor_data->drv_handle);

                if (status == HAL_OK) {
                    len = snprintf(msg, sizeof(msg), "DRV8353 enabled\r\n");
                } else {
                    len = snprintf(msg, sizeof(msg), "DRV8353 enable failed\r\n");
                }
            } else {
                len = snprintf(msg, sizeof(msg), "DRV8353 configuration failed\r\n");
            }
        } else {
            len = snprintf(msg, sizeof(msg), "DRV8353 initialization failed\r\n");
        }

        CDC_Transmit_FS((uint8_t *)msg, len);

        // Read and display any faults
        DRV8353_ReadFaults(motor_data->drv_handle);
        DRV8353_PrintFaults(motor_data->drv_handle);
    }
}

/**
 * @brief Update motor PWM and direction
 * @param motor_data: Pointer to motor structure
 * @retval None
 */
void motor_update(motor_st *motor_data) {
    uint16_t scaled_duty_cycle;
    uint16_t arr_value = __HAL_TIM_GET_AUTORELOAD(motor_data->pwm_timer);

    // Handle direction based on duty cycle sign
    if (motor_data->duty_cycle > 0) {
        // Forward direction
        HAL_GPIO_WritePin(motor_data->dir_port, motor_data->dir_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_data->led_port, motor_data->led_pin, GPIO_PIN_SET);
    } else if (motor_data->duty_cycle < 0) {
        // Reverse direction
        HAL_GPIO_WritePin(motor_data->dir_port, motor_data->dir_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_data->led_port, motor_data->led_pin, GPIO_PIN_RESET);
    } else {
        // Zero speed - LED off, keep last direction
        HAL_GPIO_WritePin(motor_data->led_port, motor_data->led_pin, GPIO_PIN_RESET);
    }

    // Scale the duty cycle based on absolute value
    scaled_duty_cycle = (uint16_t)((fabs(motor_data->duty_cycle) / 100.0f) * arr_value);
    if (scaled_duty_cycle > arr_value) {
        scaled_duty_cycle = arr_value;
    }

    // Update PWM duty cycle
    __HAL_TIM_SET_COMPARE(motor_data->pwm_timer, TIM_CHANNEL_1, scaled_duty_cycle);
}

/**
 * @brief Calculate motor duty cycles for holonomic drive
 * @param linear_x: Linear velocity in x direction
 * @param linear_y: Linear velocity in y direction
 * @param omega: Angular velocity
 * @param motor1: Pointer to motor 1 structure
 * @param motor2: Pointer to motor 2 structure
 * @param motor3: Pointer to motor 3 structure
 * @retval None
 */
void calculate_motor_duty_cycles(float linear_x, float linear_y, float omega,
                                  motor_st *motor1, motor_st *motor2, motor_st *motor3) {
    float duty_cycle_1 = (linear_x * cosf(motor1_angle) + linear_y * sinf(motor1_angle) +
                          (omega * ROBOT_RADIUS)) * PWM_SCALING_FACTOR;
    float duty_cycle_2 = (linear_x * cosf(motor2_angle) + linear_y * sinf(motor2_angle) +
                          (omega * ROBOT_RADIUS)) * PWM_SCALING_FACTOR;
    float duty_cycle_3 = (linear_x * cosf(motor3_angle) + linear_y * sinf(motor3_angle) +
                          (omega * ROBOT_RADIUS)) * PWM_SCALING_FACTOR;

    duty_cycle_1 = fmaxf(-100.0f, fminf(100.0f, duty_cycle_1));
    duty_cycle_2 = fmaxf(-100.0f, fminf(100.0f, duty_cycle_2));
    duty_cycle_3 = fmaxf(-100.0f, fminf(100.0f, duty_cycle_3));

    motor1->duty_cycle = duty_cycle_1;
    motor2->duty_cycle = duty_cycle_2;
    motor3->duty_cycle = duty_cycle_3;

    motor_update(motor1);
    motor_update(motor2);
    motor_update(motor3);

    char motor_calc_debug_msg[128];
    snprintf(motor_calc_debug_msg, sizeof(motor_calc_debug_msg),
             "M1_DC: %.2f, M2_DC: %.2f, M3_DC: %.2f\r\n",
             motor1->duty_cycle, motor2->duty_cycle, motor3->duty_cycle);
    CDC_Transmit_FS((uint8_t *)motor_calc_debug_msg, strlen(motor_calc_debug_msg));
}


void motor_enable(motor_st *motor_data) {
    HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(motor_data->pwm_timer, TIM_CHANNEL_1);

    // Enable DRV8353 if handle exists
    if (motor_data->drv_handle != NULL) {
        DRV8353_Enable(motor_data->drv_handle);
    }
}


void motor_disable(motor_st *motor_data) {
    HAL_TIM_PWM_Stop(motor_data->pwm_timer, TIM_CHANNEL_1);

    // Disable DRV8353 if handle exists
    if (motor_data->drv_handle != NULL) {
        DRV8353_Disable(motor_data->drv_handle);
    }
}


void motor_check_faults(void) {
    char msg[64];
    int len;

    // Check motor 1
    if (drv_motor1.initialized) {
        DRV8353_ReadFaults(&drv_motor1);
        if (DRV8353_HasFault(&drv_motor1)) {
            len = snprintf(msg, sizeof(msg), "Motor 1 Fault Detected:\r\n");
            CDC_Transmit_FS((uint8_t *)msg, len);
            DRV8353_PrintFaults(&drv_motor1);
        }
    }

    // Check motor 2
    if (drv_motor2.initialized) {
        DRV8353_ReadFaults(&drv_motor2);
        if (DRV8353_HasFault(&drv_motor2)) {
            len = snprintf(msg, sizeof(msg), "Motor 2 Fault Detected:\r\n");
            CDC_Transmit_FS((uint8_t *)msg, len);
            DRV8353_PrintFaults(&drv_motor2);
        }
    }

    // Check motor 3
    if (drv_motor3.initialized) {
        DRV8353_ReadFaults(&drv_motor3);
        if (DRV8353_HasFault(&drv_motor3)) {
            len = snprintf(msg, sizeof(msg), "Motor 3 Fault Detected:\r\n");
            CDC_Transmit_FS((uint8_t *)msg, len);
            DRV8353_PrintFaults(&drv_motor3);
        }
    }
}
