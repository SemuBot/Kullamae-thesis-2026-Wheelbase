/*
 * motor.h
 *
 *  Created on: Nov 18, 2024
 *      Author: medved
 *  Updated: Dec 09, 2024 - Added DRV8353 SPI support
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f3xx_hal.h"
#include "drv8353.h"

// Motor angle definitions (in radians) for 120-degree spacing
#define motor1_angle 0.0f                    // 0 degrees
#define motor2_angle 2.0944f                 // 120 degrees (2π/3)
#define motor3_angle 4.1888f                 // 240 degrees (4π/3)

// Robot parameters
#define ROBOT_RADIUS 0.15f                   // Robot radius in meters
#define PWM_SCALING_FACTOR 100.0f            // Scaling factor for PWM

// Duty cycle limits
#define DUTY_CYCLE_LIMIT 100
#define DUTY_CYCLE_LIMIT_DEFAULT 80

// Motor structure definition
typedef struct {
    GPIO_TypeDef *dir_port;                  // Direction GPIO port
    uint16_t dir_pin;                        // Direction GPIO pin
    TIM_HandleTypeDef *pwm_timer;            // PWM timer handle
    GPIO_TypeDef *pwm_port;                  // PWM GPIO port
    uint16_t pwm_pin;                        // PWM GPIO pin
    GPIO_TypeDef *led_port;                  // LED GPIO port
    uint16_t led_pin;                        // LED GPIO pin
    float duty_cycle;                        // Current duty cycle (-100 to +100)
    uint8_t duty_cycle_limit;                // Maximum duty cycle limit
    DRV8353_Handle *drv_handle;              // DRV8353 driver handle
} motor_st;

// Global motor instances
extern motor_st motor1;
extern motor_st motor2;
extern motor_st motor3;

// Function prototypes
void motor_init(motor_st *motor_data);
void motor_update(motor_st *motor_data);
void calculate_motor_duty_cycles(float linear_x, float linear_y, float omega,
                                  motor_st *motor1, motor_st *motor2, motor_st *motor3);
void motor_enable(motor_st *motor_data);
void motor_disable(motor_st *motor_data);
void motor_check_faults(void);

#endif /* INC_MOTOR_H_ */
