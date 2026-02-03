/*
 * motor.h - Motor Control Layer Header
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"

/* Motor Structure */
typedef struct {
    // Hardware
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *led_port;
    uint16_t led_pin;

    // State
    float current_duty;  // Current duty cycle (-100 to +100)
    float target_duty;   // Target duty cycle for ramping
} motor_st;

/* External Motor Instances */
extern motor_st motor1, motor2, motor3;

/* Function Prototypes */

/**
 * @brief Set motor target duty cycle
 * @param motor: Pointer to motor structure
 * @param duty: Target duty cycle (-100 to +100)
 */
void Motor_SetDuty(motor_st *motor, float duty);

/**
 * @brief Update motor PWM (call in main loop)
 * @param motor: Pointer to motor structure
 */
void Motor_Update(motor_st *motor);

/**
 * @brief Apply PWM directly to hardware
 * @param motor: Pointer to motor structure
 * @param duty: Duty cycle (-100 to +100)
 */
void Motor_ApplyPWM(motor_st *motor, float duty);

/**
 * @brief Stop motor immediately
 * @param motor: Pointer to motor structure
 */
void Motor_Stop(motor_st *motor);

/**
 * @brief Stop all motors
 */
void Motor_StopAll(void);

/**
 * @brief Update all motors (call in main loop at ~100Hz)
 */
void Motor_UpdateAll(void);

#endif /* MOTOR_H */
