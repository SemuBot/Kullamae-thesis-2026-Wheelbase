/*
 * robot_control.h - Robot Application Layer Header
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "motor.h"

/* Function Prototypes */

/**
 * @brief Initialize complete robot system
 * @retval HAL status
 */
HAL_StatusTypeDef Robot_Init(void);

/**
 * @brief Initialize DRV8353 driver handles
 */
void Robot_InitDriverHandles(void);

/**
 * @brief Power up DRV8353 drivers
 * @retval HAL status
 */
HAL_StatusTypeDef Robot_PowerUp(void);

/**
 * @brief Configure all DRV8353 drivers
 * @retval HAL status
 */
HAL_StatusTypeDef Robot_ConfigureDrivers(void);

/**
 * @brief Calculate motor speeds from cmd_vel (inverse kinematics)
 * @param linear_x: Forward/backward velocity (m/s)
 * @param linear_y: Left/right velocity (m/s)
 * @param angular_z: Rotation velocity (rad/s)
 * @param m1, m2, m3: Motor structure pointers
 */
void Robot_CalculateMotorSpeeds(float linear_x, float linear_y, float angular_z,
                                motor_st *m1, motor_st *m2, motor_st *m3);

/**
 * @brief Process velocity command
 * @param linear_x, linear_y, angular_z: Velocity commands
 */
void Robot_ProcessCmdVel(float linear_x, float linear_y, float angular_z);

/**
 * @brief Monitor all motor faults
 */
void Robot_MonitorFaults(void);

/**
 * @brief Emergency stop all motors
 */
void Robot_EmergencyStop(void);

#endif /* ROBOT_CONTROL_H */
