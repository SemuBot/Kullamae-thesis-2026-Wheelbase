/*
 * robot_control.c - Robot Application Layer
 *
 * Purpose: High-level robot control, initialization, and coordination
 * Coordinates DRV8353 drivers and motors for holonomic robot
 */

#include "robot_control.h"
#include "drv8353.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


DRV8353_Handle drv_motor1;
DRV8353_Handle drv_motor2;
DRV8353_Handle drv_motor3;


#define WHEEL_RADIUS       0.05f    // meters
#define ROBOT_RADIUS       0.15f    // meters (center to wheel)
#define MAX_WHEEL_SPEED    1.0f     // m/s

// Motor angles (3-wheel omni configuration)
#define MOTOR1_ANGLE       0.0f                 // Front (0°)
#define MOTOR2_ANGLE       (2.0f * M_PI / 3.0f) // Back-left (120°)
#define MOTOR3_ANGLE       (4.0f * M_PI / 3.0f) // Back-right (240°)


HAL_StatusTypeDef Robot_Init(void)
{
    char msg[256];
    HAL_StatusTypeDef status;

    snprintf(msg, sizeof(msg), "\r\n=== Initializing Robot ===\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    // Initialize DRV8353 handles
    Robot_InitDriverHandles();

    // Power up sequence
    status = Robot_PowerUp();
    if (status != HAL_OK) {
        snprintf(msg, sizeof(msg), "❌ Power-up failed!\r\n");
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        return status;
    }

    // Configure all drivers
    status = Robot_ConfigureDrivers();
    if (status != HAL_OK) {
        snprintf(msg, sizeof(msg), "❌ Driver configuration failed!\r\n");
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        return status;
    }

    // Release brake and calibrate
    HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    // Calibrate current sense amplifiers
    DRV8353_CalibrateCSA(&drv_motor1);
    DRV8353_CalibrateCSA(&drv_motor2);
    DRV8353_CalibrateCSA(&drv_motor3);

    // Set initial directions
    extern motor_st motor1, motor2, motor3;
    HAL_GPIO_WritePin(motor1.dir_port, motor1.dir_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2.dir_port, motor2.dir_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3.dir_port, motor3.dir_pin, GPIO_PIN_SET);

    snprintf(msg, sizeof(msg), "✓ Robot initialized successfully!\r\n\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    return HAL_OK;
}

/**
 * @brief Initialize DRV8353 SPI handles
 */
void Robot_InitDriverHandles(void)
{
    extern SPI_HandleTypeDef hspi2;

    // Motor 1
    drv_motor1.hspi = &hspi2;
    drv_motor1.cs_port = motor1_Cs_GPIO_Port;
    drv_motor1.cs_pin = motor1_Cs_Pin;
    drv_motor1.initialized = false;

    // Motor 2
    drv_motor2.hspi = &hspi2;
    drv_motor2.cs_port = motor2_Cs_GPIO_Port;
    drv_motor2.cs_pin = motor2_Cs_Pin;
    drv_motor2.initialized = false;

    // Motor 3
    drv_motor3.hspi = &hspi2;
    drv_motor3.cs_port = motor3_Cs_GPIO_Port;
    drv_motor3.cs_pin = motor3_Cs_Pin;
    drv_motor3.initialized = false;

    // Deselect all CS pins
    HAL_GPIO_WritePin(motor1_Cs_GPIO_Port, motor1_Cs_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2_Cs_GPIO_Port, motor2_Cs_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3_Cs_GPIO_Port, motor3_Cs_Pin, GPIO_PIN_SET);
}

/**
 * @brief Power up DRV8353 drivers
 * @retval HAL status
 */
HAL_StatusTypeDef Robot_PowerUp(void)
{
    char msg[128];

    // Initial state: disabled and braked
    HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    // Enable drivers
    snprintf(msg, sizeof(msg), "Enabling DRV8353 drivers...\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_SET);
    HAL_Delay(200);  // Wait for power-up

    return HAL_OK;
}


HAL_StatusTypeDef Robot_ConfigureDrivers(void)
{
    HAL_StatusTypeDef status;
    char msg[128];

    // Clear faults first
    DRV8353_ClearFaults(&drv_motor1);
    DRV8353_ClearFaults(&drv_motor2);
    DRV8353_ClearFaults(&drv_motor3);
    HAL_Delay(50);

    // Configure Motor 1
    snprintf(msg, sizeof(msg), "Configuring Motor 1...\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    status = DRV8353_Configure(&drv_motor1);
    if (status != HAL_OK) return status;

    // Configure Motor 2
    snprintf(msg, sizeof(msg), "Configuring Motor 2...\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    status = DRV8353_Configure(&drv_motor2);
    if (status != HAL_OK) return status;

    // Configure Motor 3
    snprintf(msg, sizeof(msg), "Configuring Motor 3...\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    status = DRV8353_Configure(&drv_motor3);
    if (status != HAL_OK) return status;

    snprintf(msg, sizeof(msg), "All drivers configured\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    return HAL_OK;
}

/* ============================================================================
 * INVERSE KINEMATICS
 * ============================================================================ */


void Robot_CalculateMotorSpeeds(float linear_x, float linear_y, float angular_z,
                                motor_st *m1, motor_st *m2, motor_st *m3)
{

    float v1 = -sinf(MOTOR1_ANGLE) * linear_x + cosf(MOTOR1_ANGLE) * linear_y + ROBOT_RADIUS * angular_z;
    float v2 = -sinf(MOTOR2_ANGLE) * linear_x + cosf(MOTOR2_ANGLE) * linear_y + ROBOT_RADIUS * angular_z;
    float v3 = -sinf(MOTOR3_ANGLE) * linear_x + cosf(MOTOR3_ANGLE) * linear_y + ROBOT_RADIUS * angular_z;

    // Convert to wheel angular velocities
    float omega1 = v1 / WHEEL_RADIUS;
    float omega2 = v2 / WHEEL_RADIUS;
    float omega3 = v3 / WHEEL_RADIUS;

    // Find maximum for normalization
    float max_omega = fmaxf(fmaxf(fabsf(omega1), fabsf(omega2)), fabsf(omega3));
    float max_allowed = MAX_WHEEL_SPEED / WHEEL_RADIUS;

    // Normalize if needed
    if (max_omega > max_allowed) {
        float scale = max_allowed / max_omega;
        omega1 *= scale;
        omega2 *= scale;
        omega3 *= scale;
    }

    // Convert to duty cycle (-100 to +100)
    float duty1 = (omega1 / max_allowed) * 100.0f;
    float duty2 = (omega2 / max_allowed) * 100.0f;
    float duty3 = (omega3 / max_allowed) * 100.0f;

    // Apply to motors
    Motor_SetDuty(m1, duty1);
    Motor_SetDuty(m2, duty2);
    Motor_SetDuty(m3, duty3);
}

/* ============================================================================
 * FAULT MONITORING
 * ============================================================================ */

void Robot_MonitorFaults(void)
{
    char msg[128];
    uint16_t fault1;

    // Check Motor 1
    DRV8353_ReadFaults(&drv_motor1);
    if (DRV8353_HasFault(&drv_motor1)) {
        DRV8353_ReadRegister(&drv_motor1, DRV8353_REG_FAULT_STATUS_1, &fault1);
        snprintf(msg, sizeof(msg), "M1 Fault: 0x%04X\r\n", fault1);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        Robot_EmergencyStop();
    }

    // Check Motor 2
    DRV8353_ReadFaults(&drv_motor2);
    if (DRV8353_HasFault(&drv_motor2)) {
        DRV8353_ReadRegister(&drv_motor2, DRV8353_REG_FAULT_STATUS_1, &fault1);
        snprintf(msg, sizeof(msg), "M2 Fault: 0x%04X\r\n", fault1);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        Robot_EmergencyStop();
    }

    // Check Motor 3
    DRV8353_ReadFaults(&drv_motor3);
    if (DRV8353_HasFault(&drv_motor3)) {
        DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_FAULT_STATUS_1, &fault1);
        snprintf(msg, sizeof(msg), "M3 Fault: 0x%04X\r\n", fault1);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        Robot_EmergencyStop();
    }
}


void Robot_EmergencyStop(void)
{
    Motor_StopAll();
    HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_RESET);

    char msg[] = "EMERGENCY STOP\r\n";
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
}


void Robot_ProcessCmdVel(float linear_x, float linear_y, float angular_z)
{
    extern motor_st motor1, motor2, motor3;

    // Apply inverse kinematics
    Robot_CalculateMotorSpeeds(linear_x, linear_y, angular_z,
                              &motor1, &motor2, &motor3);
}
