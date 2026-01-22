/* motor_control.h
 * High-level motor control functions for DRV8353-based motors
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "drv8353.h"
#include "tim.h"

// Motor identifiers
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_COUNT = 3
} MotorID_t;

// Motor direction
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1
} MotorDirection_t;

// Motor state
typedef enum {
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_RUNNING = 1,
    MOTOR_STATE_BRAKING = 2,
    MOTOR_STATE_FAULT = 3
} MotorState_t;

// Motor configuration structure
typedef struct {
    // Hardware references
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    // DRV8353 handle
    DRV8353_Handle *drv;

    // Current state
    MotorState_t state;
    MotorDirection_t direction;
    uint8_t duty_cycle;  // 0-100%

    // Fault tracking
    uint16_t last_fault_status;
    uint16_t last_vgs_status;
    uint32_t fault_count;
} MotorControl_t;

// Global motor control handles
extern MotorControl_t motor_ctrl[MOTOR_COUNT];

// ============================================================================
// Initialization Functions
// ============================================================================

/**
 * @brief Initialize all motor control structures and hardware
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_InitAll(void);

/**
 * @brief Initialize a specific motor
 * @param motor_id: Motor identifier (MOTOR_1, MOTOR_2, MOTOR_3)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_Init(MotorID_t motor_id);

/**
 * @brief Configure DRV8353 for 1x PWM mode
 * @param motor_id: Motor identifier
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_ConfigureDRV8353(MotorID_t motor_id);

// ============================================================================
// Motor Control Functions
// ============================================================================

/**
 * @brief Start motor with specified duty cycle and direction
 * @param motor_id: Motor identifier
 * @param duty_cycle: PWM duty cycle (0-100%)
 * @param direction: Forward or reverse
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_Start(MotorID_t motor_id, uint8_t duty_cycle, MotorDirection_t direction);

/**
 * @brief Stop motor immediately (coast mode)
 * @param motor_id: Motor identifier
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_Stop(MotorID_t motor_id);

/**
 * @brief Stop motor with braking
 * @param motor_id: Motor identifier
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_Brake(MotorID_t motor_id);

/**
 * @brief Set motor duty cycle (while running)
 * @param motor_id: Motor identifier
 * @param duty_cycle: PWM duty cycle (0-100%)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_SetDuty(MotorID_t motor_id, uint8_t duty_cycle);

/**
 * @brief Ramp motor duty cycle smoothly
 * @param motor_id: Motor identifier
 * @param target_duty: Target duty cycle (0-100%)
 * @param ramp_time_ms: Time to reach target (ms)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_Ramp(MotorID_t motor_id, uint8_t target_duty, uint32_t ramp_time_ms);

/**
 * @brief Change motor direction (stops, changes dir, restarts)
 * @param motor_id: Motor identifier
 * @param new_direction: New direction
 * @param duty_cycle: Duty cycle after direction change
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_ChangeDirection(MotorID_t motor_id, MotorDirection_t new_direction, uint8_t duty_cycle);

// ============================================================================
// Status and Diagnostic Functions
// ============================================================================

/**
 * @brief Check for faults on specific motor
 * @param motor_id: Motor identifier
 * @retval HAL_StatusTypeDef (HAL_ERROR if fault present)
 */
HAL_StatusTypeDef MotorControl_CheckFaults(MotorID_t motor_id);

/**
 * @brief Clear faults on specific motor
 * @param motor_id: Motor identifier
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MotorControl_ClearFaults(MotorID_t motor_id);

/**
 * @brief Get current motor state
 * @param motor_id: Motor identifier
 * @retval MotorState_t
 */
MotorState_t MotorControl_GetState(MotorID_t motor_id);

/**
 * @brief Print motor status to USB CDC
 * @param motor_id: Motor identifier
 */
void MotorControl_PrintStatus(MotorID_t motor_id);

/**
 * @brief Print all motors status to USB CDC
 */
void MotorControl_PrintAllStatus(void);

// ============================================================================
// Emergency Functions
// ============================================================================

/**
 * @brief Emergency stop all motors
 */
void MotorControl_EmergencyStopAll(void);

#endif /* MOTOR_CONTROL_H */
