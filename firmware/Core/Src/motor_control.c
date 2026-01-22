/* motor_control.c
 * Implementation of motor control functions
 */

#include "motor_control.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

// External hardware handles
extern TIM_HandleTypeDef htim1, htim3, htim4;
extern DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;

// Global motor control structures
MotorControl_t motor_ctrl[MOTOR_COUNT];

// Private functions
static void _MotorControl_ApplyBrake(MotorID_t motor_id, bool enable);
static void _MotorControl_SetPWM(MotorID_t motor_id, uint8_t duty_cycle);

HAL_StatusTypeDef MotorControl_InitAll(void) {
    HAL_StatusTypeDef status;
    char msg[128];

    snprintf(msg, sizeof(msg), "\r\n=== Initializing All Motors ===\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
    HAL_Delay(10);

    // Initialize each motor
    for (MotorID_t i = MOTOR_1; i < MOTOR_COUNT; i++) {
        status = MotorControl_Init(i);
        if (status != HAL_OK) {
            snprintf(msg, sizeof(msg), " Motor %d initialization failed!\r\n", i + 1);
            CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
            return status;
        }
        HAL_Delay(100);
    }

    snprintf(msg, sizeof(msg), "✓ All motors initialized successfully\r\n\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_Init(MotorID_t motor_id) {
    HAL_StatusTypeDef status;
    char msg[128];
    MotorControl_t *motor = &motor_ctrl[motor_id];

    snprintf(msg, sizeof(msg), "\r\n--- Initializing Motor %d ---\r\n", motor_id + 1);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
    HAL_Delay(10);

    switch(motor_id) {
        case MOTOR_1:
            motor->htim = &htim1;
            motor->tim_channel = TIM_CHANNEL_1;
            motor->dir_port = motor1_dir_GPIO_Port;
            motor->dir_pin = motor1_dir_Pin;
            motor->drv = &drv_motor1;
            break;

        case MOTOR_2:
            motor->htim = &htim3;
            motor->tim_channel = TIM_CHANNEL_1;
            motor->dir_port = motor2_dir_GPIO_Port;
            motor->dir_pin = motor2_dir_Pin;
            motor->drv = &drv_motor2;
            break;

        case MOTOR_3:
            motor->htim = &htim4;
            motor->tim_channel = TIM_CHANNEL_1;
            motor->dir_port = motor3_dir_GPIO_Port;
            motor->dir_pin = motor3_dir_Pin;
            motor->drv = &drv_motor3;
            break;

        default:
            return HAL_ERROR;
    }

    motor->state = MOTOR_STATE_STOPPED;
    motor->direction = MOTOR_DIR_FORWARD;
    motor->duty_cycle = 0;
    motor->fault_count = 0;

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(motor->htim, &sConfigOC, motor->tim_channel);

    HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);

    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);

    status = MotorControl_ConfigureDRV8353(motor_id);
    if (status != HAL_OK) {
        snprintf(msg, sizeof(msg), "DRV8353 configuration failed for Motor %d\r\n", motor_id + 1);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        motor->state = MOTOR_STATE_FAULT;
        return status;
    }

    snprintf(msg, sizeof(msg), "✓ Motor %d initialized\r\n", motor_id + 1);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_ConfigureDRV8353(MotorID_t motor_id) {
    HAL_StatusTypeDef status;
    MotorControl_t *motor = &motor_ctrl[motor_id];
    uint16_t readback;
    char msg[128];

    uint16_t driver_ctrl = 0x0050;
    status = DRV8353_WriteRegister(motor->drv, DRV8353_REG_DRIVER_CONTROL, driver_ctrl);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    status = DRV8353_ReadRegister(motor->drv, DRV8353_REG_DRIVER_CONTROL, &readback);
    if (status != HAL_OK || (readback & 0x0070) != 0x0050) {
        snprintf(msg, sizeof(msg), " Motor %d: PWM mode verification failed (0x%04X)\r\n",
                 motor_id + 1, readback);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        return HAL_ERROR;
    }

    DRV8353_WriteRegister(motor->drv, DRV8353_REG_GATE_DRIVE_HS, 0x0333);
    HAL_Delay(10);
    DRV8353_WriteRegister(motor->drv, DRV8353_REG_GATE_DRIVE_LS, 0x0333);
    HAL_Delay(10);

    DRV8353_WriteRegister(motor->drv, DRV8353_REG_OCP_CONTROL, 0x001F);
    HAL_Delay(10);

    DRV8353_WriteRegister(motor->drv, DRV8353_REG_CSA_CONTROL, 0x0056);
    HAL_Delay(10);

    return HAL_OK;
}


HAL_StatusTypeDef MotorControl_Start(MotorID_t motor_id, uint8_t duty_cycle, MotorDirection_t direction) {
    MotorControl_t *motor = &motor_ctrl[motor_id];
    char msg[128];

    if (duty_cycle > 100) duty_cycle = 100;

    snprintf(msg, sizeof(msg), "Motor %d: Starting %d%% %s\r\n",
             motor_id + 1, duty_cycle,
             direction == MOTOR_DIR_FORWARD ? "FWD" : "REV");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);
    _MotorControl_SetPWM(motor_id, 0);

    _MotorControl_ApplyBrake(motor_id, true);
    HAL_Delay(20);

    GPIO_PinState dir_pin = (direction == MOTOR_DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, dir_pin);
    motor->direction = direction;
    HAL_Delay(20);

    _MotorControl_ApplyBrake(motor_id, false);
    HAL_Delay(100);

    HAL_TIM_PWM_Start(motor->htim, motor->tim_channel);

    if (duty_cycle > 0) {
        uint8_t kickstart_duty = (duty_cycle < 30) ? 30 : duty_cycle;
        _MotorControl_SetPWM(motor_id, kickstart_duty);
        HAL_Delay(100);

        if (kickstart_duty > duty_cycle) {
            for (int8_t d = kickstart_duty; d >= duty_cycle; d--) {
                _MotorControl_SetPWM(motor_id, d);
                HAL_Delay(30);
            }
        }
    }

    motor->duty_cycle = duty_cycle;
    motor->state = MOTOR_STATE_RUNNING;

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_Stop(MotorID_t motor_id) {
    MotorControl_t *motor = &motor_ctrl[motor_id];
    char msg[64];

    snprintf(msg, sizeof(msg), "Motor %d: Stopping\r\n", motor_id + 1);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    for (int8_t d = motor->duty_cycle; d >= 0; d--) {
        _MotorControl_SetPWM(motor_id, d);
        HAL_Delay(20);
    }

    HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);
    _MotorControl_SetPWM(motor_id, 0);

    motor->duty_cycle = 0;
    motor->state = MOTOR_STATE_STOPPED;

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_Brake(MotorID_t motor_id) {
    MotorControl_t *motor = &motor_ctrl[motor_id];
    char msg[64];

    snprintf(msg, sizeof(msg), "Motor %d: Braking\r\n", motor_id + 1);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);
    _MotorControl_SetPWM(motor_id, 0);
    _MotorControl_ApplyBrake(motor_id, true);

    motor->duty_cycle = 0;
    motor->state = MOTOR_STATE_BRAKING;

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_SetDuty(MotorID_t motor_id, uint8_t duty_cycle) {
    MotorControl_t *motor = &motor_ctrl[motor_id];

    if (duty_cycle > 100) duty_cycle = 100;

    _MotorControl_SetPWM(motor_id, duty_cycle);
    motor->duty_cycle = duty_cycle;

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_Ramp(MotorID_t motor_id, uint8_t target_duty, uint32_t ramp_time_ms) {
    MotorControl_t *motor = &motor_ctrl[motor_id];

    if (target_duty > 100) target_duty = 100;

    int8_t current = motor->duty_cycle;
    int8_t target = target_duty;
    int8_t delta = target - current;

    if (delta == 0) return HAL_OK;

    uint8_t steps = abs(delta);
    uint32_t delay_per_step = ramp_time_ms / steps;

    if (delta > 0) {
        // Ramp up
        for (uint8_t d = current; d <= target; d++) {
            _MotorControl_SetPWM(motor_id, d);
            HAL_Delay(delay_per_step);
        }
    } else {
        // Ramp down
        for (int8_t d = current; d >= target; d--) {
            _MotorControl_SetPWM(motor_id, d);
            HAL_Delay(delay_per_step);
        }
    }

    motor->duty_cycle = target_duty;

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_ChangeDirection(MotorID_t motor_id, MotorDirection_t new_direction, uint8_t duty_cycle) {
    MotorControl_t *motor = &motor_ctrl[motor_id];

    if (motor->direction == new_direction) {
        return MotorControl_SetDuty(motor_id, duty_cycle);
    }

    MotorControl_Stop(motor_id);
    HAL_Delay(100);
    return MotorControl_Start(motor_id, duty_cycle, new_direction);
}

HAL_StatusTypeDef MotorControl_CheckFaults(MotorID_t motor_id) {
    MotorControl_t *motor = &motor_ctrl[motor_id];

    DRV8353_ReadRegister(motor->drv, DRV8353_REG_FAULT_STATUS_1, &motor->last_fault_status);
    DRV8353_ReadRegister(motor->drv, DRV8353_REG_VGS_STATUS_2, &motor->last_vgs_status);

    if (motor->last_fault_status != 0x0000 || (motor->last_vgs_status & 0x0080)) {
        motor->fault_count++;
        motor->state = MOTOR_STATE_FAULT;
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_ClearFaults(MotorID_t motor_id) {
    MotorControl_t *motor = &motor_ctrl[motor_id];
    return DRV8353_ClearFaults(motor->drv);
}

MotorState_t MotorControl_GetState(MotorID_t motor_id) {
    return motor_ctrl[motor_id].state;
}

void MotorControl_PrintStatus(MotorID_t motor_id) {
    MotorControl_t *motor = &motor_ctrl[motor_id];
    char msg[256];

    MotorControl_CheckFaults(motor_id);

    const char* state_str[] = {"STOPPED", "RUNNING", "BRAKING", "FAULT"};
    const char* dir_str[] = {"FWD", "REV"};

    snprintf(msg, sizeof(msg),
             "Motor %d: %s %s %d%% | F1=0x%04X VGS=0x%04X | Faults=%lu\r\n",
             motor_id + 1,
             state_str[motor->state],
             dir_str[motor->direction],
             motor->duty_cycle,
             motor->last_fault_status,
             motor->last_vgs_status,
             motor->fault_count);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
}

void MotorControl_PrintAllStatus(void) {
    char msg[64];
    snprintf(msg, sizeof(msg), "\r\n=== Motor Status ===\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

    for (MotorID_t i = MOTOR_1; i < MOTOR_COUNT; i++) {
        MotorControl_PrintStatus(i);
        HAL_Delay(10);
    }

    snprintf(msg, sizeof(msg), "===================\r\n\r\n");
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
}


void MotorControl_EmergencyStopAll(void) {
    for (MotorID_t i = MOTOR_1; i < MOTOR_COUNT; i++) {
        MotorControl_t *motor = &motor_ctrl[i];
        HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);
        _MotorControl_SetPWM(i, 0);
        motor->duty_cycle = 0;
        motor->state = MOTOR_STATE_STOPPED;
    }
}


static void _MotorControl_ApplyBrake(MotorID_t motor_id, bool enable) {
    GPIO_PinState brake_state = enable ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, brake_state);
}

static void _MotorControl_SetPWM(MotorID_t motor_id, uint8_t duty_cycle) {
    MotorControl_t *motor = &motor_ctrl[motor_id];
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->htim);
    uint32_t ccr = (arr * duty_cycle) / 100;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, ccr);
}
