/*
 * drv8353.c
 *
 */

#include "drv8353.h"
#include <string.h>

#define SPI_TIMEOUT 100

/**
 * @brief Read a register from DRV8353
 * @param drv: Pointer to DRV8353 handle
 * @param reg_addr: Register address (0x00-0x06)
 * @param data: Pointer to store read data
 * @retval HAL status
 */
HAL_StatusTypeDef DRV8353_ReadRegister(DRV8353_Handle *drv, uint8_t reg_addr, uint16_t *data)
{
    HAL_StatusTypeDef status;
    uint16_t tx_buffer;
    uint8_t tx_buf[2], rx_buf[2];

    // Construct read command: bit 15 = 1 (read), bits 14-11 = address
    tx_buffer = DRV8353_SPI_READ_CMD | ((reg_addr & 0x0F) << 11);

    // Convert to bytes (MSB first)
    tx_buf[0] = (tx_buffer >> 8) & 0xFF;
    tx_buf[1] = tx_buffer & 0xFF;

    // CS low
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);
    for(volatile uint32_t i = 0; i < 10; i++);

    // SPI transaction
    status = HAL_SPI_TransmitReceive(drv->hspi, tx_buf, rx_buf, 2, SPI_TIMEOUT);

    for(volatile uint32_t i = 0; i < 10; i++);
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        uint16_t response = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
        *data = response & 0x07FF;  // 11-bit data mask
    }

    return status;
}


HAL_StatusTypeDef DRV8353_WriteRegister(DRV8353_Handle *drv, uint8_t reg_addr, uint16_t data)
{
    HAL_StatusTypeDef status;
    uint16_t tx_buffer;
    uint8_t tx_buf[2], rx_buf[2];

    tx_buffer = DRV8353_SPI_WRITE_CMD | ((reg_addr & 0x0F) << 11) | (data & DRV8353_SPI_DATA_MASK);

    tx_buf[0] = (tx_buffer >> 8) & 0xFF;
    tx_buf[1] = tx_buffer & 0xFF;

    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);
    for(volatile uint32_t i = 0; i < 10; i++);

    status = HAL_SPI_TransmitReceive(drv->hspi, tx_buf, rx_buf, 2, SPI_TIMEOUT);

    for(volatile uint32_t i = 0; i < 10; i++);
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    return status;
}


HAL_StatusTypeDef DRV8353_Configure(DRV8353_Handle *drv)
{
    HAL_StatusTypeDef status;

    // Driver Control: 1x PWM mode
    status = DRV8353_WriteRegister(drv, DRV8353_REG_DRIVER_CONTROL, 0x0040);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Gate Drive HS
    status = DRV8353_WriteRegister(drv, DRV8353_REG_GATE_DRIVE_HS, 0x0300);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Gate Drive LS
    status = DRV8353_WriteRegister(drv, DRV8353_REG_GATE_DRIVE_LS, 0x0300);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // OCP Control
    status = DRV8353_WriteRegister(drv, DRV8353_REG_OCP_CONTROL, 0x0129);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // CSA Control
    status = DRV8353_WriteRegister(drv, DRV8353_REG_CSA_CONTROL, 0x0123);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    drv->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef DRV8353_ClearFaults(DRV8353_Handle *drv)
{
    HAL_StatusTypeDef status;

    // Set CLR_FLT bit (bit 0 of Driver Control register)
    status = DRV8353_WriteRegister(drv, DRV8353_REG_DRIVER_CONTROL, 0x0041);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Clear CLR_FLT bit
    status = DRV8353_WriteRegister(drv, DRV8353_REG_DRIVER_CONTROL, 0x0040);
    HAL_Delay(10);

    return status;
}


HAL_StatusTypeDef DRV8353_CalibrateCSA(DRV8353_Handle *drv)
{
    HAL_StatusTypeDef status;
    uint16_t cal_ctrl;

    // Read current calibration control register
    status = DRV8353_ReadRegister(drv, DRV8353_REG_CAL_CONTROL, &cal_ctrl);
    if (status != HAL_OK) return status;

    // Set CAL_MODE bit
    cal_ctrl |= 0x0001;
    status = DRV8353_WriteRegister(drv, DRV8353_REG_CAL_CONTROL, cal_ctrl);
    if (status != HAL_OK) return status;

    // Wait for calibration
    HAL_Delay(10);

    // Clear CAL_MODE bit
    cal_ctrl &= ~0x0001;
    status = DRV8353_WriteRegister(drv, DRV8353_REG_CAL_CONTROL, cal_ctrl);

    return status;
}

/* ============================================================================
 * FAULT READING
 * ============================================================================ */

HAL_StatusTypeDef DRV8353_ReadFaults(DRV8353_Handle *drv)
{
    HAL_StatusTypeDef status;

    // Read Fault Status 1
    status = DRV8353_ReadRegister(drv, DRV8353_REG_FAULT_STATUS_1, &drv->fault_status_1);
    if (status != HAL_OK) return status;

    // Read VGS Status 2
    status = DRV8353_ReadRegister(drv, DRV8353_REG_VGS_STATUS_2, &drv->vgs_status_2);

    return status;
}


bool DRV8353_HasFault(DRV8353_Handle *drv)
{
    return (drv->fault_status_1 != 0 || drv->vgs_status_2 != 0);
}

HAL_StatusTypeDef DRV8353_Enable(DRV8353_Handle *drv)
{
    uint16_t reg_data;
    HAL_StatusTypeDef status;

    status = DRV8353_ReadRegister(drv, DRV8353_REG_DRIVER_CONTROL, &reg_data);
    if (status != HAL_OK) return status;

    // Clear brake and coast bits
    reg_data &= ~(DRV8353_BRAKE | DRV8353_COAST);

    return DRV8353_WriteRegister(drv, DRV8353_REG_DRIVER_CONTROL, reg_data);
}

HAL_StatusTypeDef DRV8353_Disable(DRV8353_Handle *drv)
{
    uint16_t reg_data;
    HAL_StatusTypeDef status;

    status = DRV8353_ReadRegister(drv, DRV8353_REG_DRIVER_CONTROL, &reg_data);
    if (status != HAL_OK) return status;

    // Set coast bit
    reg_data |= DRV8353_COAST;

    return DRV8353_WriteRegister(drv, DRV8353_REG_DRIVER_CONTROL, reg_data);
}
