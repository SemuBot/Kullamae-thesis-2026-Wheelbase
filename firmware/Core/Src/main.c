/* ===========================================================================
 * main.c - Clean Application Entry Point
 *
 * Purpose: Minimal main file - just hardware init and main loop
 * All robot-specific logic is in robot_control.c
 * =========================================================================== */

#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "robot_control.h"
#include "motor.h"
#include "cmd_vel.h"

/* Global Variables */
CmdVel cmd_vel_data = {0};

/* Function Prototypes */
void SystemClock_Config(void);

/**
  * @brief  Application entry point
  * @retval int
  */
int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize Peripherals */
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USB_DEVICE_Init();
    MX_CAN_Init();
    MX_SPI3_Init();

    /* Wait for USB enumeration */
    HAL_Delay(5000);

    /* Initialize Robot System */
    if (Robot_Init() != HAL_OK) {
        // Initialization failed - halt
        Error_Handler();
    }

    /* Send help message */
    Send_CmdVel_Help();

    /* Main Loop Variables */
    uint32_t last_timeout_check = 0;
    uint32_t last_motor_update = 0;
    uint32_t last_fault_check = 0;

    /* Infinite Loop */
    while (1)
    {
        uint32_t current_time = HAL_GetTick();

        /* Update motor PWM outputs at 100Hz */
        if (current_time - last_motor_update >= 10) {
            last_motor_update = current_time;
            Motor_UpdateAll();
        }

        /* Check cmd_vel timeout every 100ms */
        if (current_time - last_timeout_check >= 100) {
            last_timeout_check = current_time;
            Check_CmdVel_Timeout();
        }

        /* Monitor faults every 2 seconds */
        if (current_time - last_fault_check >= 2000) {
            last_fault_check = current_time;
            Robot_MonitorFaults();
        }

        HAL_Delay(1);  // Prevent tight loop
    }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                   RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_TIM1|
                                          RCC_PERIPHCLK_TIM34;
    PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
    PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/**
  * @brief  Error handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        // Optionally blink an LED or send error via USB
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report the error */
}
#endif
