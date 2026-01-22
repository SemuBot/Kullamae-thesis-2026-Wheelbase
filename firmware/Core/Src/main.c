/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Switched to MOTOR 2
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmd_vel.h"
#include "motor.h"
#include "drv8353.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CmdVel cmd_vel_data;
extern motor_st motor1, motor2, motor3;
extern DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t test_cycle = 0;
char debug_msg[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_CAN_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(5000);  // Wait for USB enumeration

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
  HAL_Delay(100);


  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // Motor 1 (TIM1 CH1)
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  // Motor 2 (TIM3 CH1)
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

  // Motor 3 (TIM4 CH1)
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);

  snprintf(debug_msg, sizeof(debug_msg), "Setting ENABLE HIGH...\r\n");
  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));
  HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // Test SPI
  snprintf(debug_msg, sizeof(debug_msg), "Testing SPI M1, M2, M3...\r\n");
  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));
  DRV8353_RawSPITest(&drv_motor1);
  DRV8353_RawSPITest(&drv_motor2);
  DRV8353_RawSPITest(&drv_motor3);


  uint16_t driver_ctrl = 0x0040; // 1x PWM Mode
  uint16_t gate_hs_ctrl = 0x0300;
  uint16_t gate_ls_ctrl = 0x0300;
  uint16_t ocp_ctrl = 0x0129;
  uint16_t csa_ctrl = 0x0123;

  // M1
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_DRIVER_CONTROL, driver_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_GATE_DRIVE_HS, gate_hs_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_GATE_DRIVE_LS, gate_ls_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_OCP_CONTROL, ocp_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_CSA_CONTROL, csa_ctrl);

  // M2
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_DRIVER_CONTROL, driver_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_GATE_DRIVE_HS, gate_hs_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_GATE_DRIVE_LS, gate_ls_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_OCP_CONTROL, ocp_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_CSA_CONTROL, csa_ctrl);

  // M3
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_DRIVER_CONTROL, driver_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_GATE_DRIVE_HS, gate_hs_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_GATE_DRIVE_LS, gate_ls_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_OCP_CONTROL, ocp_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_CSA_CONTROL, csa_ctrl);

  snprintf(debug_msg, sizeof(debug_msg), "Configuration Complete.\r\n");
  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));

  // Release Brake
  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // Calibration
  DRV8353_CalibrateCSAs(&drv_motor1);
  DRV8353_CalibrateCSAs(&drv_motor2);
  DRV8353_CalibrateCSAs(&drv_motor3);
  // Set Directions Forward initially
  HAL_GPIO_WritePin(motor1.dir_port, motor1.dir_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor2.dir_port, motor2.dir_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor3.dir_port, motor3.dir_pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      static uint32_t last_control_change = 0;
      static uint8_t control_state = 0;
      uint32_t current_time = HAL_GetTick();
      // Monitoring fautls
      static uint32_t last_log = 0;
      if (current_time - last_log > 2000) {
          last_log = current_time;
          uint16_t f1, vgs;

          DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_FAULT_STATUS_1, &f1);
          DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_VGS_STATUS_2, &vgs);
          snprintf(debug_msg, sizeof(debug_msg), "[%lu] M3 Status: F1=0x%04X, VGS=0x%04X\r\n", current_time, f1, vgs);
          CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));
      }

      if (current_time - last_control_change > 5000) {
          last_control_change = current_time;
          uint32_t arr_m1 = __HAL_TIM_GET_AUTORELOAD(&htim1);
          uint32_t arr_m2 = __HAL_TIM_GET_AUTORELOAD(&htim3);
          uint32_t arr_m3 = __HAL_TIM_GET_AUTORELOAD(&htim4);

          uint16_t drv_ctrl_m1, drv_ctrl_m2, drv_ctrl_m3;
          GPIO_PinState dir_state_m1, dir_state_m2, dir_state_m3;

          switch(control_state) {
              case 0:  // STOP
                  snprintf(debug_msg, sizeof(debug_msg), "\r\n[%lu] MOTORS STOP\r\n", current_time);
                  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));

                  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

                  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_RESET); // Brake ON

                  control_state = 1;
                  break;

              case 1:  // FORWARD 10% (ALL)
                  snprintf(debug_msg, sizeof(debug_msg), "\r\n[%lu] ALL FORWARD 10%%\r\n", current_time);
                  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));

                  // Stop & Brake
                  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

                  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_RESET);
                  HAL_Delay(50);

                  //Set Direction (SET = Forward)
                  HAL_GPIO_WritePin(motor1.dir_port, motor1.dir_pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(motor2.dir_port, motor2.dir_pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(motor3.dir_port, motor3.dir_pin, GPIO_PIN_RESET);
                  HAL_Delay(100);

                  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_SET);
                  HAL_Delay(100);

                  __HAL_TIM_MOE_ENABLE(&htim1);
                  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
                  HAL_Delay(50);


                  // Ramp
                  for (uint8_t duty = 5; duty <= 20; duty++) {
                      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr_m1 * duty / 100);
                      //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_m2 * duty / 100);
                      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arr_m3 * duty / 100);
                      HAL_Delay(40);
                  }
                  control_state = 2;
                  break;

              case 2:  // STOP
                  snprintf(debug_msg, sizeof(debug_msg), "\r\n[%lu] MOTORS STOP\r\n", current_time);
                  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));

                  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
                  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_RESET);

                  control_state = 3;
                  break;

              case 3:  // REVERSE
                  snprintf(debug_msg, sizeof(debug_msg), "\r\n[%lu] ALL REVERSE 10%%\r\n", current_time);
                  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));

                  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

                  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_RESET);
                  HAL_Delay(50);

                  HAL_GPIO_WritePin(motor1.dir_port, motor1.dir_pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(motor2.dir_port, motor2.dir_pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(motor3.dir_port, motor3.dir_pin, GPIO_PIN_SET);
                  HAL_Delay(100);

                  HAL_GPIO_WritePin(nBRAKE_GPIO_Port, nBRAKE_Pin, GPIO_PIN_SET);
                  HAL_Delay(100);

                  __HAL_TIM_MOE_ENABLE(&htim1);
                  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
                  uint16_t drv_ctrl;

                  DRV8353_ReadRegister(&drv_motor2, DRV8353_REG_DRIVER_CONTROL, &drv_ctrl);
                  snprintf(debug_msg, sizeof(debug_msg), "DRV_CTRL: 0x%04X (bit6=%d)\r\n",
                           drv_ctrl, (drv_ctrl >> 6) & 1);
                  CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));
                  HAL_Delay(50);
                  // Ramp
                  for (int duty = 5; duty <= 20; duty++) {
                      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr_m1 * duty / 100);
                      //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_m2 * duty / 100);
                      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arr_m3 * duty / 100);
                      HAL_Delay(30);
                  }

                  control_state = 0;
                  break;
          }
      }
      HAL_Delay(50);
  }
  /* USER CODE END 3 */
}


// ================================
// 1. Calibrate Current Sense Amplifiers
// ================================
void DRV8353_CalibrateCSAs(DRV8353_Handle *drv)
{
    uint16_t csa_ctrl;
    char msg[128];

    // Read current CSA_CTRL
    DRV8353_ReadRegister(drv, DRV8353_REG_CAL_CONTROL, &csa_ctrl);

    //Set CAL_MODE = 1
    csa_ctrl |= 0x0001;  // Bit0 = 1
    DRV8353_WriteRegister(drv, DRV8353_REG_CAL_CONTROL, csa_ctrl);

    snprintf(msg, sizeof(msg), "DRV8353: Starting CSA Auto-Calibration...\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

    // Wait for calibration to complete
    HAL_Delay(10);

    // Clear CAL_MODE = 0 to return to normal operation
    csa_ctrl &= ~0x0001;
    DRV8353_WriteRegister(drv, DRV8353_REG_CAL_CONTROL, csa_ctrl);

    snprintf(msg, sizeof(msg), "DRV8353: CSA Auto-Calibration Complete.\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void DRV8353_RawSPITest(DRV8353_Handle *drv) {
    char msg[256];
    uint8_t tx_buf[4], rx_buf[4];

    // Read Register 0x02
    tx_buf[0] = 0x90;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);
    for(volatile uint32_t i = 0; i < 20; i++);
    HAL_SPI_TransmitReceive(drv->hspi, tx_buf, rx_buf, 2, 100);
    for(volatile uint32_t i = 0; i < 20; i++);
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    snprintf(msg, sizeof(msg), "  Response: [0x%02X 0x%02X]\r\n", rx_buf[0], rx_buf[1]);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
}
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}
void Error_Handler(void){__disable_irq();while (1){}}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
