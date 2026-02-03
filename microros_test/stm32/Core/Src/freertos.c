/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>
#include "usb_cdc_transport.h"
#include "usbd_cdc_if.h"
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void cmd_vel_callback(const void * msgin);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rcl_node_t node;
rclc_support_t support;
rcl_publisher_t motor_pub;
rcl_publisher_t encoder_pub;
rclc_executor_t executor;
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

/* Messages */
std_msgs__msg__Float32MultiArray motor_msg;
std_msgs__msg__Int32MultiArray encoder_msg;

/* Static data buffers (NO dynamic alloc!) */
float motor_data[3];
int32_t encoder_data[3];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    /* --- USB CDC init --- */
    MX_USB_DEVICE_Init();
    osDelay(2000);

    /* --- Set custom transport --- */
    rmw_uros_set_custom_transport(
        true,
        NULL,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
		cubemx_transport_read);

    rcl_allocator_t allocator = rcl_get_default_allocator();

    /* --- Init support --- */
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        Error_Handler();
    }

    /* --- Create node --- */
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
    {
        // Toggle LED to indicate waiting for agent
        HAL_GPIO_TogglePin(debug_GPIO_Port, debug_Pin);
        osDelay(100);
    }

    /* --- Create node --- */
    if (rclc_node_init_default(&node, "mcu", "", &support) != RCL_RET_OK) {
        Error_Handler();
    }
    if (rclc_subscription_init_best_effort(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK)
    {
        Error_Handler();
    }

    /* --- Init publishers --- */
    if (rclc_publisher_init_best_effort(
            &motor_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/motors/current") != RCL_RET_OK) {

        Error_Handler();
    }

    if (rclc_publisher_init_best_effort(
            &encoder_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/encoders/absolute") != RCL_RET_OK) {

        Error_Handler();
    }

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA);

    motor_msg.data.data = motor_data;
    motor_msg.data.size = 3;
    motor_msg.data.capacity = 3;

    encoder_msg.data.data = encoder_data;
    encoder_msg.data.size = 3;
    encoder_msg.data.capacity = 3;

    int32_t enc1 = 0, enc2 = 0, enc3 = 0;

    for(;;)
    {
        motor_data[0] = ((float)rand() / (float)RAND_MAX) * 10.0f;
        motor_data[1] = ((float)rand() / (float)RAND_MAX) * 10.0f;
        motor_data[2] = ((float)rand() / (float)RAND_MAX) * 10.0f;

        encoder_data[0] = enc1;
        encoder_data[1] = enc2;
        encoder_data[2] = enc3;

        rcl_publish(&motor_pub, &motor_msg, NULL);
        rcl_publish(&encoder_pub, &encoder_msg, NULL);


        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        enc1 = (enc1 + 5) % 8192;
        enc2 = (enc2 + 3) % 8192;
        enc3 = (enc3 + 7) % 8192;

        osDelay(10);
    }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear = msg->linear.x;
    float angular = msg->angular.z;

}

/* USER CODE END Application */

