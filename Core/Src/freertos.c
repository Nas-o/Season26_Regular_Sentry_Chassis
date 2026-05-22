/*
 * @Date: 2025-10-24 19:37:34
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-21 09:05:30
 * @FilePath: \Season26_Regular_Sentry_Chassis\Core\Src\freertos.c
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "remote_control.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Global_status.h"
#include "Auto_control.h"
#include "shoot.h"
#include "music.h"

#include "referee_system.h"
#include "supercup.h"
#include "LED.h"
#include "DT7.h"
#include "VT13.h"
#include "motor.h"
#include <cmsis_os2.h>

#include "iwdg.h"
#include "buzzer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t color = 0;
uint8_t referee_data[256];
static uint8_t referee_tx_buffer[128];
/* USER CODE END Variables */
/* Definitions for Remote_control */
osThreadId_t Remote_controlHandle;
const osThreadAttr_t Remote_control_attributes = {
  .name = "Remote_control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gimbal */
osThreadId_t GimbalHandle;
const osThreadAttr_t Gimbal_attributes = {
  .name = "Gimbal",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_control */
osThreadId_t Motor_controlHandle;
const osThreadAttr_t Motor_control_attributes = {
  .name = "Motor_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Referee */
osThreadId_t RefereeHandle;
const osThreadAttr_t Referee_attributes = {
  .name = "Referee",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Log_and_debug */
osThreadId_t Log_and_debugHandle;
const osThreadAttr_t Log_and_debug_attributes = {
  .name = "Log_and_debug",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Remote_control_Task(void *argument);
void Gimbal_Task(void *argument);
void Chassis_Task(void *argument);
void Motor_control_Task(void *argument);
void Shoot_Task(void *argument);
void Referee_Task(void *argument);
void Log_and_debug_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

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
  /* creation of Remote_control */
  Remote_controlHandle = osThreadNew(Remote_control_Task, NULL, &Remote_control_attributes);

  /* creation of Gimbal */
  GimbalHandle = osThreadNew(Gimbal_Task, NULL, &Gimbal_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(Chassis_Task, NULL, &Chassis_attributes);

  /* creation of Motor_control */
  Motor_controlHandle = osThreadNew(Motor_control_Task, NULL, &Motor_control_attributes);

  /* creation of Shoot */
  ShootHandle = osThreadNew(Shoot_Task, NULL, &Shoot_attributes);

  /* creation of Referee */
  RefereeHandle = osThreadNew(Referee_Task, NULL, &Referee_attributes);

  /* creation of Log_and_debug */
  Log_and_debugHandle = osThreadNew(Log_and_debug_Task, NULL, &Log_and_debug_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Remote_control_Task */
/**
  * @brief  Function implementing the Remote_control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Remote_control_Task */
void Remote_control_Task(void *argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN Remote_control_Task */
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_control_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  osDelay(500);
  Gimbal_Init();
  /* Infinite loop */
  for(;;)
  {
    Gimbal_Tasks();
    Gimbal_CAN_SendAll();
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  osDelay(500);
  Chassis_Init();
  /* Infinite loop */
  for(;;)
  {
    Chassis_Tasks();
    Supercup_SendData();
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Motor_control_Task */
/**
* @brief Function implementing the Motor_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_control_Task */
void Motor_control_Task(void *argument)
{
  /* USER CODE BEGIN Motor_control_Task */
  /* Infinite loop */
  for(;;)
  {
    /* 0: 发送 DMMotor, 1: 发送 DJI */
    static uint8_t send_toggle = 0;
    static uint8_t can_send_clock = 0;
      /* DMMotor124_SendCurrent(DM_J4310_1_4_ID,DM_CAN_1); */
      if(send_toggle)
      {
        DMMotor_SendCtrl(DM_CAN_1_1);
        DJIMotor_SendCurrent(CAN_6020_1_4_ID, DJI_CAN_3);
        DJIMotor_SendCurrent(CAN_6020_1_4_ID, DJI_CAN_1);
        DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_1);
        DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_3);
      }
      else
      {
        DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_1);
        DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_3); 
        DJIMotor_SendCurrent(CAN_6020_1_4_ID, DJI_CAN_3);
        DJIMotor_SendCurrent(CAN_6020_1_4_ID, DJI_CAN_1);
        DMMotor_SendCtrl(DM_CAN_1_1);
      }
   /*  else
    {
        uint8_t zero_data[8] = {0};
        
        
        Fdcanx_SendData(&hfdcan1, CAN_20063508_1_4_send_ID, zero_data, 8);
        Fdcanx_SendData(&hfdcan1, CAN_6020_1_4_send_ID, zero_data, 8);
        Fdcanx_SendData(&hfdcan3, CAN_20063508_1_4_send_ID, zero_data, 8);
        Fdcanx_SendData(&hfdcan3, CAN_6020_1_4_send_ID, zero_data, 8); 
    } */
    send_toggle ^= 1;
    
    if (Global.Chassis.input.reset != 1)
      HAL_IWDG_Refresh(&hiwdg1); 
    osDelay(1);
  }
  /* USER CODE END Motor_control_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
void Shoot_Task(void *argument)
{
  /* USER CODE BEGIN Shoot_Task */
  osDelay(500);
  Shoot_Init();
  /* Infinite loop */
  for(;;)
  {
  Shoot_Tasks();
  osDelay(1);

  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the Referee thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
void Referee_Task(void *argument)
{
  /* USER CODE BEGIN Referee_Task */
  Refree_system_init();
  /* Infinite loop */
  for(;;)
  {
    Referee_unpack_fifo_data(&referee_fifo, &referee_unpack_obj);
 // 自动复活逻辑：当血量为0时，发送复活确认指令
    if (robot_status.current_HP == 0)
    {
        Sentry_cmd_1.check_revive_status = 1;
    }
    else
    {
        Sentry_cmd_1.check_revive_status = 0;
    }
    // if(Referee_data.rest_time < 100 && Referee_data.projectile_allowance_17mm
    // 设置哨兵想要兑换的发弹量值为 100
    /* Sentry_cmd_1.request_increase_ammo_number = 100; */
    Sentry_cmd_1.sentry_posture = Get_Sentry_Posture_From_Aligning(Chassis.is_aligning);
    // Sentry_cmd_1.activate_energy_mechanism = 1;
    Decision_Transmission(&Sentry_cmd_1, referee_tx_buffer);
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}

/* USER CODE BEGIN Header_Log_and_debug_Task */
/**
* @brief Function implementing the Log_and_debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Log_and_debug_Task */
void Log_and_debug_Task(void *argument)
{
  /* USER CODE BEGIN Log_and_debug_Task */

  /* Infinite loop */
  for(;;)
  {
  LED_ShowColor(RED);
  osDelay(500);
  LED_ShowColor(BLUE);
  osDelay(500);
  LED_ShowColor(GREEN);
  }
  /* USER CODE END Log_and_debug_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

