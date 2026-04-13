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
#include "buzzer.h"
#include "ws2812.h"
#include "remote_control.h"
#include "bmi088.h"

#include "referee_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern RC_ctrl_t *rc_ctl;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t UI_TaskHandle;
uint32_t UI_TaskBuffer[512];
StaticTask_t UI_TaskControlBlock;
const osThreadAttr_t UI_Task_attributes = {
    .name = "UI_Task",
    .cb_mem = &UI_TaskControlBlock,
    .cb_size = sizeof(UI_TaskControlBlock),
    .stack_mem = &UI_TaskBuffer[0],
    .stack_size = sizeof(UI_TaskBuffer),
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void Start_UI_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  UI_TaskHandle = osThreadNew(Start_UI_Task, NULL, &UI_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
uint32_t beat = 0;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	WS2812_Control(ws2812_instance, GREEN_WS2812_COLOR);
	Buzzer_Play(StartUP_sound);
	
  /* Infinite loop */
  for(;;)
  {
		beat++;
		if((beat % 30000) == 0)
		{
			Buzzer_Play(Warming_sound);
		}
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Start_UI_Task(void *argument)
{
  /* USER CODE BEGIN Start_UI_Task */
  uint8_t first_flag = 0;
  osDelay(1500);
  /* Infinite loop */
  for (;;)
  {
    static uint8_t last_key_cnt[16] = {0};
    static uint8_t key_mode_last = 0;
#define KEY_CLICK(k) (rc_ctl->key_count[KEY_PRESS][(k)] != last_key_cnt[(k)])
#define KEY_ACK(k) (last_key_cnt[(k)] = rc_ctl->key_count[KEY_PRESS][(k)])
    if (first_flag == 0)
    {
      User_UI_Init();
      first_flag = 1;
    }
    if (KEY_CLICK(Key_X))
    {
      User_UI_Init();
      osDelay(910);
      KEY_ACK(Key_X);
    }
#undef KEY_CLICK
#undef KEY_ACK
    UI_Task();
  }
  /* USER CODE END Start_UI_Task */
}


/* USER CODE END Application */

