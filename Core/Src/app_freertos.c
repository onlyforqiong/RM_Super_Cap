/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LedshineHandle;
osThreadId CapBuckControlHandle;
osThreadId Boost_ControlHandle;
osThreadId Ref_Power_LimitHandle;
osThreadId Power_ControlHandle;
osThreadId Feed_DogHandle;
osThreadId Error_DetectHandle;
osThreadId BlueToothHandle;
osThreadId Oled_ShowHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Task_Ledshine(void const * argument);
void Task_CapBuckControl(void const * argument);
void Task_Boost_Control(void const * argument);
void Task_Ref_Power_Limit(void const * argument);
void Task_Power_Control(void const * argument);
void Task_Feed_Dog(void const * argument);
void Task_Error_Detect(void const * argument);
void Task_BlueTooth(void const * argument);
void Task_Oled_Show(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Ledshine */
  osThreadDef(Ledshine, Task_Ledshine, osPriorityIdle, 0, 128);
  LedshineHandle = osThreadCreate(osThread(Ledshine), NULL);

  /* definition and creation of CapBuckControl */
  osThreadDef(CapBuckControl, Task_CapBuckControl, osPriorityRealtime, 0, 512);
  CapBuckControlHandle = osThreadCreate(osThread(CapBuckControl), NULL);

  /* definition and creation of Boost_Control */
  osThreadDef(Boost_Control, Task_Boost_Control, osPriorityIdle, 0, 256);
  Boost_ControlHandle = osThreadCreate(osThread(Boost_Control), NULL);

  /* definition and creation of Ref_Power_Limit */
  osThreadDef(Ref_Power_Limit, Task_Ref_Power_Limit, osPriorityIdle, 0, 256);
  Ref_Power_LimitHandle = osThreadCreate(osThread(Ref_Power_Limit), NULL);

  /* definition and creation of Power_Control */
  osThreadDef(Power_Control, Task_Power_Control, osPriorityHigh, 0, 256);
  Power_ControlHandle = osThreadCreate(osThread(Power_Control), NULL);

  /* definition and creation of Feed_Dog */
  osThreadDef(Feed_Dog, Task_Feed_Dog, osPriorityNormal, 0, 128);
  Feed_DogHandle = osThreadCreate(osThread(Feed_Dog), NULL);

  /* definition and creation of Error_Detect */
  osThreadDef(Error_Detect, Task_Error_Detect, osPriorityIdle, 0, 256);
  Error_DetectHandle = osThreadCreate(osThread(Error_Detect), NULL);

  /* definition and creation of BlueTooth */
  osThreadDef(BlueTooth, Task_BlueTooth, osPriorityNormal, 0, 512);
  BlueToothHandle = osThreadCreate(osThread(BlueTooth), NULL);

  /* definition and creation of Oled_Show */
  osThreadDef(Oled_Show, Task_Oled_Show, osPriorityNormal, 0, 256);
  Oled_ShowHandle = osThreadCreate(osThread(Oled_Show), NULL);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Task_Ledshine */
/**
* @brief Function implementing the Ledshine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Ledshine */
__weak void Task_Ledshine(void const * argument)
{
  /* USER CODE BEGIN Task_Ledshine */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Ledshine */
}

/* USER CODE BEGIN Header_Task_CapBuckControl */
/**
* @brief Function implementing the CapBuckControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_CapBuckControl */
__weak void Task_CapBuckControl(void const * argument)
{
  /* USER CODE BEGIN Task_CapBuckControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_CapBuckControl */
}

/* USER CODE BEGIN Header_Task_Boost_Control */
/**
* @brief Function implementing the Boost_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Boost_Control */
__weak void Task_Boost_Control(void const * argument)
{
  /* USER CODE BEGIN Task_Boost_Control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Boost_Control */
}

/* USER CODE BEGIN Header_Task_Ref_Power_Limit */
/**
* @brief Function implementing the Ref_Power_Limit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Ref_Power_Limit */
__weak void Task_Ref_Power_Limit(void const * argument)
{
  /* USER CODE BEGIN Task_Ref_Power_Limit */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Ref_Power_Limit */
}

/* USER CODE BEGIN Header_Task_Power_Control */
/**
* @brief Function implementing the Power_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Power_Control */
__weak void Task_Power_Control(void const * argument)
{
  /* USER CODE BEGIN Task_Power_Control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Power_Control */
}

/* USER CODE BEGIN Header_Task_Feed_Dog */
/**
* @brief Function implementing the Feed_Dog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Feed_Dog */
__weak void Task_Feed_Dog(void const * argument)
{
  /* USER CODE BEGIN Task_Feed_Dog */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Feed_Dog */
}

/* USER CODE BEGIN Header_Task_Error_Detect */
/**
* @brief Function implementing the Error_Detect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Error_Detect */
__weak void Task_Error_Detect(void const * argument)
{
  /* USER CODE BEGIN Task_Error_Detect */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Error_Detect */
}

/* USER CODE BEGIN Header_Task_BlueTooth */
/**
* @brief Function implementing the BlueTooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_BlueTooth */
__weak void Task_BlueTooth(void const * argument)
{
  /* USER CODE BEGIN Task_BlueTooth */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_BlueTooth */
}

/* USER CODE BEGIN Header_Task_Oled_Show */
/**
* @brief Function implementing the Oled_Show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Oled_Show */
__weak void Task_Oled_Show(void const * argument)
{
  /* USER CODE BEGIN Task_Oled_Show */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Oled_Show */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

