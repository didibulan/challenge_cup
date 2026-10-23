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
osThreadId DebugTaskHandle;
osThreadId Chassis_TaskHandle;
osThreadId Comm_TaskHandle;
osThreadId Referee_TaskHandle;
osThreadId Command_TaskHandle;
osThreadId Init_TaskHandle;
osThreadId IMU_TaskHandle;
osThreadId UI_TaskHandle;
osMutexId CommMutexHandle;
osMutexId RefMutexHandle;
osMutexId CmdMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void App_DebugTask(void const * argument);
extern void App_ChassisTask(void const * argument);
extern void App_CommTask(void const * argument);
extern void App_RefereeTask(void const * argument);
extern void App_CommandTask(void const * argument);
void App_Init(void const * argument);
extern void App_IMUTask(void const * argument);
extern void App_UiTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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
  /* Create the mutex(es) */
  /* definition and creation of CommMutex */
  osMutexDef(CommMutex);
  CommMutexHandle = osMutexCreate(osMutex(CommMutex));

  /* definition and creation of RefMutex */
  osMutexDef(RefMutex);
  RefMutexHandle = osMutexCreate(osMutex(RefMutex));

  /* definition and creation of CmdMutex */
  osMutexDef(CmdMutex);
  CmdMutexHandle = osMutexCreate(osMutex(CmdMutex));

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
  /* definition and creation of DebugTask */
  osThreadDef(DebugTask, App_DebugTask, osPriorityAboveNormal, 0, 1024);
  DebugTaskHandle = osThreadCreate(osThread(DebugTask), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, App_ChassisTask, osPriorityAboveNormal, 0, 1024);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of Comm_Task */
  osThreadDef(Comm_Task, App_CommTask, osPriorityAboveNormal, 0, 1024);
  Comm_TaskHandle = osThreadCreate(osThread(Comm_Task), NULL);

  /* definition and creation of Referee_Task */
  osThreadDef(Referee_Task, App_RefereeTask, osPriorityAboveNormal, 0, 1024);
  Referee_TaskHandle = osThreadCreate(osThread(Referee_Task), NULL);

  /* definition and creation of Command_Task */
  osThreadDef(Command_Task, App_CommandTask, osPriorityAboveNormal, 0, 206);
  Command_TaskHandle = osThreadCreate(osThread(Command_Task), NULL);

  /* definition and creation of Init_Task */
  osThreadDef(Init_Task, App_Init, osPriorityRealtime, 0, 206);
  Init_TaskHandle = osThreadCreate(osThread(Init_Task), NULL);

  /* definition and creation of IMU_Task */
  osThreadDef(IMU_Task, App_IMUTask, osPriorityHigh, 0, 1024);
  IMU_TaskHandle = osThreadCreate(osThread(IMU_Task), NULL);

  /* definition and creation of UI_Task */
  osThreadDef(UI_Task, App_UiTask, osPriorityAboveNormal, 0, 206);
  UI_TaskHandle = osThreadCreate(osThread(UI_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_App_DebugTask */
/**
  * @brief  Function implementing the DebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_App_DebugTask */
__weak void App_DebugTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN App_DebugTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_DebugTask */
}

/* USER CODE BEGIN Header_App_Init */
/**
* @brief Function implementing the Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_Init */
__weak void App_Init(void const * argument)
{
  /* USER CODE BEGIN App_Init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
