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
osThreadId Init_TaskHandle;
osThreadId Arm_TaskHandle;
osThreadId Comm_TaskHandle;
osThreadId Alg_TaskHandle;
osThreadId Referee_TaskHandle;
osThreadId IMU_TaskHandle;
osThreadId StateMachine_TaskHandle;
osMutexId RoboticAlgMutexHandle;
osMutexId RemoteCtrlMutexHandle;
osMutexId CommMutexHandle;
osMutexId RefereeMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void App_Init(void const * argument);
extern void App_ArmTask(void const * argument);
extern void App_CommTask(void const * argument);
extern void App_AlgTask(void const * argument);
extern void App_RefereeTask(void const * argument);
extern void App_IMUTask(void const * argument);
extern void App_StateMachineTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of RoboticAlgMutex */
  osMutexDef(RoboticAlgMutex);
  RoboticAlgMutexHandle = osMutexCreate(osMutex(RoboticAlgMutex));

  /* definition and creation of RemoteCtrlMutex */
  osMutexDef(RemoteCtrlMutex);
  RemoteCtrlMutexHandle = osMutexCreate(osMutex(RemoteCtrlMutex));

  /* definition and creation of CommMutex */
  osMutexDef(CommMutex);
  CommMutexHandle = osMutexCreate(osMutex(CommMutex));

  /* definition and creation of RefereeMutex */
  osMutexDef(RefereeMutex);
  RefereeMutexHandle = osMutexCreate(osMutex(RefereeMutex));

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
  /* definition and creation of Init_Task */
  osThreadDef(Init_Task, App_Init, osPriorityRealtime, 0, 1024);
  Init_TaskHandle = osThreadCreate(osThread(Init_Task), NULL);

  /* definition and creation of Arm_Task */
  osThreadDef(Arm_Task, App_ArmTask, osPriorityAboveNormal, 0, 1024);
  Arm_TaskHandle = osThreadCreate(osThread(Arm_Task), NULL);

  /* definition and creation of Comm_Task */
  osThreadDef(Comm_Task, App_CommTask, osPriorityHigh, 0, 1024);
  Comm_TaskHandle = osThreadCreate(osThread(Comm_Task), NULL);

  /* definition and creation of Alg_Task */
  osThreadDef(Alg_Task, App_AlgTask, osPriorityNormal, 0, 4096);
  Alg_TaskHandle = osThreadCreate(osThread(Alg_Task), NULL);

  /* definition and creation of Referee_Task */
  osThreadDef(Referee_Task, App_RefereeTask, osPriorityHigh, 0, 2048);
  Referee_TaskHandle = osThreadCreate(osThread(Referee_Task), NULL);

  /* definition and creation of IMU_Task */
  osThreadDef(IMU_Task, App_IMUTask, osPriorityNormal, 0, 1024);
  IMU_TaskHandle = osThreadCreate(osThread(IMU_Task), NULL);

  /* definition and creation of StateMachine_Task */
  osThreadDef(StateMachine_Task, App_StateMachineTask, osPriorityHigh, 0, 1024);
  StateMachine_TaskHandle = osThreadCreate(osThread(StateMachine_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_App_Init */
/**
  * @brief  Function implementing the Init_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_App_Init */
__weak void App_Init(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
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
