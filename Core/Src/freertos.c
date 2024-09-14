/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "sys_dwt.h"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Init_TASKHandle;
osThreadId Ins_TASKHandle;
osThreadId Client_TASKHandle;
osThreadId WatchDog_TASKHandle;
osThreadId Remote_TASKHandle;
osThreadId Comm_TASKHandle;
osThreadId Shell_TASKHandle;
osThreadId Quadruped_TASKHandle;
osThreadId WheelLeg_TASKHandle;
osThreadId Gimbal_TASKHandle;
osMessageQId Key_QueueHandle;
osMessageQId Key_TriggerQueueHandle;
osTimerId Beeper_TimerHandle;
osSemaphoreId Shell_RxCMDHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Init_Task(void const * argument);
void Ins_Task(void const * argument);
void Client_Task(void const * argument);
void WatchDog_Task(void const * argument);
void Remote_Task(void const * argument);
void Comm_Task(void const * argument);
void Shell_Task(void const * argument);
void Quadruped_Task(void const * argument);
void WheelLeg_Task(void const * argument);
void Gimbal_Task(void const * argument);
void BeeperTimerCallback(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);
void vApplicationDaemonTaskStartupHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
    return DWT_GetTimeline_us();
	//return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
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

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN DAEMON_TASK_STARTUP_HOOK */
void vApplicationDaemonTaskStartupHook(void)
{
}
/* USER CODE END DAEMON_TASK_STARTUP_HOOK */

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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Shell_RxCMD */
  osSemaphoreDef(Shell_RxCMD);
  Shell_RxCMDHandle = osSemaphoreCreate(osSemaphore(Shell_RxCMD), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of Beeper_Timer */
  osTimerDef(Beeper_Timer, BeeperTimerCallback);
  Beeper_TimerHandle = osTimerCreate(osTimer(Beeper_Timer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Key_Queue */
  osMessageQDef(Key_Queue, 16, uint16_t);
  Key_QueueHandle = osMessageCreate(osMessageQ(Key_Queue), NULL);

  /* definition and creation of Key_TriggerQueue */
  osMessageQDef(Key_TriggerQueue, 16, uint16_t);
  Key_TriggerQueueHandle = osMessageCreate(osMessageQ(Key_TriggerQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Init_TASK */
  osThreadDef(Init_TASK, Init_Task, osPriorityRealtime, 0, 256);
  Init_TASKHandle = osThreadCreate(osThread(Init_TASK), NULL);

  /* definition and creation of Ins_TASK */
  osThreadDef(Ins_TASK, Ins_Task, osPriorityRealtime, 0, 256);
  Ins_TASKHandle = osThreadCreate(osThread(Ins_TASK), NULL);

  /* definition and creation of Client_TASK */
  osThreadDef(Client_TASK, Client_Task, osPriorityRealtime, 0, 256);
  Client_TASKHandle = osThreadCreate(osThread(Client_TASK), NULL);

  /* definition and creation of WatchDog_TASK */
  osThreadDef(WatchDog_TASK, WatchDog_Task, osPriorityRealtime, 0, 256);
  WatchDog_TASKHandle = osThreadCreate(osThread(WatchDog_TASK), NULL);

  /* definition and creation of Remote_TASK */
  osThreadDef(Remote_TASK, Remote_Task, osPriorityRealtime, 0, 256);
  Remote_TASKHandle = osThreadCreate(osThread(Remote_TASK), NULL);

  /* definition and creation of Comm_TASK */
  osThreadDef(Comm_TASK, Comm_Task, osPriorityRealtime, 0, 256);
  Comm_TASKHandle = osThreadCreate(osThread(Comm_TASK), NULL);

  /* definition and creation of Shell_TASK */
  osThreadDef(Shell_TASK, Shell_Task, osPriorityHigh, 0, 256);
  Shell_TASKHandle = osThreadCreate(osThread(Shell_TASK), NULL);

  /* definition and creation of Quadruped_TASK */
  osThreadDef(Quadruped_TASK, Quadruped_Task, osPriorityRealtime, 0, 256);
  Quadruped_TASKHandle = osThreadCreate(osThread(Quadruped_TASK), NULL);

  /* definition and creation of WheelLeg_TASK */
  osThreadDef(WheelLeg_TASK, WheelLeg_Task, osPriorityRealtime, 0, 256);
  WheelLeg_TASKHandle = osThreadCreate(osThread(WheelLeg_TASK), NULL);

  /* definition and creation of Gimbal_TASK */
  osThreadDef(Gimbal_TASK, Gimbal_Task, osPriorityRealtime, 0, 256);
  Gimbal_TASKHandle = osThreadCreate(osThread(Gimbal_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Init_Task */
/**
  * @brief  Function implementing the Init_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Init_Task */
__weak void Init_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the Ins_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
__weak void Ins_Task(void const * argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}

/* USER CODE BEGIN Header_Client_Task */
/**
* @brief Function implementing the Client_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Client_Task */
__weak void Client_Task(void const * argument)
{
  /* USER CODE BEGIN Client_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Client_Task */
}

/* USER CODE BEGIN Header_WatchDog_Task */
/**
* @brief Function implementing the WatchDog_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WatchDog_Task */
__weak void WatchDog_Task(void const * argument)
{
  /* USER CODE BEGIN WatchDog_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WatchDog_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the Remote_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void const * argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Comm_Task */
/**
* @brief Function implementing the Comm_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Comm_Task */
__weak void Comm_Task(void const * argument)
{
  /* USER CODE BEGIN Comm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Comm_Task */
}

/* USER CODE BEGIN Header_Shell_Task */
/**
* @brief Function implementing the Shell_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shell_Task */
__weak void Shell_Task(void const * argument)
{
  /* USER CODE BEGIN Shell_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shell_Task */
}

/* USER CODE BEGIN Header_Quadruped_Task */
/**
* @brief Function implementing the Quadruped_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Quadruped_Task */
__weak void Quadruped_Task(void const * argument)
{
  /* USER CODE BEGIN Quadruped_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Quadruped_Task */
}

/* USER CODE BEGIN Header_WheelLeg_Task */
/**
* @brief Function implementing the WheelLeg_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WheelLeg_Task */
__weak void WheelLeg_Task(void const * argument)
{
  /* USER CODE BEGIN WheelLeg_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WheelLeg_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* BeeperTimerCallback function */
__weak void BeeperTimerCallback(void const * argument)
{
  /* USER CODE BEGIN BeeperTimerCallback */

  /* USER CODE END BeeperTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
