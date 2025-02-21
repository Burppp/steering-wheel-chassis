/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "queue.h"
#include "can_hardwares.h"
#include "bsp_can.h"
#include "Cap.h."

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
osThreadId usb_task_handle;
osThreadId decode_task_handle;
osThreadId can_decode_task_handle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId CalibrateTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId detectTaskHandle;
osThreadId RefereeTaskHandle;
osThreadId left_gimbalTaskHandle;
osThreadId right_gimbalTasHandle;
osThreadId cap_task_handle;


osThreadId steering_wheelTaskHandle;
osThreadId travelling_wheelTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void calibrate_task(void const * argument);
void chassis_task(void const * argument);
void gimbal_task(void const * argument);
void INS_task(void const * argument);
void detect_task(void const * argument);
void Referee_send_task(void const * argument);
void left_gimbal(void const * argument);
void right_gimbal(void const * argument);

void TRA_ctrl(void const * argument);
void STR_ctrl(void const * argument);
void cap_task(void const *pvParameters);

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
QueueHandle_t CDC_send_queue;
extern void usb_task(void const * argument);
extern void decode_task(void const * arg);

_Noreturn extern void can_decode_task(void const * argument);
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
    CDC_send_queue = xQueueCreate(5, 256);

    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osThreadDef(CapTask,cap_task,osPriorityNormal,0, 128);
    cap_task_handle=osThreadCreate(osThread(CapTask),NULL);

    /* definition and creation of CalibrateTask */
    osThreadDef(CalibrateTask, calibrate_task, osPriorityNormal, 0, 512);
    CalibrateTaskHandle = osThreadCreate(osThread(CalibrateTask), NULL);

    /* definition and creation of ChassisTask */
//  osThreadDef(ChassisTask, chassis_task, osPriorityLow, 0, 512);
//  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

    /* definition and creation of gimbalTask */
    osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 512);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    /* definition and creation of imuTask */
    osThreadDef(imuTask, INS_task, osPriorityIdle, 0, 512);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

    /* definition and creation of detectTask */
    osThreadDef(detectTask, detect_task, osPriorityIdle, 0, 256);
    detectTaskHandle = osThreadCreate(osThread(detectTask), NULL);

    /* definition and creation of RefereeTask */
    osThreadDef(RefereeTask, Referee_send_task, osPriorityNormal, 0, 128);
    RefereeTaskHandle = osThreadCreate(osThread(RefereeTask), NULL);

    /* definition and creation of left_gimbalTask */
//  osThreadDef(left_gimbalTask, left_gimbal, osPriorityNormal, 0, 128);
//  left_gimbalTaskHandle = osThreadCreate(osThread(left_gimbalTask), NULL);

    /* definition and creation of right_gimbalTas */
//  osThreadDef(right_gimbalTas, right_gimbal, osPriorityNormal, 0, 128);
//  right_gimbalTasHandle = osThreadCreate(osThread(right_gimbalTas), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    osThreadDef(USBtask,usb_task,osPriorityHigh,0,128);
    usb_task_handle= osThreadCreate(osThread(USBtask),NULL);

    osThreadDef(DecodeTask, decode_task, osPriorityHigh, 0, 128);
    decode_task_handle = osThreadCreate(osThread(DecodeTask), NULL);

    osThreadDef(strTask, STR_ctrl, osPriorityRealtime, 0, 128);
    steering_wheelTaskHandle = osThreadCreate(osThread(strTask), NULL);

    osThreadDef(traTask, TRA_ctrl, osPriorityRealtime, 0, 128);
    travelling_wheelTaskHandle = osThreadCreate(osThread(traTask), NULL);
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
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_calibrate_task */
/**
* @brief Function implementing the CalibrateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_calibrate_task */
__weak void calibrate_task(void const * argument)
{
    /* USER CODE BEGIN calibrate_task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END calibrate_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void const * argument)
{
    /* USER CODE BEGIN chassis_task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
    /* USER CODE BEGIN gimbal_task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void const * argument)
{
    /* USER CODE BEGIN INS_task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END INS_task */
}

/* USER CODE BEGIN Header_detect_task */
/**
* @brief Function implementing the detectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_detect_task */
__weak void detect_task(void const * argument)
{
    /* USER CODE BEGIN detect_task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END detect_task */
}

/* USER CODE BEGIN Header_Referee_send_task */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_send_task */
__weak void Referee_send_task(void const * argument)
{
    /* USER CODE BEGIN Referee_send_task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END Referee_send_task */
}

/* USER CODE BEGIN Header_left_gimbal */
/**
* @brief Function implementing the left_gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_left_gimbal */
__weak void left_gimbal(void const * argument)
{
    /* USER CODE BEGIN left_gimbal */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END left_gimbal */
}

/* USER CODE BEGIN Header_right_gimbal */
/**
* @brief Function implementing the right_gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_right_gimbal */
__weak void right_gimbal(void const * argument)
{
    /* USER CODE BEGIN right_gimbal */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END right_gimbal */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */