/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern osThreadId Init_TASKHandle;
extern osThreadId Ins_TASKHandle;
extern osThreadId Client_TASKHandle;
extern osThreadId WatchDog_TASKHandle;
extern osThreadId Remote_TASKHandle;
extern osThreadId Comm_TASKHandle;
extern osThreadId Shell_TASKHandle;
extern osThreadId Quadruped_TASKHandle;
extern osThreadId WheelLeg_TASKHandle;
extern osThreadId Log_TASKHandle;
extern osThreadId Gimbal_TASKHandle;
extern osMessageQId Key_QueueHandle;
extern osMessageQId Key_TriggerQueueHandle;
extern osTimerId Beeper_TimerHandle;
extern osSemaphoreId Shell_RxCMDHandle;

#define forever while(1)
#define ever ;;
#define Gravity 9.81f

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum {
    Main_NO_CONNECTED  = 0,
    Main_WHEEL         = 1,
    Main_QUADRUPED     = 2,
    Main_BASIC         = 3,
    Main_Head          = 4,    
} Main_FuncTypeEnum;

extern Main_FuncTypeEnum Main_Type;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Green_Pin GPIO_PIN_13
#define LED_Green_GPIO_Port GPIOC
#define BigKey1_Pin GPIO_PIN_0
#define BigKey1_GPIO_Port GPIOC
#define BigKey1_EXTI_IRQn EXTI0_IRQn
#define BigKey2_Pin GPIO_PIN_2
#define BigKey2_GPIO_Port GPIOC
#define BigKey2_EXTI_IRQn EXTI2_IRQn
#define CSAccel_Pin GPIO_PIN_4
#define CSAccel_GPIO_Port GPIOA
#define BMI088_SCK_Pin GPIO_PIN_5
#define BMI088_SCK_GPIO_Port GPIOA
#define BMI088_MISO_Pin GPIO_PIN_6
#define BMI088_MISO_GPIO_Port GPIOA
#define BMI088_MOSI_Pin GPIO_PIN_7
#define BMI088_MOSI_GPIO_Port GPIOA
#define BMI088_INT1_Pin GPIO_PIN_4
#define BMI088_INT1_GPIO_Port GPIOC
#define BMI088_INT1_EXTI_IRQn EXTI4_IRQn
#define BMI088_INT2_Pin GPIO_PIN_5
#define BMI088_INT2_GPIO_Port GPIOC
#define BMI088_INT2_EXTI_IRQn EXTI9_5_IRQn
#define CSGyro_Pin GPIO_PIN_0
#define CSGyro_GPIO_Port GPIOB
#define IST8310_RST_Pin GPIO_PIN_1
#define IST8310_RST_GPIO_Port GPIOB
#define IST8301_SCL_Pin GPIO_PIN_10
#define IST8301_SCL_GPIO_Port GPIOB
#define IST8301_SDA_Pin GPIO_PIN_11
#define IST8301_SDA_GPIO_Port GPIOB
#define Fun1_Pin GPIO_PIN_14
#define Fun1_GPIO_Port GPIOB
#define Fun1_EXTI_IRQn EXTI15_10_IRQn
#define Fun2_Pin GPIO_PIN_15
#define Fun2_GPIO_Port GPIOB
#define Fun2_EXTI_IRQn EXTI15_10_IRQn
#define Dbus_Pin GPIO_PIN_9
#define Dbus_GPIO_Port GPIOA
#define DbusA10_Pin GPIO_PIN_10
#define DbusA10_GPIO_Port GPIOA
#define LED_Red_Pin GPIO_PIN_15
#define LED_Red_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
