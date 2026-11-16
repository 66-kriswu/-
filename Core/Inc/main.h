/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* [学习标注] 在 stm32f4xx_hal_msp.c 中实现：把定时器通道引脚配成复用功能。 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
/* [学习标注] 致命错误兜底：关中断 + PC13/PE0 10 Hz 双灯快闪（便于与“没电”区分）。 */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z_DIR_Pin GPIO_PIN_2
#define Z_DIR_GPIO_Port GPIOE
#define ESTOP_Pin GPIO_PIN_6
#define ESTOP_GPIO_Port GPIOE
#define ESTOP_EXTI_IRQn EXTI9_5_IRQn
#define LED_ONBOARD_Pin GPIO_PIN_13
#define LED_ONBOARD_GPIO_Port GPIOC
#define X_MIN_Pin GPIO_PIN_0
#define X_MIN_GPIO_Port GPIOC
#define X_MIN_EXTI_IRQn EXTI0_IRQn
#define Y_MIN_Pin GPIO_PIN_1
#define Y_MIN_GPIO_Port GPIOC
#define Y_MIN_EXTI_IRQn EXTI1_IRQn
#define Z_MIN_Pin GPIO_PIN_2
#define Z_MIN_GPIO_Port GPIOC
#define Z_MIN_EXTI_IRQn EXTI2_IRQn
#define Z_MAX_Pin GPIO_PIN_3
#define Z_MAX_GPIO_Port GPIOC
#define Z_MAX_EXTI_IRQn EXTI3_IRQn
#define X_MAX_Pin GPIO_PIN_4
#define X_MAX_GPIO_Port GPIOC
#define X_MAX_EXTI_IRQn EXTI4_IRQn
#define Y_DIR_Pin GPIO_PIN_1
#define Y_DIR_GPIO_Port GPIOB
#define Q_STEP_Pin GPIO_PIN_14
#define Q_STEP_GPIO_Port GPIOD
#define X_ENABLE_Pin GPIO_PIN_10
#define X_ENABLE_GPIO_Port GPIOC
#define Y_ENABLE_Pin GPIO_PIN_11
#define Y_ENABLE_GPIO_Port GPIOC
#define Z_ENABLE_Pin GPIO_PIN_12
#define Z_ENABLE_GPIO_Port GPIOC
#define Q_ENABLE_Pin GPIO_PIN_0
#define Q_ENABLE_GPIO_Port GPIOD
#define Q_DIR_Pin GPIO_PIN_1
#define Q_DIR_GPIO_Port GPIOD
#define PC_TX_Pin GPIO_PIN_5
#define PC_TX_GPIO_Port GPIOD
#define PC_RX_Pin GPIO_PIN_6
#define PC_RX_GPIO_Port GPIOD
#define X_DIR_Pin GPIO_PIN_7
#define X_DIR_GPIO_Port GPIOD
#define LED_HEARTBEAT_Pin GPIO_PIN_0
#define LED_HEARTBEAT_GPIO_Port GPIOE
#define LED_FAULT_Pin GPIO_PIN_1
#define LED_FAULT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
