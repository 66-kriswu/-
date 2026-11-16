/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

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

/* Exported functions prototypes ---------------------------------------------*/
/* [学习标注] 学习标注：以下为各中断向量的声明；每个函数的实现与作用见 stm32f4xx_it.c 同名函数上方的注释。 */
void EXTI0_IRQHandler(void);
/* [学习标注] EXTI1 中断入口（Y_MIN / PC1）。 */
void EXTI1_IRQHandler(void);
/* [学习标注] EXTI2 中断入口（Z_MIN / PC2）。 */
void EXTI2_IRQHandler(void);
/* [学习标注] EXTI3 中断入口（Z_MAX / PC3）。 */
void EXTI3_IRQHandler(void);
/* [学习标注] EXTI4 中断入口（X_MAX / PC4）。 */
void EXTI4_IRQHandler(void);
/* [学习标注] DMA1_S1 中断入口（USART3_RX）。 */
void DMA1_Stream1_IRQHandler(void);
/* [学习标注] DMA1_S2 中断入口（TIM3，旧脉冲通路）。 */
void DMA1_Stream2_IRQHandler(void);
/* [学习标注] DMA1_S5 中断入口（USART2_RX）。 */
void DMA1_Stream5_IRQHandler(void);
/* [学习标注] EXTI9_5 中断入口（急停 PE6）。 */
void EXTI9_5_IRQHandler(void);
/* [学习标注] TIM2 中断入口（旧 PWM 引擎）。 */
void TIM2_IRQHandler(void);
/* [学习标注] TIM3 中断入口（旧 PWM 引擎）。 */
void TIM3_IRQHandler(void);
/* [学习标注] USART2 中断入口（G-code + IDLE 判帧）。 */
void USART2_IRQHandler(void);
/* [学习标注] USART3 中断入口（调试 + IDLE 判帧）。 */
void USART3_IRQHandler(void);
/* [学习标注] DMA1_S7 中断入口（TIM2，旧脉冲通路）。 */
void DMA1_Stream7_IRQHandler(void);
/* [学习标注] TIM6 中断入口（HAL 1 ms 时基）。 */
void TIM6_DAC_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
