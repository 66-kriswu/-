/**
  ******************************************************************************
  * @file    arm_board.h
  * @brief   Run time correction of the differences between the generated
  *          CubeMX project and the frozen baseline in handover doc section 4.
  *          See arm_board.c for the full rationale of each deviation.
  ******************************************************************************
  */

#ifndef ARM_BOARD_H
#define ARM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/** Correct every known deviation and install the TIM5 microsecond base.
  * Call once from main(), after MX_GPIO_Init()/MX_DMA_Init()/MX_TIMx_Init()
  * and before osKernelInitialize().                                          */
void arm_board_init(void);

/** 1. Drive X/Y/Z/Q ENABLE high so the drivers are OFF at power up.          */
void arm_board_drivers_disable(void);

/** 2/3. Reinstall the six EXTI lines: limits on both edges, E-stop falling.  */
void arm_board_exti_fixup(void);

/** 4. Restore the section 4.6 priority ladder for EXTI and USART2.           */
void arm_board_nvic_fixup(void);

/** Emit the boot report on the debug UART.                                   */
void arm_board_boot_report(void);

/* Provided by arm_uart.c. */
void arm_uart_boot_report(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_BOARD_H */
