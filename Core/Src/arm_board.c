/**
  ******************************************************************************
  * @file    arm_board.c
  * @brief   Run time correction of the four deviations between the generated
  *          CubeMX project and the frozen configuration baseline (handover doc
  *          section 4).  The .ioc file is deliberately NOT modified, so this
  *          module brings the hardware up to the baseline at boot.
  *
  *          Deviation 1 - stepper ENABLE pins power up ENABLED
  *            The .ioc uses the key "PC10.GPIO_OutputLevel", which does not
  *            exist in the CubeMX database (the real key is "PinState"), so
  *            the value was dropped and CubeMX emitted a single
  *            HAL_GPIO_WritePin(..., GPIO_PIN_RESET) for the whole group.
  *            EN is active low, so the drivers were energised at power up,
  *            the opposite of section 4.2.1.  Corrected here.
  *
  *          Deviation 2 - limit switch EXTI edges are rising only
  *            No "GPIO_ModeDefaultEXTI" key exists in the .ioc, so CubeMX used
  *            its default GPIO_MODE_IT_RISING.  Section 4.2.4 requires BOTH
  *            edges.  Corrected here.
  *
  *          Deviation 3 - emergency stop EXTI edge is rising
  *            Same root cause.  Section 4.2.4 requires FALLING.  Corrected.
  *
  *          Deviation 4 - EXTI and USART2 NVIC priorities are 5 instead of 7/6
  *            Section 4.6 requires 5 = pulse, 6 = G-code, 7 = limit / debug.
  *            Corrected here.
  *
  *          Plus: TIM5 (section 4.3) is missing from the .ioc entirely and is
  *          installed by arm_us_init().
  ******************************************************************************
  */

#include "arm_board.h"

#include "main.h"
#include "arm_us.h"
#include "arm_config.h"
/*============================================================================*/
/* Deviation 1: force the stepper drivers OFF at power up                     */
/*============================================================================*/

void arm_board_drivers_disable(void)
{
  /* The level that turns a TB6600 OFF follows from ARM_ENABLE_ACTIVE_LOW, the
     same single switch arm_stepper_axis_enable() uses, so the power-up state
     and M17/M18 can never disagree.  The drivers must come up disabled so the
     motors cannot be driven by noise before the first M17, matching
     setStepperEnable(false) in robotArm.ino:66. */
#if (ARM_ENABLE_ACTIVE_LOW == 1)
  const GPIO_PinState off = GPIO_PIN_SET;
#else
  const GPIO_PinState off = GPIO_PIN_RESET;
#endif

  HAL_GPIO_WritePin(GPIOC, X_ENABLE_Pin | Y_ENABLE_Pin | Z_ENABLE_Pin, off);
  HAL_GPIO_WritePin(GPIOD, Q_ENABLE_Pin, off);
}

/*============================================================================*/
/* Deviations 2 and 3: rebuild the six EXTI lines with the required edges      */
/*============================================================================*/

void arm_board_exti_fixup(void)
{
  GPIO_InitTypeDef init = {0};

  /* Re-initialising an EXTI pin is safe: HAL_GPIO_Init() de-initialises the
     pin first, which clears its EXTI configuration and unmasks the line
     again, then re-applies the requested trigger.  Pin, pull and label are
     unchanged from the generated MX_GPIO_Init(), only the edge differs. */

  /* Limit switches PC0..PC4, EXTI0..EXTI4, both edges (section 4.2.4).
     Each pin owns a private EXTI vector, so no shared ISR has to
     disambiguate by reading EXTI->PR. */
  init.Pin   = X_MIN_Pin | Y_MIN_Pin | Z_MIN_Pin | Z_MAX_Pin | X_MAX_Pin;
  init.Mode  = GPIO_MODE_IT_RISING_FALLING;
  init.Pull  = GPIO_PULLUP;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &init);

  /* Emergency stop PE6, EXTI9_5, falling edge only: the button shorts the
     pin to GND, so pressing it always produces a falling edge. */
  init.Pin   = ESTOP_Pin;
  init.Mode  = GPIO_MODE_IT_FALLING;
  init.Pull  = GPIO_PULLUP;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESTOP_GPIO_Port, &init);
}

/*============================================================================*/
/* Deviation 4: restore the priority ladder of section 4.6                    */
/*============================================================================*/

void arm_board_nvic_fixup(void)
{
  /* Re-programming a priority before osKernelStart() is safe: no interrupt
     can observe the intermediate state because the scheduler is not running
     and every one of these lines is still masked at the device level. */

  HAL_NVIC_SetPriority(EXTI0_IRQn, 7u, 0u);   /* X_MIN  PC0 */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 7u, 0u);   /* Y_MIN  PC1 */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 7u, 0u);   /* Z_MIN  PC2 */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 7u, 0u);   /* Z_MAX  PC3 */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 7u, 0u);   /* X_MAX  PC4 */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7u, 0u); /* ESTOP  PE6 */

  HAL_NVIC_SetPriority(USART2_IRQn, 6u, 0u);  /* G-code IDLE framing */

  /* Untouched on purpose, they already match section 4.6:
       TIM2 / TIM3            = 5
       DMA1_Stream2 / Stream7 = 5
       DMA1_Stream5           = 6
       USART3 / DMA1_Stream1  = 7
       TIM6_DAC / SysTick     = 15

     TIM7, the timer of the software pulse generator (arm_stepper.c, section 23
     of the handover document), is given priority 5 by arm_stepper_init(), the
     same level the pulse DMA streams used to have.  Stream2 and Stream7 are
     switched off there as well, because the PWM/DMA pulse path is retired. */
}

/*============================================================================*/
/* Aggregate entry point                                                      */
/*============================================================================*/

/**
  * @brief  Reconfigure the three TB6600 ENA pins as OPEN DRAIN.
  *
  * The harness is common anode: ENA+ sits on +5 V and ENA- is driven by the MCU.
  * The STM32 only swings to 3.3 V, so even when the firmware writes a logic 1
  * there is still 5 - 3.0 = 2.0 V across the opto LED, which is more than its
  * forward voltage.  The opto therefore conducts and the driver stays DISABLED
  * whether the firmware writes 0 or 1.  Measured on the real machine:
  *
  *     PC12 at 0.3 V  -> shaft loose   (driver disabled)
  *     PC12 at 3.0 V  -> shaft loose   (driver disabled)   <-- M17 did nothing
  *     PC12 floating  -> shaft STIFF   (driver energised)
  *
  * Open drain removes the problem without touching the wiring: writing 1
  * RELEASES the pin, no current flows through the opto and the driver
  * energises; writing 0 sinks the pin and disables it.  PC10..PC12 are 5 V
  * tolerant on the F407, so the +5 V on ENA+ cannot damage them.
  *
  * This lives here, next to the other runtime corrections, so that GENERATE
  * CODE cannot undo it.
  */
static void arm_board_ena_open_drain(void)
{
  GPIO_InitTypeDef init = {0};

  /* Only ENA.  STEP and DIR are left exactly as CubeMX generates them
     (AF push-pull / GPIO push-pull): the reason the motor never stepped was a
     missing PUL- wire, not the drive type, so nothing here should change them.
     Verified on the machine: PC12 read 0.3 V and 3.0 V with the shaft loose in
     both cases, and 4-5 V once released - that is the ENA opto threshold. */
  init.Pin   = X_ENABLE_Pin | Y_ENABLE_Pin | Z_ENABLE_Pin;
  init.Mode  = GPIO_MODE_OUTPUT_OD;
  init.Pull  = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &init);
}

/**
  * @brief  Reconfigure the three STEP and three DIR pins as OPEN DRAIN.
  *
  * This is the same electrical trap as the ENA pins, but on the pulse path, and
  * it only became visible once the PUL- wire was actually connected:
  *
  *     PUL+ sits on +5 V (common anode).  A push-pull STM32 output drives the
  *     "high" level to 3.3 V, so 5.0 - 3.3 = 1.7 V is left across the opto LED
  *     plus its series resistor.  1.7 V is above the LED forward voltage, so a
  *     residual current (about 1.8 mA with a 270 ohm board resistor) keeps the
  *     input asserted.  The driver then never sees a clean rising edge and the
  *     motor holds torque without ever stepping - the "buzzes but does not
  *     turn" failure that was chased for two sessions.
  *
  * Measured proof on the real machine (2026-11-04): with PA0 disconnected and
  * PUL- pulsed to GND by hand (+ release = 5 V floating) the shaft steps
  * audibly, so driver, motor and coil pairs are all good; only the 3.3 V "high"
  * of the MCU is unusable in this common anode wiring.
  *
  * Open drain turns a logic 1 into a true release: no current through the opto,
  * so the input is completely off.  The released level (~4.4 V) is safe here -
  * the driver's series resistor limits the current to a few mA and the F407 pin
  * clamps at VDD + 0.7 V, far below the injection limit.
  *
  * STEP and DIR are both plain open drain GPIO outputs now.  Until 2026-11-05
  * the STEP pins were AF open drain driven by TIM2_CH1 / TIM3_CH1 / TIM3_CH2;
  * the pulse generator in arm_stepper.c produces the pulses from a TIM7
  * interrupt instead (see the explanation at the top of that file), so the
  * timers no longer own the pins and the level is written by software.  The
  * electrical requirement is unchanged: a logic 1 must be a true release.
  */
static void arm_board_step_dir_open_drain(void)
{
  GPIO_InitTypeDef init = {0};

  /* STEP: PA0 (base rotation), PA7 (lower arm), PA6 (upper arm) */
  init.Pin       = GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_7;
  init.Mode      = GPIO_MODE_OUTPUT_OD;
  init.Pull      = GPIO_NOPULL;
  init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  init.Alternate = 0u;
  HAL_GPIO_Init(GPIOA, &init);

  /* Idle level straight away, so the pins are never undefined between here and
     the first move (arm_stepper_init() writes the same level again). */
#if (ARM_DRIVER_COMMON_ANODE == 1)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
#endif

  /* DIR: PE2 (rotation), PB1 (lower arm), PD7 (upper arm) */
  init.Mode      = GPIO_MODE_OUTPUT_OD;
  init.Alternate = 0u;
  init.Speed     = GPIO_SPEED_FREQ_LOW;

  init.Pin = Z_DIR_Pin;
  HAL_GPIO_Init(Z_DIR_GPIO_Port, &init);
  init.Pin = Y_DIR_Pin;
  HAL_GPIO_Init(Y_DIR_GPIO_Port, &init);
  init.Pin = X_DIR_Pin;
  HAL_GPIO_Init(X_DIR_GPIO_Port, &init);

  /* Released = opto off.  The real direction is written again on every move;
     this only removes an undefined level between power up and the first move.
     arm_stepper_set_dir() keeps the "SET means forward" convention, which now
     reads as "SET means release". */
  HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
}

/**
  * @brief  [学习标注] 板级修正总入口：ENA/STEP/DIR 改开漏 → 上电失能 → EXTI 边沿 → NVIC 优先
  *         级 → 装 TIM5。
  * @note   顺序不能换：先改电气类型，再写电平（同一个 GPIO_PIN_SET 含义不同）。
  */
void arm_board_init(void)
{
  /* Must come before drivers_disable(): that call sets the power-up level, and
     the level only means "off" once the pins are open drain. */
  arm_board_ena_open_drain();

  /* Same reasoning for the pulse and direction lines, see above.  Runs after
     MX_TIM2_Init()/MX_TIM3_Init() so the generated AF push-pull setup is
     replaced, not replaced by it. */
  arm_board_step_dir_open_drain();

  arm_board_drivers_disable();
  arm_board_exti_fixup();
  arm_board_nvic_fixup();
  arm_us_init();
}

/*============================================================================*/
/* Boot report, for the debug UART                                            */
/*============================================================================*/

/**
  * @brief  [学习标注] 上电横幅，转调 arm_uart_boot_report()。
  */
void arm_board_boot_report(void)
{
  /* Overridden by arm_debug_boot_report() in arm_uart.c when the debug link
     is enabled; kept here so the board module stays self contained. */
  arm_uart_boot_report();
}
