/**
  ******************************************************************************
  * @file    arm_fault.h
  * @brief   Minimal crash visibility for the robot arm firmware.
  *
  *          WHY THIS EXISTS
  *          ---------------
  *          The CubeMX startup file aliases HardFault_Handler to Default_Handler,
  *          which is nothing but "b Infinite_Loop".  Any bus error, stack
  *          overflow or corrupted pointer therefore freezes the board in total
  *          silence: no LED, no serial, no clue.  On a bench that is
  *          indistinguishable from a dead board, which makes field diagnosis
  *          nearly impossible.
  *
  *          arm_fault_mark() writes a stage number into a small block of RAM
  *          that lives ABOVE .bss, so it is not cleared by the startup code and
  *          survives a reset.  HardFault_Handler() records the fault registers
  *          and the stacked PC/LR into the same block and then stops.
  *
  *          After a crash the block can be read over SWD (or printed on the
  *          next boot) and it says exactly how far the firmware got and what
  *          faulted.
  ******************************************************************************
  */

#ifndef ARM_FAULT_H
#define ARM_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Stage numbers written by arm_fault_mark().  Reading block[0] after a reset
   tells how far the last boot got. */
#define ARM_STAGE_BOARD_INIT        (1u)
#define ARM_STAGE_STEPPER_INIT      (2u)
#define ARM_STAGE_HOME_POSE         (3u)
#define ARM_STAGE_LIMITS_INIT       (4u)
#define ARM_STAGE_UART_INIT         (5u)
#define ARM_STAGE_APP_INIT          (6u)
#define ARM_STAGE_APP_INIT_DONE     (7u)
#define ARM_STAGE_MOTION_TASK       (8u)
#define ARM_STAGE_COMMAND_TASK      (9u)
#define ARM_STAGE_SAFETY_TASK       (10u)
#define ARM_STAGE_LED_TASK          (11u)
#define ARM_STAGE_DEFAULT_TASK      (12u)

/** Clear the record and count this boot.  Call once, first thing in main(). */
void arm_fault_boot(void);

/** Record how far the firmware has got.  Cheapest possible store. */
void arm_fault_mark(uint32_t stage);

/** Called from vApplicationStackOverflowHook().  Stores the first four
  * characters of the offending task name and halts, so that a stack overflow
  * stops being an invisible memory corruption. */
void arm_fault_note_overflow(const char *task_name);

/** Address of the diagnostic block, for a debugger or a print on next boot. */
volatile uint32_t *arm_fault_block(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_FAULT_H */
