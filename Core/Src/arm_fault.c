/**
  ******************************************************************************
  * @file    arm_fault.c
  * @brief   Crash capture.  See arm_fault.h for the rationale.
  *
  *          Memory map of the block at 0x2001F000 (survives reset, because only
  *          .bss is zeroed by the startup code and .bss ends at 0x200097B0):
  *
  *            [0]  stage  0xB0070000 | last stage that was reached
  *            [1]  fault  0xDEADFA17 when HardFault_Handler ran
  *            [2]  boot counter
  *            [3]  CFSR
  *            [4]  HFSR
  *            [5]  EXC_RETURN (LR at fault entry)
  *            [6]  MSP
  *            [7]  PSP
  *            [8]  R0    [9] R1    [10] R2    [11] R3
  *            [12] R12   [13] LR   [14] PC    [15] xPSR
  *            [16] MMFAR
  *            [17] BFAR
  *            [18] ICSR
  ******************************************************************************
  */

#include "arm_fault.h"

#include "main.h"

#define ARM_FAULT_BASE      (0x2001F000u)
#define ARM_STAGE_MAGIC     (0xB0070000u)
#define ARM_STAGE_MASK      (0xFFFF0000u)
#define ARM_FAULT_MAGIC     (0xDEADFA17u)

static volatile uint32_t *const s_block = (volatile uint32_t *)ARM_FAULT_BASE;

/**
  * @brief  [学习标注] 返回 0x2001F000 诊断块地址（调试器/上位打印用）。
  */
volatile uint32_t *arm_fault_block(void)
{
  return s_block;
}

/**
  * @brief  [学习标注] 开机第一件事：校验魔数、启动计数 +1、清故障标志。
  */
void arm_fault_boot(void)
{
  /* Only trust the previous stage if it was written by this same firmware. */
  if ((s_block[0] & ARM_STAGE_MASK) != ARM_STAGE_MAGIC)
  {
    s_block[0] = ARM_STAGE_MAGIC;
    s_block[2] = 0u;
  }

  s_block[2]++;
  s_block[1] = 0u;
  s_block[0] = ARM_STAGE_MAGIC;
}

/**
  * @brief  [学习标注] 记录“走到哪一步”（ARM_STAGE_*）；死机后看 stage 就知道死在哪一步。
  */
void arm_fault_mark(uint32_t stage)
{
  s_block[0] = ARM_STAGE_MAGIC | (stage & 0xFFFFu);
}

/**
  * @brief  [学习标注] 栈溢出钩子：把任务名前 4 字节写进块尾并停机（不再静默踩内存）。
  */
void arm_fault_note_overflow(const char *task_name)
{
  uint32_t packed = 0u;

  /* Task names are at most configMAX_TASK_NAME_LEN (16) characters, so the
     first four are enough to identify the culprit without needing a UART. */
  if (task_name != NULL)
  {
    for (uint32_t i = 0u; i < 4u; i++)
    {
      if (task_name[i] == '\0')
      {
        break;
      }

      packed |= ((uint32_t)(uint8_t)task_name[i]) << (8u * i);
    }
  }

  s_block[19] = packed;                       /* "defa", "Comm", "LED\0" ... */
  s_block[3]  = 0x53544143u;                  /* 'STAC' - stack overflow     */
  s_block[0]  = ARM_STAGE_MAGIC | 0xE000u;    /* distinctive stage           */
  s_block[1]  = ARM_FAULT_MAGIC;

  for (;;)
  {
    /* Halt: memory is already corrupted, carrying on makes it worse. */
  }
}

/**
  * @brief  Records the fault and halts.
  * @note   Deliberately a plain C function: at this point the main stack is
  *         still usable, which is not true inside a handler that has to run on
  *         a corrupted task stack.
  */
void HardFault_Handler(void)
{
  uint32_t exc_lr;
  uint32_t msp;
  uint32_t psp;

  __asm volatile ("mov %0, lr"  : "=r" (exc_lr));
  __asm volatile ("mrs %0, msp" : "=r" (msp));
  __asm volatile ("mrs %0, psp" : "=r" (psp));

  s_block[1]  = ARM_FAULT_MAGIC;
  s_block[3]  = SCB->CFSR;
  s_block[4]  = SCB->HFSR;
  s_block[5]  = exc_lr;
  s_block[6]  = msp;
  s_block[7]  = psp;
  s_block[16] = SCB->MMFAR;
  s_block[17] = SCB->BFAR;
  s_block[18] = SCB->ICSR;

  /* Frame is on PSP when EXC_RETURN bit 2 is set, on MSP otherwise. */
  {
    const uint32_t *frame = (const uint32_t *)(((exc_lr & 4u) != 0u) ? psp : msp);

    for (uint32_t i = 0u; i < 8u; i++)
    {
      s_block[8u + i] = frame[i];
    }
  }

  for (;;)
  {
    /* Stay here: the diagnostic block holds everything needed. */
  }
}
