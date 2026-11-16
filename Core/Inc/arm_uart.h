/**
  ******************************************************************************
  * @file    arm_uart.h
  * @brief   Serial layer: G-code link on USART2 (9600 8N1) and debug log on
  *          USART3 (115200 8N1), both received by DMA in circular mode and
  *          framed by the USART IDLE line detection.
  *
  *          Framing rules reproduce the original firmware exactly:
  *            - '\r' terminates a line and triggers parsing
  *            - '\n' is ignored  (robotArm.ino:18-20)
  *            - every answer is terminated with CR LF, because the Delphi host
  *              splits its input on #13#10 (UnitRobotArm.pas:182)
  ******************************************************************************
  */

#ifndef ARM_UART_H
#define ARM_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "arm_config.h"
#include "arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Consumer of one complete command line (without the terminating CR).
  * @retval true when the line was handled and therefore needs no further
  *         answer, false when the caller must still answer it.
  * @note   Runs in task context, inside arm_uart_poll().                     */
typedef bool (*arm_line_cb_t)(const char *line);

/** Start DMA circular reception on both links and create the TX mutex.
  * Call after osKernelInitialize(), before osKernelStart().                  */
void arm_uart_init(void);

/** Register the consumer of complete command lines. Must be called before
  * the first arm_uart_poll().                                                */
void arm_uart_set_line_callback(arm_line_cb_t cb);

/** Drain the received bytes and hand complete lines to the command layer.
  * Call from a task context, it blocks in osMessageQueuePut() when needed.   */
void arm_uart_poll(void);

/** USART2 IDLE hook.  Call from the USER CODE section of USART2_IRQHandler.  */
void arm_uart_gcode_idle_isr(void);

/** USART3 IDLE hook.  Call from the USER CODE section of USART3_IRQHandler.  */
void arm_uart_debug_idle_isr(void);

/*------------------------------ G-code transmit ----------------------------*/

/** Send one answer line on USART2, appending CR LF.  Task context only.
  * Serialised by an internal mutex so several tasks may answer safely.       */
void arm_uart_send_line(const char *text);

/** Convenience wrappers reproducing the original printOk/printErr/... */
void arm_uart_print_ok(void);
/* [学习标注] 回 "rs"（解析失败，要求重发）。 */
void arm_uart_print_resend(void);
/* [学习标注] 回 "!!"（执行失败）。 */
void arm_uart_print_fault(void);
/* [学习标注] 回 "// 文本" 注释行。 */
void arm_uart_print_comment(const char *text);

/** Answer a command result.
  *
  *  Only the deferred part of the answer is produced here.  "ok" is NOT sent
  *  by this function, because the original firmware sends it once at enqueue
  *  time (printOk() in robotArm.ino:96) and only the error indication follows
  *  later, when the queued command is actually executed
  *  (handleAsErr() in robotArm.ino:154-157).
  *
  *  Order follows section 4.10 rule 3: the "// WARN" comment first, then "!!",
  *  never the other way round.                                                */
void arm_uart_send_deferred(arm_ack_t ack, const char *warn);

/*------------------------------- Debug link --------------------------------*/

/** Queue a line for USART3; never blocks.                                    */
void arm_uart_debug(const char *text);

/** printf-style helper for the debug link.                                   */
void arm_uart_debugf(const char *fmt, ...);

/** Emit the boot report, called by arm_board_boot_report().                  */
void arm_uart_boot_report(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_UART_H */
