/**
  ******************************************************************************
  * @file    arm_command.h
  * @brief   G-code parsing and execution.
  ******************************************************************************
  */

#ifndef ARM_COMMAND_H
#define ARM_COMMAND_H

#include <stdbool.h>

#include "arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Parse one received line and hand it to CommandTask.
  * @param  line  Null terminated line without the terminating CR.
  * @retval true when the line was accepted, in which case the caller still has
  *         to send "ok" - exactly like robotArm.ino:93-97, where "ok" is sent
  *         at enqueue time and not when the move finally completes.
  * @note   Called from arm_uart_poll(), i.e. in task context.
  */
bool arm_command_handle_line(const char *line);

/**
  * @brief  Execute one already queued command.
  * @param  cmd  Command popped from gcode_q by MotionTask.
  * @retval Answer the caller has to transmit, with a warning text.
  */
arm_ack_t arm_command_execute(const arm_cmd_t *cmd, const char **warn_out);

/** Feed the parser self test, used once at start up. */
bool arm_command_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_COMMAND_H */
