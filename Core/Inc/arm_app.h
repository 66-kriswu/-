/**
  ******************************************************************************
  * @file    arm_app.h
  * @brief   Application entry point: task creation and the command queue.
  *
  *          Handover doc section 4.10 lists this file as app_freertos.c and
  *          describes it as an override of the CubeMX generated freertos.c.
  *          It is kept under a separate name instead, so that re-running
  *          GENERATE CODE can never overwrite the task set, and so that the
  *          generated StartDefaultTask() stays reachable for debugging.
  ******************************************************************************
  */

#ifndef ARM_APP_H
#define ARM_APP_H

#include <stdbool.h>

#include "arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Create the queue, the synchronisation objects and the four worker tasks.
  * Call after osKernelInitialize(), before osKernelStart().
  * @note  defaultTask is NOT created here: main() creates it through the
  *        CubeMX generated call, and its body calls arm_app_run_default_task()
  *        from inside the StartDefaultTask USER CODE block.                  */
void arm_app_init(void);

/**
  * @brief  Body of the CubeMX created defaultTask.  Never returns.
  * @note   Called from StartDefaultTask() in main.c, inside
  *         "USER CODE BEGIN 5" so that GENERATE CODE preserves it.
  *         Also lowers the task priority to ARM_PRIO_DEFAULT at run time,
  *         because the .ioc carries osPriorityNormal.
  */
void arm_app_run_default_task(void);

/** Offer one parsed command to the queue.  Returns false when it is full.
  * Called by arm_command_handle_line() in CommandTask context.               */
bool arm_app_post_command(const arm_cmd_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* ARM_APP_H */
