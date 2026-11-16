/**
  ******************************************************************************
  * @file    arm_app.c
  * @brief   Task set for the robot arm, following handover doc section 4.10.
  *
  *          Task          Priority                 Period / trigger   Job
  *          ------------  -----------------------  -----------------  --------------------------
  *          MotionTask    osPriorityAboveNormal   1 kHz              interpolate -> geometry ->
  *                                                                   queue step pulses
  *          CommandTask   osPriorityNormal        blocks on gcode_q  parse lines, feed queue,
  *                                                                   answer "ok"
  *          SafetyTask    osPriorityBelowNormal    10 ms              limits, soft limits, E-stop
  *          LedTask       osPriorityLow            250 ms             heartbeat + alarm lamp
  *          defaultTask   osPriorityLow            1 s                diagnostics
  *
  *          Data flow, which mirrors the original loop() exactly:
  *
  *            USART2 DMA + IDLE
  *                -> arm_uart_poll()            (CommandTask)
  *                -> arm_command_handle_line()  parse, "ok" immediately
  *                -> gcode_q                    depth 15, sizeof(arm_cmd_t)
  *                -> MotionTask                 interpolate, drive the three axes
  *
  *          The host is stop-and-wait: it sends one command and waits for "ok"
  *          before sending the next (UnitRobotArm.pas:149-156).  Because "ok"
  *          is sent at enqueue time, the queue can never hold more than a
  *          couple of entries in practice, and MotionTask drains it fast enough
  *          that the 15 entry depth is never the limiting factor.
  ******************************************************************************
  */

#include "arm_app.h"

#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_config.h"
#include "arm_command.h"
#include "arm_geometry.h"
#include "arm_interpolation.h"
#include "arm_limits.h"
#include "arm_stepper.h"
#include "arm_uart.h"
#include "arm_us.h"
#include "arm_fault.h"

/*============================== Private types ===============================*/

typedef enum
{
  MOTION_IDLE = 0,
  MOTION_MOVING,
  MOTION_DWELL
} motion_state_t;

/** State of the M400 "park at home when the batch is over" request. */
typedef enum
{
  AUTO_HOME_OFF = 0,
  AUTO_HOME_ARMED,      /**< M400 seen, waiting for the command queue to drain  */
  AUTO_HOME_RUNNING     /**< the synthetic return move is in progress          */
} auto_home_state_t;

/*============================= Private state ===============================*/

static osMessageQueueId_t s_gcode_q = NULL;
static osEventFlagsId_t   s_arm_evt = NULL;

static osThreadId_t s_motion_task  = NULL;
static osThreadId_t s_command_task = NULL;
static osThreadId_t s_safety_task  = NULL;
static osThreadId_t s_led_task     = NULL;

static motion_state_t s_motion_state = MOTION_IDLE;
static uint32_t       s_dwell_until  = 0u;

/* M400 return-to-home request, advanced by MotionTask only. */
static auto_home_state_t s_auto_home = AUTO_HOME_OFF;

/* FreeRTOS task handles, needed by vTaskDelayUntil(). */
static TaskHandle_t s_motion_task_handle = NULL;

/* Diagnostics. */
static volatile uint32_t s_motion_cycles = 0u;
static volatile uint32_t s_shortfall     = 0u;

/*=========================== Internal prototypes ===========================*/

static void arm_motion_task(void *argument);
static void arm_command_task(void *argument);
static void arm_safety_task(void *argument);
static void arm_led_task(void *argument);
static void arm_motion_apply_joints(const arm_joints_t *joints);
static void arm_motion_start_command(const arm_cmd_t *cmd);
static void arm_motion_auto_home_start(void);

/*============================================================================*/
/* Construction                                                              */
/*============================================================================*/

/**
  * @brief  [学习标注] 创建 gcode_q(深度 15)、事件组和 4 个任务（defaultTask 由 main.c 创建），
  *         并挂串口行回调。
  */
void arm_app_init(void)
{
  arm_fault_mark(ARM_STAGE_APP_INIT);

  const osMessageQueueAttr_t q_attr   = {.name = "gcode_q"};
  const osEventFlagsAttr_t   e_attr   = {.name = "arm_evt"};

  const osThreadAttr_t motion_attr =
  {
    .name       = "MotionTask",
    .stack_size = ARM_STACK_MOTION * 4u,
    .priority   = (osPriority_t)ARM_PRIO_MOTION,
  };

  const osThreadAttr_t command_attr =
  {
    .name       = "CommandTask",
    .stack_size = ARM_STACK_COMMAND * 4u,
    .priority   = (osPriority_t)ARM_PRIO_COMMAND,
  };

  const osThreadAttr_t safety_attr =
  {
    .name       = "SafetyTask",
    .stack_size = ARM_STACK_SAFETY * 4u,
    .priority   = (osPriority_t)ARM_PRIO_SAFETY,
  };

  const osThreadAttr_t led_attr =
  {
    .name       = "LedTask",
    .stack_size = ARM_STACK_LED * 4u,
    .priority   = (osPriority_t)ARM_PRIO_LED,
  };

  /* Synchronisation objects of section 4.10. */
  s_gcode_q = osMessageQueueNew(ARM_CMD_QUEUE_DEPTH, sizeof(arm_cmd_t), &q_attr);
  s_arm_evt = osEventFlagsNew(&e_attr);

  if ((s_gcode_q == NULL) || (s_arm_evt == NULL))
  {
    Error_Handler();
  }

  arm_limits_set_event_group(s_arm_evt);
  arm_uart_set_line_callback(arm_command_handle_line);

  s_motion_task  = osThreadNew(arm_motion_task,  NULL, &motion_attr);
  s_command_task = osThreadNew(arm_command_task, NULL, &command_attr);
  s_safety_task  = osThreadNew(arm_safety_task,  NULL, &safety_attr);
  s_led_task     = osThreadNew(arm_led_task,     NULL, &led_attr);

  /* NOTE: the fifth task, defaultTask, is deliberately NOT created here.
     main() creates it through the CubeMX generated osThreadNew() call, and its
     body calls arm_app_run_default_task().  Creating it in both places used to
     produce two live tasks with the same name, because GENERATE CODE restores
     the generated creation call.  One owner only: the generated one. */

  if ((s_motion_task == NULL) || (s_command_task == NULL) ||
      (s_safety_task == NULL) || (s_led_task == NULL))
  {
    Error_Handler();
  }

  /* Keep a plain FreeRTOS handle as well, vTaskDelayUntil() needs it. */
  s_motion_task_handle = (TaskHandle_t)s_motion_task;
}

/*============================================================================*/
/* defaultTask body, called from StartDefaultTask() in main.c                  */
/*============================================================================*/

/**
  * @brief  Joint position in deci-degrees (0.1 deg) for the diagnostic line.
  * @note   Integer only: the project links nano.specs, so %f is not available
  *         in printf.  0.1 deg is a third of one micro-step at the joints
  *         (1811.44 steps/rad), which is fine for spotting a lost step or a
  *         model/real-world mismatch.
  */
static long arm_pose_decideg(arm_axis_t axis)
{
  return (long)(arm_stepper_get_position_rad(axis) * 572.9577951f);
}

/**
  * @brief  [学习标注] defaultTask 的任务体：把优先级降到 Low，然后每 200 ms 打一行 diag。
  * @note   由 main.c 的 StartDefaultTask() 在 USER CODE 5 里调用（只能有一个创建者）。
  */
void arm_app_run_default_task(void)
{
  arm_fault_mark(ARM_STAGE_DEFAULT_TASK);

  /* The CubeMX project creates this task with the priority stored in
     FREERTOS.Tasks01, which is osPriorityNormal (24).  Section 4.10 wants the
     diagnostics task LOWEST so that it can never delay a command or a move, so
     the priority is corrected at run time.  Doing it here rather than in the
     .ioc keeps it working whichever priority CubeMX happens to emit. */
  if (osThreadSetPriority(osThreadGetId(), (osPriority_t)ARM_PRIO_DEFAULT) != osOK)
  {
    /* Not fatal: the task simply keeps the priority it was created with. */
  }

  for (;;)
  {
    /* pos = rot/low/high in 0.1 deg.  Reading it back after a move is the
       quickest way to tell a lost step from a geometry effect.
       us  = programmed pulse period per axis in microseconds.  It has to follow
             the cosine ramp of the interpolation; a period pinned at its
             maximum (100000) means the generator is starving.
       tk  = 20 us ticks served by the pulse generator.  It runs from the first
             queued pulse until the segment is over, so "pend>0 and tk frozen"
             means the generator is dead while "pend>0 and tk rising" means it
             is emitting and merely slow.
       ph  = bit0 rot, bit1 low, bit2 high: the pulse currently asserted. */
    {
      uint32_t ph = 0u;
      uint32_t bit;

      arm_stepper_debug_axis(ARM_AXIS_ROT,  NULL, NULL, &bit); ph |= (bit & 1u);
      arm_stepper_debug_axis(ARM_AXIS_LOW,  NULL, NULL, &bit); ph |= (bit & 1u) << 1u;
      arm_stepper_debug_axis(ARM_AXIS_HIGH, NULL, NULL, &bit); ph |= (bit & 1u) << 2u;

      arm_uart_debugf("diag: cyc=%lu short=%lu pend=%lu/%lu/%lu sw=0x%02X tmul=%d pos=%ld/%ld/%ld us=%lu/%lu/%lu tk=%lu ph=%lu m=%d%d\r\n",
                      (unsigned long)s_motion_cycles,
                      (unsigned long)s_shortfall,
                      (unsigned long)arm_stepper_pending(ARM_AXIS_ROT),
                      (unsigned long)arm_stepper_pending(ARM_AXIS_LOW),
                      (unsigned long)arm_stepper_pending(ARM_AXIS_HIGH),
                      (unsigned)arm_limits_raw_state(),
                      (int)arm_interp_get_tmul(),
                      arm_pose_decideg(ARM_AXIS_ROT),
                      arm_pose_decideg(ARM_AXIS_LOW),
                      arm_pose_decideg(ARM_AXIS_HIGH),
                      (unsigned long)arm_stepper_pulse_us(ARM_AXIS_ROT),
                      (unsigned long)arm_stepper_pulse_us(ARM_AXIS_LOW),
                      (unsigned long)arm_stepper_pulse_us(ARM_AXIS_HIGH),
                      (unsigned long)arm_stepper_tick_count(),
                      (unsigned long)ph,
                      (int)s_motion_state,
                      (arm_interp_is_finished() ? 1 : 0));
    }

    /* Fixed cadence, see ARM_DIAG_PERIOD_MS: a state dependent delay used to
       skip whole segments when a print landed between two of them. */
    osDelay(ARM_DIAG_PERIOD_MS);
  }
}

/**
  * @brief  [学习标注] 非阻塞把命令投进 gcode_q；队列满返回 false（调用方仍回 ok，保持原版字节流
  *         ）。
  */
bool arm_app_post_command(const arm_cmd_t *cmd)
{
  if ((s_gcode_q == NULL) || (cmd == NULL))
  {
    return false;
  }

  /* Never block: the caller has to answer the host immediately. */
  return (osMessageQueuePut(s_gcode_q, cmd, 0u, 0u) == osOK);
}

/*============================================================================*/
/* MotionTask, 1 kHz                                                          */
/*============================================================================*/

/**
  * @brief  [学习标注] 1 kHz 运动主循环：空闲时取命令 → 插补 → 逆解 → 路径检查 → 三轴下发 →
  *         判完成。
  */
static void arm_motion_task(void *argument)
{
  arm_fault_mark(ARM_STAGE_MOTION_TASK);

  TickType_t last_wake = xTaskGetTickCount();
  arm_cmd_t  cmd;

  (void)argument;

  for (;;)
  {
    /* 1. Pick up at most one queued command.  Only accepted while idle, which
          is what makes the stop-and-wait protocol and the 15 deep queue
          behave like the original. */
    if (s_motion_state == MOTION_IDLE)
    {
      if (osMessageQueueGet(s_gcode_q, &cmd, NULL, 0u) == osOK)
      {
        arm_motion_start_command(&cmd);
      }
    }

    /* 2. Retire a finished dwell. */
    if (s_motion_state == MOTION_DWELL)
    {
      if ((int32_t)((uint32_t)xTaskGetTickCount() - s_dwell_until) >= 0)
      {
        s_motion_state = MOTION_IDLE;
      }
    }

    /* 3. Advance the interpolation and drive the axes. */
    if (s_motion_state == MOTION_MOVING)
    {
      if (arm_limits_is_faulted())
      {
        /* A limit switch or the E-stop fired in the middle of a segment.
           Freezing the segment here (the old behaviour) also froze command
           processing: MotionTask only dequeues when it is idle, so the M18 or
           M17 that clears the fault could never run, and the machine stayed
           dead until it was reset.  Abort the segment instead.  The
           interpolator keeps its progress, so the next move is planned from
           the pose where the arm actually stopped; the model is at most a few
           in-flight pulses away from the hardware and M210 re-zeroes it. */
        arm_interp_stall();
        arm_stepper_stop_all();
        s_motion_state = MOTION_IDLE;

        arm_uart_debugf("fault: segment aborted at XYZ %ld/%ld/%ld mm\r\n",
                        (long)arm_interp_get_x(),
                        (long)arm_interp_get_y(),
                        (long)arm_interp_get_z());

        if (s_auto_home == AUTO_HOME_RUNNING)
        {
          s_auto_home = AUTO_HOME_OFF;
          arm_uart_debug("auto home cancelled by fault\r\n");
        }
      }
      else
      {
        arm_joints_t joints;

        arm_interp_update();

        arm_geometry_solve(arm_interp_get_x(), arm_interp_get_y(), arm_interp_get_z(),
                           &joints);

        /* arm_command_execute() only validates the TARGET pose, but the path
           between two legal poses can still leave the workspace: a base
           rotation dips the lower arm to -17.5 deg halfway along the chord,
           and a straight line that passes near the base axis can fold the arm
           much further in.  The machine has a hard stop 20 deg behind the
           power-on pose (handover document section 23.10), so the interpolated
           pose is checked every cycle and the segment is abandoned rather than
           driven into it. */
        if (arm_limits_check_pose(joints.rot, joints.low, joints.high) != ARM_LIMIT_OK)
        {
          arm_stepper_stop_all();
          s_motion_state = MOTION_IDLE;

          if (s_auto_home == AUTO_HOME_RUNNING)
          {
            /* An automatic return must not inject an unsolicited comment and
               "!!" into the host's stop-and-wait link; the debug port is
               enough. */
            s_auto_home = AUTO_HOME_OFF;
            arm_uart_debug("auto home aborted: path leaves safe range\r\n");
          }
          else
          {
            arm_uart_print_comment("PATH LEAVES SAFE RANGE - move aborted");
            arm_uart_print_fault();
          }
        }
        else
        {
          arm_motion_apply_joints(&joints);
        }

        /* The segment only ends once every pulse has actually been handed to
           the engine, so the virtual position never runs ahead of the
           hardware. */
        if (arm_interp_is_finished() && arm_stepper_all_idle())
        {
          s_motion_state = MOTION_IDLE;

          if (s_auto_home == AUTO_HOME_RUNNING)
          {
            /* No M210 here on purpose.  Once the return has completed the
               model already sits at the power-on pose, so M210 would rewrite
               exactly the same numbers - it is a no-op, not a correction.  It
               would only do damage after a return that was aborted half way,
               where it would declare that half-way pose to be home.  The
               physical pose after the return is still only as good as the
               steps that were not lost; nothing on the firmware side can
               measure that. */
            s_auto_home = AUTO_HOME_OFF;
            arm_uart_debugf("auto home done: pos=%ld/%ld/%ld (0.1 deg)\r\n",
                            arm_pose_decideg(ARM_AXIS_ROT),
                            arm_pose_decideg(ARM_AXIS_LOW),
                            arm_pose_decideg(ARM_AXIS_HIGH));
          }

          if (s_arm_evt != NULL)
          {
            (void)osEventFlagsSet(s_arm_evt, ARM_EVT_MOVE_DONE);
          }
        }
      }
    }

    /* 4. M400: park at the power-on pose once everything queued has run.
          "Everything" is decided by the queue being empty, so commands that
          arrived after the M400 in the same batch run first.  The return then
          runs to completion; a command that arrives meanwhile simply waits in
          the queue, which is what a stop-and-wait host expects. */
    if ((s_auto_home == AUTO_HOME_ARMED) &&
        (s_motion_state == MOTION_IDLE) &&
        (s_gcode_q != NULL) &&
        (osMessageQueueGetCount(s_gcode_q) == 0u))
    {
      arm_motion_auto_home_start();
    }

    s_motion_cycles++;

    /* Fixed 1 kHz cadence, the same period the original loop() ran at when the
       steppers happened to be fast enough. */
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARM_MOTION_PERIOD_MS));
  }
}

/**
  * @brief  [学习标注] 命令分派：G4 进 dwell、M400 只登记、G0/G1 起插补、其余立即执行。
  */
static void arm_motion_start_command(const arm_cmd_t *cmd)
{
  const char *warn = NULL;
  arm_cmd_t   local = *cmd;

  /* G4 advances real time, everything else moves the arm. */
  if ((local.id == 'G') && (local.num == 4))
  {
    float seconds = local.valueT;

    if (seconds < 0.0f)
    {
      seconds = 0.0f;
    }

    if (seconds > 3600.0f)
    {
      seconds = 3600.0f;
    }

    s_dwell_until  = (uint32_t)xTaskGetTickCount() + (uint32_t)(seconds * 1000.0f);
    s_motion_state = MOTION_DWELL;
    arm_uart_send_deferred(ARM_ACK_OK, NULL);
    return;
  }

  /* M400 - "everything queued after this is the batch; park the arm when it is
     done".  Grbl and Marlin use the same number for "wait for all motions".
     Handled here rather than in arm_command_execute() because arming the return
     is application state, not command semantics.  The move itself is started by
     MotionTask once the queue is empty, never from this function, so the M400
     itself still costs nothing. */
  if ((local.id == 'M') && (local.num == 400))
  {
    s_auto_home = AUTO_HOME_ARMED;
    arm_uart_debug("M400: auto home armed - returns to 0/120/120 at F10 "
                   "when the queue drains\r\n");
    return;
  }

  if ((local.id == 'G') && ((local.num == 0) || (local.num == 1)))
  {
    arm_ack_t ack = arm_command_execute(&local, &warn);

    if (ack == ARM_ACK_FAULT)
    {
      /* A soft limit refused the move: there is nothing to interpolate, so the
         deferred part of the answer goes out immediately.  "ok" already went
         out when the command was queued, exactly as the original firmware
         did, and this "!!" reports the failure that was only discovered once
         the command reached the front of the queue. */
      s_motion_state = MOTION_IDLE;
      arm_uart_send_deferred(ack, warn);
      return;
    }

    s_motion_state = MOTION_MOVING;
    return;
  }

  /* Everything else executes instantly.  "ok" has already been sent at enqueue
     time, so only the deferred part is produced here. */
  arm_uart_send_deferred(arm_command_execute(&local, &warn), warn);

  /* M18 additionally aborts whatever is still in flight. */
  if ((local.id == 'M') && (local.num == 18))
  {
    arm_stepper_stop_all();
  }

  s_motion_state = MOTION_IDLE;
}

/**
  * @brief  Start the automatic return to the power-on pose (M400).
  * @note   The return is an ordinary G1 through the existing interpolation and
  *         per-cycle path guard, only synthesised here instead of arriving on
  *         the serial link.  It is deliberately slow (ARM_AUTO_HOME_F_MM_S): a
  *         return from a pressed-down pose lifts the lower arm, which is
  *         exactly the direction in which this machine loses steps above about
  *         77 mm/s.
  */
static void arm_motion_auto_home_start(void)
{
  arm_cmd_t cmd;

  /* A disabled driver would only advance the model: the position would walk to
     home while the machine stands still, which is one whole return worth of
     desync.  Refuse instead of doing that silently. */
  if ((arm_stepper_is_enabled(ARM_AXIS_ROT) == false) ||
      (arm_stepper_is_enabled(ARM_AXIS_LOW) == false) ||
      (arm_stepper_is_enabled(ARM_AXIS_HIGH) == false))
  {
    s_auto_home = AUTO_HOME_OFF;
    arm_uart_debug("auto home skipped: drivers disabled (send M17 first)\r\n");
    return;
  }

  if (arm_limits_is_faulted())
  {
    s_auto_home = AUTO_HOME_OFF;
    arm_uart_debug("auto home skipped: machine faulted\r\n");
    return;
  }

  memset(&cmd, 0, sizeof(cmd));
  cmd.id     = 'G';
  cmd.num    = 1;
  cmd.valueX = ARM_BOOT_X_MM;
  cmd.valueY = ARM_BOOT_Y_MM;
  cmd.valueZ = ARM_BOOT_Z_MM;
  cmd.valueF = ARM_AUTO_HOME_F_MM_S;

  s_auto_home = AUTO_HOME_RUNNING;
  arm_motion_start_command(&cmd);

  if (s_motion_state != MOTION_MOVING)
  {
    /* arm_command_move() refused the target and has already answered on the
       host link, so there is nothing to add here but the debug trace. */
    s_auto_home = AUTO_HOME_OFF;
    arm_uart_debug("auto home refused by the target check\r\n");
    return;
  }

  arm_uart_debug("auto home: returning to 0/120/120\r\n");
}

/** Convert one joint pose into step requests for the three axes. */
static void arm_motion_apply_joints(const arm_joints_t *joints)
{
  static const arm_axis_t order[ARM_AXIS_COUNT] =
  {
    /* Same order as robotArm.ino:88-90, so the axes are serviced in the same
       sequence the original firmware used. */
    ARM_AXIS_ROT, ARM_AXIS_LOW, ARM_AXIS_HIGH
  };

  float wanted[ARM_AXIS_COUNT];
  int32_t desired[ARM_AXIS_COUNT];

  wanted[ARM_AXIS_ROT]  = joints->rot;
  wanted[ARM_AXIS_LOW]  = joints->low;
  wanted[ARM_AXIS_HIGH] = joints->high;

  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    arm_axis_t axis = order[i];

    desired[axis] = (int32_t)(wanted[axis] * ARM_RAD_TO_STEP);
  }

  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    arm_axis_t axis = order[i];
    int32_t current = arm_stepper_get_position_steps(axis);
    int32_t delta   = desired[axis] - current;

    if (delta == 0)
    {
      /* Still call in with zero: arm_stepper_step() advances its rate
         measurement window on every call, and skipping the ticks where no step
         is due would make a slow move look 6x faster than it is. */
      (void)arm_stepper_step(axis, 0);
      continue;
    }

    {
      int32_t accepted = arm_stepper_step(axis, delta);

      if (accepted != delta)
      {
        /* The pending budget was momentarily full, or a direction change had to
           wait for the pulse that was already on the wire.  The rejected
           remainder is simply retried on the next cycle, at which point the
           interpolation has barely moved, so the tracking error stays below one
           pulse. */
        s_shortfall++;
      }
    }
  }
}

/*============================================================================*/
/* CommandTask, blocks on the serial input                                    */
/*============================================================================*/

/**
  * @brief  [学习标注] 1 ms 轮询串口：拆行、解析、回 ok；启动时先打印脉冲引擎自检结果。
  */
static void arm_command_task(void *argument)
{
  arm_fault_mark(ARM_STAGE_COMMAND_TASK);

  (void)argument;

#if (ARM_EMIT_START_BANNER == 1)
  /* robotArm.ino:69 printed "start" once after setup().  Off by default: the
     Delphi host only ever releases its stop-and-wait lock when the FIRST line
     it receives starts with "ok" (UnitRobotArm.pas:162-166), so an unbidden
     leading line wedges the host after one command.  See arm_config.h.
     Sent from this task rather than from main() so that it happens in normal
     scheduler context, before the first arm_uart_poll() can answer anything. */
  arm_uart_send_line("start");
#endif

  /* Report the outcome of the boot time pulse engine self test.  It used to
     call Error_Handler() on failure, which froze the firmware before the UART
     existed; now it only shows up here, so a dead pulse engine is visibly
     different from a dead board. */
  if (arm_stepper_selftest_ok())
  {
    arm_uart_debug("step engine self test: OK (TIM7 pulse tick is being served)\r\n");
  }
  else
  {
    arm_uart_debug("step engine self test: *** FAILED *** the pulse timer never "
                   "interrupted; no axis will emit pulses "
                   "(check the TIM7 clock, PSC/ARR and the TIM7 NVIC line)\r\n");
  }

  for (;;)
  {
    /* arm_uart_poll() drains the DMA ring, assembles complete lines and calls
       arm_command_handle_line() for each one.  It is deliberately driven from
       this task rather than from an interrupt, because the command queue and
       the "ok" answer both need a task context. */
    arm_uart_poll();

    osDelay(1u);
  }
}

/*============================================================================*/
/* SafetyTask, 10 ms                                                          */
/*============================================================================*/

/**
  * @brief  [学习标注] 10 ms 调一次限位/急停状态机（arm_limits_safety_task）。
  */
static void arm_safety_task(void *argument)
{
  arm_fault_mark(ARM_STAGE_SAFETY_TASK);

  TickType_t last_wake = xTaskGetTickCount();

  (void)argument;

  for (;;)
  {
    arm_limits_safety_task();

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARM_SAFETY_PERIOD_MS));
  }
}

/*============================================================================*/
/* LedTask, 250 ms                                                            */
/*============================================================================*/

/**
  * @brief  [学习标注] 250 ms 翻转 PE0 心跳；有故障/WARN 时让 PE1 闪烁。
  */
static void arm_led_task(void *argument)
{
  arm_fault_mark(ARM_STAGE_LED_TASK);

  TickType_t last_wake = xTaskGetTickCount();
  uint8_t    heartbeat = 0u;

  (void)argument;

  for (;;)
  {
    /* PE0 heartbeat, 250 ms toggle, matching LED_PIN in robotArm.ino:103-107
       but at the section 4.2.5 rate. */
    heartbeat ^= 1u;
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin,
                      (heartbeat != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* PE1 alarm, 5 Hz while a fault, a warning flash or an E-stop is latched. */
    if (arm_limits_alarm_active() || arm_limits_is_faulted())
    {
      HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
    }
    else
    {
      HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARM_HEARTBEAT_PERIOD_MS));
  }
}

/*============================================================================*/
/* defaultTask body is arm_app_run_default_task(), defined above next to       */
/* arm_app_init() so that the two stay together.                              */
/*============================================================================*/
