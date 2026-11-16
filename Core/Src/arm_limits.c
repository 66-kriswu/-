/**
  ******************************************************************************
  * @file    arm_limits.c
  * @brief   Limit switches, emergency stop and the fault state machine.
  *
  *          Interrupt side (section 4.9)
  *          ----------------------------
  *          The six switches live on PC0..PC4 (EXTI0..EXTI4) and PE6 (EXTI9_5).
  *          Each limit pin owns a private EXTI vector, so no shared handler has
  *          to work out which line fired by reading EXTI->PR.
  *
  *          The handler does exactly three things and nothing else:
  *            1. HAL_GPIO_EXTI_Callback() clears the pending flag
  *            2. store the edge time stamp
  *            3. set an event flag with the FromISR API
  *          No floating point, no printing, no DMA call.  Stopping the axis is
  *          done by arm_limits_safety_task() in task context.
  *
  *          Switch polarity
  *          ---------------
  *          RAMPS 1.4 end stops are normally closed to ground, so an open
  *          switch reads high through the internal pull up and a triggered one
  *          reads low.  Both edges are watched: the falling edge latches the
  *          fault, the rising edge is recorded so the debounce window can
  *          confirm that the switch really moved rather than bounced.
  *
  *          Because the real polarity of the user's wiring is not known yet,
  *          every transition is also reported on the debug UART.  That way the
  *          wiring can be identified from the log without a scope.
  ******************************************************************************
  */

#include "arm_limits.h"

#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_stepper.h"
#include "arm_uart.h"
#include "arm_us.h"

/*============================== Private defines =============================*/

/** Logic level that means "this end stop is triggered", derived from the wiring
  * style selected in arm_config.h.  GPIO_PinState is 0 for RESET, 1 for SET. */
#if (ARM_LIMIT_ACTIVE_LOW == 1)
#define ARM_LIMIT_ACTIVE_LEVEL            ((uint8_t)0)
#else
#define ARM_LIMIT_ACTIVE_LEVEL            ((uint8_t)1)
#endif

/** The emergency stop is a momentary button to GND, so it is always low. */
#define ARM_ESTOP_ACTIVE_LEVEL            ((uint8_t)0)

/*============================== Private types ===============================*/

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  arm_axis_t    axis;
  bool          maximum;      /**< TRUE for a MAX switch */
  uint8_t       level;        /**< last sampled level                      */
  uint32_t      last_edge_us; /**< time of the most recent transition       */
  uint8_t       stable;       /**< last debounced level                     */
} arm_switch_t;

/*============================= Private state ===============================*/

static osEventFlagsId_t s_evt = NULL;

/* Bits latched by the EXTI handler and published to the event group by
   arm_limits_safety_task().  The interrupt deliberately calls NO FreeRTOS API:
   osEventFlagsSet() from an ISR has to be routed through the timer daemon task
   (configUSE_OS2_EVENTFLAGS_FROM_ISR), which drags the whole software timer
   machinery into the firmware for no reason - a plain volatile latch is both
   cheaper and impossible to misuse. */
static volatile uint32_t s_isr_events = 0u;

static arm_switch_t s_sw[6];

/** Global fault latch.  Cleared by M18 or by a successful M17. */
static volatile uint8_t s_fault_latched = 0u;
static volatile uint8_t s_fault_reported = 0u;
static volatile uint8_t s_estop_latched = 0u;
static volatile uint8_t s_estop_reported = 0u;

/* Alarm LED state. */
static volatile uint8_t  s_alarm = 0u;
static volatile uint32_t s_alarm_until_ms = 0u;

/* Diagnostic counters. */
static volatile uint32_t s_edge_count[6];

/*=========================== Internal prototypes ===========================*/

static void arm_limits_handle_falling(arm_axis_t axis, bool maximum);
static void arm_limits_handle_estop(void);
static void arm_limits_alarm_set(uint32_t duration_ms);

/*============================================================================*/
/* Construction                                                              */
/*============================================================================*/

/**
  * @brief  [学习标注] 建立 6 路开关 → (轴, MIN/MAX) 映射，并把上电时的电平记为基线。
  */
void arm_limits_init(void)
{
  /* PC0..PC2 and PE6 are MIN switches, PC3/PC4 are MAX switches.  The pin to
     axis assignment follows the original pinout.h, including the
     X_STEP/Z_STEP swap that robotArm.ino:13-15 documents. */
  s_sw[0].port = X_MIN_GPIO_Port; s_sw[0].pin = X_MIN_Pin;   /* base rotation */
  s_sw[0].axis = ARM_AXIS_ROT;    s_sw[0].maximum = false;

  s_sw[1].port = Y_MIN_GPIO_Port; s_sw[1].pin = Y_MIN_Pin;   /* lower arm     */
  s_sw[1].axis = ARM_AXIS_LOW;    s_sw[1].maximum = false;

  s_sw[2].port = Z_MIN_GPIO_Port; s_sw[2].pin = Z_MIN_Pin;   /* upper arm     */
  s_sw[2].axis = ARM_AXIS_HIGH;   s_sw[2].maximum = false;

  s_sw[3].port = X_MAX_GPIO_Port; s_sw[3].pin = X_MAX_Pin;   /* base rotation */
  s_sw[3].axis = ARM_AXIS_ROT;    s_sw[3].maximum = true;

  /* Note: RAMPS has no Y_MAX switch.  Z_MAX (PC3) belongs to the upper arm,
     whose STEP line is the one labelled X in pinout.h. */
  s_sw[4].port = Z_MAX_GPIO_Port; s_sw[4].pin = Z_MAX_Pin;   /* upper arm     */
  s_sw[4].axis = ARM_AXIS_HIGH;   s_sw[4].maximum = true;

  s_sw[5].port = ESTOP_GPIO_Port; s_sw[5].pin = ESTOP_Pin;   /* emergency stop */
  s_sw[5].axis = ARM_AXIS_ROT;    s_sw[5].maximum = false;

  for (uint32_t i = 0u; i < 6u; i++)
  {
    s_sw[i].level        = (uint8_t)HAL_GPIO_ReadPin(s_sw[i].port, s_sw[i].pin);
    s_sw[i].stable       = s_sw[i].level;
    s_sw[i].last_edge_us = arm_us_micros();
    s_edge_count[i]      = 0u;
  }
}

/**
  * @brief  [学习标注] 注入应用层创建的事件组（ISR 只置位，由任务发布）。
  */
void arm_limits_set_event_group(osEventFlagsId_t evt)
{
  s_evt = evt;
}

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

/**
  * @brief  [学习标注] 轴名，日志用：rot / low / high。
  */
const char *arm_limits_axis_name(arm_axis_t axis)
{
  switch (axis)
  {
    case ARM_AXIS_ROT:  return "rot";
    case ARM_AXIS_LOW:  return "low";
    case ARM_AXIS_HIGH: return "high";
    default:            return "?";
  }
}

/**
  * @brief  [学习标注] 开关名，日志用：X_MIN / Y_MIN / Z_MIN / Z_MAX / X_MAX。
  */
const char *arm_limits_switch_name(arm_axis_t axis, bool maximum)
{
  switch (axis)
  {
    case ARM_AXIS_ROT:  return maximum ? "X_MAX" : "X_MIN";
    case ARM_AXIS_LOW:  return maximum ? "Y_MAX" : "Y_MIN";
    case ARM_AXIS_HIGH: return maximum ? "Z_MAX" : "Z_MIN";
    default:            return "?";
  }
}

/**
  * @brief  [学习标注] 点亮告警灯一个时间窗（到点由安全任务熄灭）。
  */
static void arm_limits_alarm_set(uint32_t duration_ms)
{
  s_alarm          = 1u;
  s_alarm_until_ms = (uint32_t)osKernelGetTickCount() + duration_ms;
}

/**
  * @brief  [学习标注] 告警灯是否还在闪。
  */
bool arm_limits_alarm_active(void)
{
  return (s_alarm != 0u);
}

/**
  * @brief  [学习标注] WARN 用：让 PE1 闪 1 秒。
  */
void arm_limits_flash_warn(void)
{
  arm_limits_alarm_set(ARM_WARN_FLASH_MS);
}

/**
  * @brief  [学习标注] 是否处于故障锁存（限位或急停）。
  */
bool arm_limits_is_faulted(void)
{
  return (s_fault_latched != 0u);
}

/**
  * @brief  [学习标注] 是否处于急停锁存（只有 M17 能清）。
  */
bool arm_limits_is_estopped(void)
{
  return (s_estop_latched != 0u);
}

/**
  * @brief  [学习标注] 6 路开关原始电平位图（diag 的 sw：bit0..4 限位、bit5 急停）。
  */
uint8_t arm_limits_raw_state(void)
{
  uint8_t value = 0u;

  for (uint32_t i = 0u; i < 6u; i++)
  {
    if (HAL_GPIO_ReadPin(s_sw[i].port, s_sw[i].pin) == GPIO_PIN_SET)
    {
      value |= (uint8_t)(1u << i);
    }
  }

  return value;
}

/*============================================================================*/
/* Interrupt path                                                             */
/*============================================================================*/

/**
  * @brief  [学习标注] EXTI 回调（6 路共用）：只记录时间戳/边沿计数并置 volatile 事件位，不做停
  *         机。
  * @note   停机放在 SafetyTask，ISR 里不调用任何 RTOS API（见 §3.10.1）。
  */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  uint32_t now = arm_us_micros();

  /* The HAL already cleared the pending bit before calling us.  Only store the
     time stamp and set the event flag here - nothing else is interrupt safe
     with respect to the 30 us pulse engine. */
  if (gpio_pin == ESTOP_Pin)
  {
    s_sw[5].last_edge_us = now;
    if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET)
    {
      s_edge_count[5]++;
      s_isr_events |= ARM_EVT_ESTOP;
    }
    return;
  }

  for (uint32_t i = 0u; i < 5u; i++)
  {
    if (gpio_pin == s_sw[i].pin)
    {
      s_sw[i].last_edge_us = now;
      s_edge_count[i]++;
      if (s_sw[i].axis == ARM_AXIS_ROT)
      {
        s_isr_events |= ARM_EVT_LIMIT_ROT;
      }
      else if (s_sw[i].axis == ARM_AXIS_LOW)
      {
        s_isr_events |= ARM_EVT_LIMIT_LOW;
      }
      else
      {
        s_isr_events |= ARM_EVT_LIMIT_HIGH;
      }
      return;
    }
  }
}

/*============================================================================*/
/* Debounced polling                                                          */
/*============================================================================*/

/**
  * @brief  [学习标注] 10 ms 扫描：急停按电平判、5 路限位做 5 ms 去抖；触发则停机并锁存。
  * @note   急停必须在中断置位之外再按电平判一次，否则“开机就按着/按住 M17”都漏（案例 12 的反
  *         面）。
  */
void arm_limits_scan(void)
{
  /* Emergency stop is a hard interlock and is therefore polled by LEVEL, not
     by edge.  A button that is already held at power up, or that is still held
     when M17 re-arms the drivers, has to latch again: an edge-only stop would
     leave the machine live under a pressed stop button, which is the one
     failure a stop button must never allow.  One GPIO read per safety period,
     and the latch makes it idempotent. */
  if ((s_estop_latched == 0u) &&
      (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == ARM_ESTOP_ACTIVE_LEVEL))
  {
    arm_limits_handle_estop();
  }

  for (uint32_t i = 0u; i < 6u; i++)
  {
    uint8_t level = (uint8_t)HAL_GPIO_ReadPin(s_sw[i].port, s_sw[i].pin);

    if (level != s_sw[i].level)
    {
      s_sw[i].level        = level;
      s_sw[i].last_edge_us = arm_us_micros();
      continue;   /* start the settle window again */
    }

    /* The level has been steady; accept it once the debounce window elapsed. */
    if (s_sw[i].stable != level)
    {
      if (arm_us_elapsed(s_sw[i].last_edge_us) >= (ARM_LIMIT_DEBOUNCE_MS * 1000u))
      {
        s_sw[i].stable = level;

        arm_uart_debugf("sw %s %s -> %s (edges %lu)\r\n",
                        arm_limits_axis_name(s_sw[i].axis),
                        arm_limits_switch_name(s_sw[i].axis, s_sw[i].maximum),
                        (level == ARM_LIMIT_ACTIVE_LEVEL) ? "TRIGGERED" : "idle",
                        (unsigned long)s_edge_count[i]);

        if (i == 5u)
        {
          /* The level itself was handled at the top of this function; this
             only records the debounced state and its log line. */
          if ((level == ARM_ESTOP_ACTIVE_LEVEL) && (s_estop_latched == 0u))
          {
            arm_limits_handle_estop();
          }
        }
        else if (level == ARM_LIMIT_ACTIVE_LEVEL)
        {
          arm_limits_handle_falling(s_sw[i].axis, s_sw[i].maximum);
        }
      }
    }
  }
}

/*============================================================================*/
/* Fault actions                                                              */
/*============================================================================*/

/**
  * @brief  [学习标注] 限位动作：先停该轴，再锁存故障、置事件、打日志。
  */
static void arm_limits_handle_falling(arm_axis_t axis, bool maximum)
{
  /* Stop first, report afterwards: the axis must not advance even by the time
     it takes to build the log line. */
  arm_stepper_stop(axis);

  arm_limits_alarm_set(ARM_WARN_FLASH_MS);

  if (s_fault_latched == 0u)
  {
    s_fault_latched  = 1u;
    s_fault_reported = 0u;

    if (s_evt != NULL)
    {
      (void)osEventFlagsSet(s_evt, ARM_EVT_FAULT);
    }
  }

  arm_uart_debugf("LIMIT %s on %s axis, pulses stopped\r\n",
                  arm_limits_switch_name(axis, maximum),
                  arm_limits_axis_name(axis));
}

/**
  * @brief  [学习标注] 急停动作：停三轴 + 双锁存 + 上报；默认保持驱动器使能（
  *         ARM_ESTOP_KEEP_TORQUE=1，避免掉臂）。
  */
static void arm_limits_handle_estop(void)
{
  /* Stopping the pulse stream is what the stop button has to do within one
     tick.  Whether the drivers are also de-energised is a separate decision,
     see ARM_ESTOP_KEEP_TORQUE: this machine has no brake and the power-on pose
     has the lower arm horizontal, so dropping the enable lines would let the
     arm fall.  Keeping the torque holds the pose; M18 is then the explicit way
     to release it. */
  arm_stepper_stop_all();

#if (ARM_ESTOP_KEEP_TORQUE == 0)
  arm_stepper_enable_all(false);
#endif

  s_estop_latched  = 1u;
  s_estop_reported = 0u;
  s_fault_latched  = 1u;
  s_fault_reported = 0u;

  arm_limits_alarm_set(ARM_WARN_FLASH_MS);

  if (s_evt != NULL)
  {
    (void)osEventFlagsSet(s_evt, ARM_EVT_FAULT);
  }

#if (ARM_ESTOP_KEEP_TORQUE == 1)
  arm_uart_debug("EMERGENCY STOP: pulses stopped, drivers KEEP holding "
                 "(M17 to re-arm, M18 to release)\r\n");
#else
  arm_uart_debug("EMERGENCY STOP: all axes stopped, drivers disabled\r\n");
#endif
}

/**
  * @brief  [学习标注] M18 清故障锁存与告警。
  */
void arm_limits_clear_fault(void)
{
  s_fault_latched  = 0u;
  s_fault_reported = 0u;
  s_alarm          = 0u;
}

/**
  * @brief  [学习标注] M17 清急停锁存（同时清故障）。
  */
void arm_limits_clear_estop(void)
{
  s_estop_latched  = 0u;
  s_estop_reported = 0u;
  arm_limits_clear_fault();
}

/*============================================================================*/
/* Safety task                                                                */
/*============================================================================*/

/**
  * @brief  [学习标注] SafetyTask 主体：发布 ISR 事件 → 处理急停/限位 → 扫描去抖 → 一次性上报
  *         → 告警计时。
  */
void arm_limits_safety_task(void)
{
  uint32_t flags = 0u;

  /* Publish whatever the interrupt latched.  Done here, in task context, so the
     ISR never touches FreeRTOS. */
  if (s_isr_events != 0u)
  {
    uint32_t pending = s_isr_events;

    s_isr_events = 0u;

    if (s_evt != NULL)
    {
      (void)osEventFlagsSet(s_evt, pending);
    }
  }

  if (s_evt != NULL)
  {
    flags = osEventFlagsGet(s_evt);
  }

  /* 1. Emergency stop: the latched bit stays set until M17 re-arms. */
  if ((flags & ARM_EVT_ESTOP) != 0u)
  {
    if (s_estop_latched == 0u)
    {
      arm_limits_handle_estop();
    }

    if (s_evt != NULL)
    {
      (void)osEventFlagsClear(s_evt, ARM_EVT_ESTOP);
    }
  }

  /* 2. Limit events raised by the interrupt.  The pin is verified again in
        task context so a single glitch cannot stop the machine. */
  if ((flags & ARM_EVT_LIMIT_ALL) != 0u)
  {
    if (s_evt != NULL)
    {
      (void)osEventFlagsClear(s_evt, ARM_EVT_LIMIT_ALL);
    }
  }

  /* 3. Debounced polling catches every state the interrupts may have missed,
        for instance a switch that was already closed at power up. */
  arm_limits_scan();

  /* 4. Report a latched fault exactly once, so the host sees a single "!!"
        per event instead of a continuous stream. */
  if ((s_fault_latched != 0u) && (s_fault_reported == 0u))
  {
    s_fault_reported = 1u;

    if (s_estop_latched != 0u)
    {
      arm_uart_print_comment("EMERGENCY STOP - send M17 to re-arm");
      arm_uart_print_fault();
    }
    else
    {
      arm_uart_print_comment("LIMIT HIT - axis stopped, send M18 then M17");
      arm_uart_print_fault();
    }
  }

  /* 5. Retire the alarm blink once its window has elapsed. */
  if (s_alarm != 0u)
  {
    if ((int32_t)((uint32_t)osKernelGetTickCount() - s_alarm_until_ms) >= 0)
    {
      s_alarm = 0u;
    }
  }
}

/*============================================================================*/
/* Soft limits                                                                */
/*============================================================================*/

/**
  * @brief  [学习标注] 软限位判定：三关节范围 + 肘角耦合 + NaN 守卫，返回第一个越界项。
  */
arm_limit_verdict_t arm_limits_check_pose(float rot_rad, float low_rad, float high_rad)
{
#if (ARM_SOFT_LIMIT_ENABLE == 1)
  bool bad_rot  = (rot_rad  < ARM_LIM_ROT_MIN_RAD)  || (rot_rad  > ARM_LIM_ROT_MAX_RAD);
  bool bad_low  = (low_rad  < ARM_LIM_LOW_MIN_RAD)  || (low_rad  > ARM_LIM_LOW_MAX_RAD);
  bool bad_high = (high_rad < ARM_LIM_HIGH_MIN_RAD) || (high_rad > ARM_LIM_HIGH_MAX_RAD);

  /* The two shanks cannot pass through each other.  With the corrected
     forward kinematics (arm_geometry.c) the angle between them is
     2*|P| where P = (low + high) / 2 and P <= 0 on the working branch, so
     elbow = -(low + high) = 2*acos(rside / 240).  How far they may close
     depends on how far the lower arm is lifted (measured 2026-11-07). */
  float elbow     = -(low_rad + high_rad);
  float elbow_min = (low_rad > ARM_LIM_ELBOW_SPLIT_RAD)
                    ? ARM_LIM_ELBOW_MIN_NEAR_RAD   /* arm near horizontal */
                    : ARM_LIM_ELBOW_MIN_FAR_RAD;   /* arm lifted          */
  bool  bad_elbow = (elbow < elbow_min) || (elbow > ARM_LIM_ELBOW_MAX_RAD);

  /* A NaN would make every comparison false; reject it explicitly, because a
     NaN here means the geometry produced no solution at all. */
  if ((rot_rad != rot_rad) || (low_rad != low_rad) || (high_rad != high_rad))
  {
    return ARM_LIMIT_ROT;
  }

  if (bad_rot)
  {
    return ARM_LIMIT_ROT;
  }

  if (bad_low)
  {
    return ARM_LIMIT_LOW;
  }

  if (bad_high)
  {
    return ARM_LIMIT_HIGH;
  }

  if (bad_elbow)
  {
    return ARM_LIMIT_ELBOW;
  }
#else
  (void)rot_rad;
  (void)low_rad;
  (void)high_rad;
#endif

  return ARM_LIMIT_OK;
}
