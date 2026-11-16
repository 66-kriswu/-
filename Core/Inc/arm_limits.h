/**
  ******************************************************************************
  * @file    arm_limits.h
  * @brief   Limit switches, emergency stop and the fault state machine.
  *
  *          Handover doc section 4.9:
  *            - the EXTI interrupt does exactly three things: clear the flag,
  *              record the edge time stamp and set an event flag.  No float,
  *              no printing, no DMA call.  The real stop happens in SafetyTask
  *            - a trigger aborts the affected axis and answers "!!"
  *            - M18 clears the fault and disables the drivers, and then motion
  *              away from the limit is allowed again
  ******************************************************************************
  */

#ifndef ARM_LIMITS_H
#define ARM_LIMITS_H

#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os.h"      /* osEventFlagsId_t */
#include "arm_config.h"
#include "arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Soft limit verdict for a commanded joint pose. */
typedef enum
{
  ARM_LIMIT_OK = 0,
  ARM_LIMIT_ROT,
  ARM_LIMIT_LOW,
  ARM_LIMIT_HIGH,
  ARM_LIMIT_ELBOW      /**< the two arms are folded too far together or pulled
                            past straight; see ARM_LIM_ELBOW_* in arm_config.h */
} arm_limit_verdict_t;

/*============================== Life cycle =================================*/

/**
  * @brief  Create the event group used by the interrupt handlers.
  * @note   Call after osKernelInitialize(), before osKernelStart().
  */
void arm_limits_init(void);

/**
  * @brief  Create the event group that the EXTI handlers signal into.
  * @param  evt  event group created by the application layer.
  */
void arm_limits_set_event_group(osEventFlagsId_t evt);

/*============================= Interrupt path ==============================*/

/** Body of HAL_GPIO_EXTI_Callback, called from the EXTI vectors. */
void arm_limits_gpio_exti_callback(uint16_t pin);

/*============================= Safety task =================================*/

/**
  * @brief  One pass of the safety state machine.  Called by SafetyTask every
  *         ARM_SAFETY_PERIOD_MS milliseconds.
  */
void arm_limits_safety_task(void);

/** Poll the six switches, debounce them and raise events.
  *
  *  The E-stop is additionally checked by LEVEL on every pass, so a button that
  *  is already held at power up - or still held when M17 re-arms - latches
  *  instead of leaving the drivers live.                                       */
void arm_limits_scan(void);

/*================================ Queries ==================================*/

/** True while a limit fault or an emergency stop latches the machine. */
bool arm_limits_is_faulted(void);

/** True after an emergency stop until M17 re-arms the drives. */
bool arm_limits_is_estopped(void);

/** True while the alarm LED should be blinking fast. */
bool arm_limits_alarm_active(void);

/** Convert a commanded joint pose into a soft limit verdict. */
arm_limit_verdict_t arm_limits_check_pose(float rot_rad, float low_rad, float high_rad);

/** Human readable name of an axis, for log lines. */
const char *arm_limits_axis_name(arm_axis_t axis);

/** Human readable name of a limit switch. */
const char *arm_limits_switch_name(arm_axis_t axis, bool maximum);

/*================================ Actions ==================================*/

/** Clear the latched fault, used by M18. */
void arm_limits_clear_fault(void);

/** Re-arm after an emergency stop, used by M17. */
void arm_limits_clear_estop(void);

/** Start the one second alarm blink, used for WARN answers. */
void arm_limits_flash_warn(void);

/** Current raw state of the six switches, bit 0..5, for diagnostics. */
uint8_t arm_limits_raw_state(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_LIMITS_H */
