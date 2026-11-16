/**
  ******************************************************************************
  * @file    arm_types.h
  * @brief   Shared data types.  Mirrors the Arduino Cmd struct and the
  *          error codes used by the serial protocol.
  ******************************************************************************
  */

#ifndef ARM_TYPES_H
#define ARM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#include "arm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Axis identifiers.  The order matches the original object creation order in
  * robotArm.ino:13-15 (stepperRotate, stepperLower, stepperHigher).          */
typedef enum
{
  ARM_AXIS_ROT  = 0,   /**< base rotation,   RAMPS Z_STEP_PIN -> TIM2_CH1 / PA0 */
  ARM_AXIS_LOW  = 1,   /**< lower arm,       RAMPS Y_STEP_PIN -> TIM3_CH2 / PA7 */
  ARM_AXIS_HIGH = 2,   /**< upper arm,       RAMPS X_STEP_PIN -> TIM3_CH1 / PA6 */
  ARM_AXIS_COUNT = 3
} arm_axis_t;

/** One parsed G/M command.  Field names follow command.h in the original.    */
typedef struct
{
  char     id;                                  /**< 'G' or 'M'             */
  int16_t  num;                                 /**< command number         */
  float    valueX;                              /**< NAN when not given     */
  float    valueY;
  float    valueZ;
  float    valueE;
  float    valueF;                              /**< feed rate, mm/s        */
  float    valueT;                              /**< dwell / gripper count  */
} arm_cmd_t;

/** Execution result of one command, used for the serial answer.             */
typedef enum
{
  ARM_ACK_OK = 0,      /**< plain "ok"                                     */
  ARM_ACK_WARN,        /**< "ok" then "// WARN: ..."                       */
  ARM_ACK_FAULT,       /**< "ok" then "// WARN: ..." then "!!"             */
  ARM_ACK_RESEND       /**< "rs" (parse error, nothing was queued)         */
} arm_ack_t;

/** Event group bits, section 4.10.                                          */
#define ARM_EVT_MOVE_DONE                (1UL << 0)
#define ARM_EVT_LIMIT_ROT                (1UL << 1)
#define ARM_EVT_LIMIT_LOW                (1UL << 2)
#define ARM_EVT_LIMIT_HIGH               (1UL << 3)
#define ARM_EVT_ESTOP                    (1UL << 4)
#define ARM_EVT_FAULT                    (1UL << 5)

#define ARM_EVT_LIMIT_ALL                (ARM_EVT_LIMIT_ROT | ARM_EVT_LIMIT_LOW | ARM_EVT_LIMIT_HIGH)

#ifdef __cplusplus
}
#endif

#endif /* ARM_TYPES_H */
