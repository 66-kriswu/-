/**
  ******************************************************************************
  * @file    arm_interpolation.c
  * @brief   Cosine interpolation, translated from interpolation.cpp.
  *
  *          Deliberately preserved quirks
  *          ------------------------------
  *          1. The legacy speed law v = sqrt(dist) * 10 (interpolation.cpp:69)
  *             is kept behind ARM_KEEP_LEGACY_SPEED, because the Delphi host
  *             does not send an F word at all: UnitRobotArm.pas:131 computes
  *             the same law on the PC side and the firmware has to agree with
  *             it or the motion would feel different from the original.
  *          2. tmul = v / dist (interpolation.cpp:75) is not a rate.  The
  *             progress curve is -cos(t * tmul * PI) * 0.5 + 0.5, so the
  *             segment lasts 1 / tmul seconds whatever the distance is.
  *             Changing this would change the machine's feel, so it stays.
  *          3. Time is taken from the 32 bit microsecond counter, which wraps
  *             every 71.58 minutes like the original micros().  The wrap is
  *             harmless: elapsed is computed with unsigned modular
  *             subtraction, exactly as the C standard guarantees.
  ******************************************************************************
  */

#include "arm_interpolation.h"

#include <math.h>

#include "arm_config.h"
#include "arm_us.h"

/*============================= Private state ===============================*/

static float    s_x_start;
static float    s_y_start;
static float    s_z_start;

static float    s_x_delta;
static float    s_y_delta;
static float    s_z_delta;

static float    s_x_pos;
static float    s_y_pos;
static float    s_z_pos;

static float    s_x_target;
static float    s_y_target;
static float    s_z_target;

static float    s_tmul;
static uint32_t s_start_us;
static uint8_t  s_state;      /**< 0 = running, 1 = finished */

/*============================================================================*/

/**
  * @brief  [学习标注] 起一段余弦插补：算弦长、速度钳位(F≤120)、tmul=v/dist、记录起点与目标。
  */
void arm_interp_start(float p0x, float p0y, float p0z,
                      float p1x, float p1y, float p1z, float v)
{
  float a = p1x - p0x;
  float b = p1y - p0y;
  float c = p1z - p0z;
  float dist = sqrtf((a * a) + (b * b) + (c * c));

#if (ARM_KEEP_LEGACY_SPEED == 1)
  if (v < 5.0f)
  {
    /* interpolation.cpp:68-70: "set a good value for v". */
    v = sqrtf(dist) * 10.0f;
  }
#endif

  if (v < 5.0f)
  {
    v = 5.0f;
  }

  /* Hard ceiling, see ARM_MAX_SPEED_MM_S.  Applied after both laws so that a
     typo in an F word cannot ask the pulse engine for more steps per second
     than the motors can start from without losing steps. */
  if (v > ARM_MAX_SPEED_MM_S)
  {
    v = ARM_MAX_SPEED_MM_S;
  }

  /* interpolation.cpp:75.  Guarded against a zero length segment, where the
     original would divide by zero and produce inf. */
  s_tmul = (dist > 0.0f) ? (v / dist) : 0.0f;

  s_x_start = p0x;
  s_y_start = p0y;
  s_z_start = p0z;

  s_x_delta = a;
  s_y_delta = b;
  s_z_delta = c;

  s_x_target = p1x;
  s_y_target = p1y;
  s_z_target = p1z;

  /* A zero length segment is finished immediately, which is the behaviour the
     original reaches one millisecond later after clamping progress to 1. */
  s_state    = (dist > 0.0f) ? 0u : 1u;
  s_start_us = arm_us_micros();

  s_x_pos = p0x;
  s_y_pos = p0y;
  s_z_pos = p0z;
}

/**
  * @brief  [学习标注] 每 1 ms 按 elapsed 推进进度：progress = −cos(t·tmul·π)/2 + 0.5。
  */
void arm_interp_update(void)
{
  float t;
  float progress;
  uint32_t now;
  uint32_t elapsed;

  if (s_state != 0u)
  {
    return;
  }

  now     = arm_us_micros();
  elapsed = now - s_start_us;              /* wrap safe, unsigned */
  t       = (float)elapsed / 1000000.0f;

  /* interpolation.cpp:108-112, cosine easing. */
  if (s_tmul <= 0.0f)
  {
    progress = 1.0f;
    s_state  = 1u;
  }
  else if ((t * s_tmul) >= 1.0f)
  {
    progress = 1.0f;
    s_state  = 1u;
  }
  else
  {
    progress = -cosf(t * s_tmul * ARM_PI) * 0.5f + 0.5f;
  }

  s_x_pos = s_x_start + (progress * s_x_delta);
  s_y_pos = s_y_start + (progress * s_y_delta);
  s_z_pos = s_z_start + (progress * s_z_delta);
}

/**
  * @brief  [学习标注] 本段是否已走完（脉冲引擎据此切“排空速率”）。
  */
bool arm_interp_is_finished(void)
{
  return (s_state != 0u);
}

/**
  * @brief  [学习标注] 当前虚拟位置 X（mm）。
  */
float arm_interp_get_x(void)
{
  return s_x_pos;
}

/**
  * @brief  [学习标注] 当前虚拟位置 Y（mm）。
  */
float arm_interp_get_y(void)
{
  return s_y_pos;
}

/**
  * @brief  [学习标注] 当前虚拟位置 Z（mm）。
  */
float arm_interp_get_z(void)
{
  return s_z_pos;
}

/**
  * @brief  [学习标注] 本段目标 X（mm）。
  */
float arm_interp_get_target_x(void)
{
  return s_x_target;
}

/**
  * @brief  [学习标注] 本段目标 Y（mm）。
  */
float arm_interp_get_target_y(void)
{
  return s_y_target;
}

/**
  * @brief  [学习标注] 本段目标 Z（mm）。
  */
float arm_interp_get_target_z(void)
{
  return s_z_target;
}

/**
  * @brief  [学习标注] 本段 tmul（= v/dist，diag 用）。
  */
float arm_interp_get_tmul(void)
{
  return s_tmul;
}

/**
  * @brief  [学习标注] 直接改虚拟位置（上电 / M210 对锚点用）。
  */
void arm_interp_set_position(float x, float y, float z)
{
  s_x_pos = x;
  s_y_pos = y;
  s_z_pos = z;
}

/**
  * @brief  [学习标注] 冻住进度但不改状态：把起始时刻往前推“已经过去的时间”，下次 elapsed 就不
  *         变。
  */
void arm_interp_stall(void)
{
  if (s_state != 0u)
  {
    return;
  }

  /* Move the anchor forward by exactly the time that has passed, so the next
     call to arm_interp_update() sees the same elapsed value as this one.  The
     segment therefore holds its progress instead of running to its end. */
  s_start_us += arm_us_elapsed(s_start_us);
}
