/**
  ******************************************************************************
  * @file    arm_interpolation.h
  * @brief   Cosine eased straight line interpolation, translated from
  *          interpolation.cpp.
  ******************************************************************************
  */

#ifndef ARM_INTERPOLATION_H
#define ARM_INTERPOLATION_H

#include "arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start a new interpolation segment from p0 to p1.
  * @param v  feed rate in mm/s; values below 5 select the legacy law.        */
void arm_interp_start(float p0x, float p0y, float p0z,
                      float p1x, float p1y, float p1z, float v);

/** Advance the virtual position.  Call once per control period. */
void arm_interp_update(void);

/** True once the segment has been completed. */
bool arm_interp_is_finished(void);

/** Current virtual position in millimetres. */
float arm_interp_get_x(void);
/* [学习标注] 当前虚拟位置 Y（mm）。 */
float arm_interp_get_y(void);
/* [学习标注] 当前虚拟位置 Z（mm）。 */
float arm_interp_get_z(void);

/** Last commanded target in millimetres (the segment end point). */
float arm_interp_get_target_x(void);
/* [学习标注] 本段目标 Y（mm）。 */
float arm_interp_get_target_y(void);
/* [学习标注] 本段目标 Z（mm）。 */
float arm_interp_get_target_z(void);

/** Force the virtual position, used for synchronisation after a fault. */
void arm_interp_set_position(float x, float y, float z);

/**
  * @brief  Freeze the running segment at its current progress.
  * @note   Stalls the segment clock by re-anchoring the start time so that the
  *         elapsed value no longer grows.  Used while a fault is latched: the
  *         trajectory must not keep advancing in virtual time while the
  *         hardware is standing still, otherwise the next command would be
  *         planned from a pose the arm never reached.
  */
void arm_interp_stall(void);

/** tmul of the running segment, exposed for the debug log. */
float arm_interp_get_tmul(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_INTERPOLATION_H */
