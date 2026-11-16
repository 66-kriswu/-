/**
  ******************************************************************************
  * @file    arm_geometry.h
  * @brief   Inverse kinematics: Cartesian millimetres to the three joint
  *          angles.  Straight translation of robotGeometry.cpp.
  ******************************************************************************
  */

#ifndef ARM_GEOMETRY_H
#define ARM_GEOMETRY_H

#include "arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Joint angles produced for one Cartesian point.  All values are radians. */
typedef struct
{
  float rot;
  float low;
  float high;
} arm_joints_t;

/** Solve the three joint angles for one point. */
void arm_geometry_solve(float xmm, float ymm, float zmm, arm_joints_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ARM_GEOMETRY_H */
