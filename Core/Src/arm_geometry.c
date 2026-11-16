/**
  ******************************************************************************
  * @file    arm_geometry.c
  * @brief   Inverse kinematics for the two-shank arm.
  *
  *          HISTORY
  *          -------
  *          Until 2026-11-07 this file was a one-to-one translation of
  *          robotGeometry.cpp (Florin Tobler 2016) and used
  *
  *              r = L (sin low - sin high)
  *              z = L (cos low - cos high)
  *
  *          Measured on the real machine on 2026-11-07 with a ruler, that is
  *          NOT what the hardware does.  Two points pin it down
  *          (L = 120 mm, z positive DOWN from the shoulder, home = 0/0/-90
  *          = tool at r 120 / z 120):
  *
  *            pose          command                 measured tool
  *            home          low 0,      high -90   r 120, z 120
  *            after G1 X0 Y120 Z60
  *                          low +7.4,   high -60.5 r 178, z 120
  *
  *          Both are satisfied by
  *
  *              r = L (cos low + cos high)
  *              z = L (sin low - sin high)
  *
  *          i.e. the old file had sin and cos swapped (equivalently a 90 deg
  *          shift of the zero of both joints).  The two agree only at the
  *          power-on pose, which is why the bug went unnoticed for so long.
  *          Consequence of the old version: G1 put the tool tens of
  *          millimetres away from the commanded point as soon as the arm left
  *          home.
  *
  *          With P = (low + high) / 2 and Q = (low - high) / 2 the forward
  *          kinematics is
  *
  *              r / L = 2 cos P cos Q ,   z / L = 2 cos P sin Q
  *
  *          so  Q = atan2(z, r),  2 cos P = rside / L  with rside = |(r, z)|,
  *          and the branch that contains the power-on pose is P = -acos(...).
  *
  *          The elbow angle between the two shanks is 2 |P| =
  *          2 acos(rside / 240 mm) - which is exactly the "high" formula
  *          drawn in robotGeometry.pdf, so the drawing was right and the
  *          translation was not.
  *
  *          The F407 has a single precision FPU, so everything stays float.
  ******************************************************************************
  */

#include "arm_geometry.h"

#include <math.h>

#include "arm_config.h"

/**
  * @brief  [学习标注] 逆解：笛卡尔 (X,Y,Z) → (rot, low, high)。P=−acos(rside/240)、Q=atan2(z,
  *         r)、low=P+Q、high=P−Q。
  * @note   旧版 sin/cos 互换，只在 home 重合，是本项目最贵的一个 bug（案例 8）。
  */
void arm_geometry_solve(float xmm, float ymm, float zmm, arm_joints_t *out)
{
  float rrot;    /* radius seen from above            */
  float rside;   /* distance shoulder -> tool         */
  float rot;
  float p;       /* half the sum of the two shank angles */
  float q;       /* half their difference                */
  float low;
  float high;

  rrot  = sqrtf((xmm * xmm) + (ymm * ymm));
  rside = sqrtf((rrot * rrot) + (zmm * zmm));

  /* robotGeometry.cpp:51 - the base rotation is unchanged. */
  rot = asinf(xmm / rrot);

  /* The two shanks cannot be longer than 2 * L between their ends. */
  if (rside > (2.0f * ARM_SHANK_MM))
  {
    rside = 2.0f * ARM_SHANK_MM;
  }

  /* P = -acos(rside / 2L) is the branch that puts the power-on pose at
     low 0 / high -90 and keeps the elbow on the same side as the hardware. */
  p = -acosf(rside / (2.0f * ARM_SHANK_MM));

  /* Q from the direction of the target in the (r, z) plane. */
  q = atan2f(zmm, rrot);

  low  = p + q;
  high = p - q;

  if (out != NULL)
  {
    out->rot  = rot;
    out->low  = low;
    out->high = high;
  }
}
