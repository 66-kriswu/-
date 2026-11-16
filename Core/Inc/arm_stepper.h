/**
  ******************************************************************************
  * @file    arm_stepper.h
  * @brief   Step pulse engine: one hardware timer interrupt, three independent
  *          software pulse generators.
  *
  *          WHY THIS IS NOT THE DMA ENGINE ANY MORE
  *          --------------------------------------
  *          The original port generated the pulses with TIM2/TIM3 PWM plus a
  *          1024 entry ping-pong DMA table.  That design had two problems that
  *          could not be tuned away (handover doc sections 18.10, 21.2, 22.1):
  *
  *            1. Both arm joints had to share TIM3 and its single UPDATE DMA
  *               request, so only one of them could move at a time.  The other
  *               one queued up and then dumped its backlog in one burst - the
  *               "base stops, lower arm jerks, base carries on" symptom.
  *
  *            2. The table holds a FIXED NUMBER of timer periods, so its
  *               duration changes by more than a hundredfold with the step
  *               rate (1024 x 30 us = 31 ms at full speed, 1024 x 5 ms = 5 s at
  *               crawl speed).  A half buffer that is only partly filled plays
  *               its pulses back to back and then stays silent until the next
  *               refill, which is exactly the "starts, pauses for half a second,
  *               continues" behaviour.
  *
  *          This module therefore produces the pulses in software from a single
  *          20 us timer tick (TIM7).  Each axis owns a countdown and a pulse
  *          phase, so:
  *
  *            - all three axes run truly concurrently, no arbitration, no
  *              hand-over, no backlog,
  *            - the interval between pulses is set directly, so a slow move is
  *              slow everywhere instead of bursty,
  *            - the whole queue is at most a handful of pulses deep, so a move
  *              ends the moment the interpolation ends.
  *
  *          Timing accuracy: the tick is 20 us, and the interval is kept in Q8
  *          fixed point, so the AVERAGE period is exact and the only error is a
  *          +-20 us quantisation of individual pulses.  At the fastest step rate
  *          this project uses (about 2.5 kHz) that is 4 % of one period; the
  *          pulse width is one whole tick = 20 us, which is well above the
  *          TB6600 opto-coupler requirement.
  *
  *          The idle CPU cost is zero: the tick interrupt is switched off as
  *          soon as every axis has run out of pulses.
  ******************************************************************************
  */

#ifndef ARM_STEPPER_H
#define ARM_STEPPER_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"          /* GPIO_TypeDef / HAL */
#include "arm_config.h"
#include "arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Number of pulses the motion task may queue ahead of the generator.
  *
  *  This is a safety limit only: the queue is normally 0..3 pulses deep, so the
  *  logical position never runs more than a few pulses ahead of the hardware and
  *  a segment ends within a few milliseconds of the interpolation.  The limit
  *  exists so that a direction rejection cannot let the caller run away. */
#define ARM_STEP_PENDING_MAX             1536u

/*============================== Life cycle =================================*/

/**
  * @brief  Set the STEP pins to their idle level, stop the old PWM/DMA pulse
  *         path and start the 20 us pulse timer.
  * @note   Call after arm_board_init(), which turns the STEP pins into open
  *         drain GPIO outputs, and before the first M17.
  */
void arm_stepper_init(void);

/**
  * @brief  Result of the boot time self test of the pulse timer.
  * @retval true  the tick interrupt is being served, pulses can be generated
  * @retval false the timer never interrupted, so no axis will ever move
  * @note   A failure does NOT stop the firmware; it is reported on the debug
  *         UART so that a silent-dead-board and a dead-pulse-engine can be told
  *         apart.  Check this before blaming the motor wiring.
  */
bool arm_stepper_selftest_ok(void);

/**
  * @brief  Drive every axis into its power-on pose without emitting a pulse.
  * @note   Any pulse still queued is dropped first, because it was counted
  *         against the old frame of reference.  Used at power up and by M210
  *         ("the pose the arm is standing in is home again").
  */
void arm_stepper_set_home_pose(void);

/*================================= Enable ==================================*/

/** M17: energise one axis (the ENA pin is released by the open drain driver). */
void arm_stepper_axis_enable(arm_axis_t axis, bool enable);

/** M17 / M18 for all three axes. */
void arm_stepper_enable_all(bool enable);

/** True when the axis driver is energised. */
bool arm_stepper_is_enabled(arm_axis_t axis);

/*================================= Motion ==================================*/

/**
  * @brief  Queue up to steps pulses on one axis and advance its position.
  * @param  steps  Signed pulse count; negative moves in the opposite direction.
  * @return Number of pulses actually accepted, with the sign of the request
  *         (0 when the queue is full or a direction change is not safe yet).
  * @note   May be called from MotionTask only.  When the driver is disabled the
  *         full request is accepted but no pulse is emitted, which reproduces
  *         the "virtual move while the steppers are off" behaviour of the
  *         original firmware.
  */
int32_t arm_stepper_step(arm_axis_t axis, int32_t steps);

/** Move the logical position without touching the hardware at all. */
void arm_stepper_set_position_steps(arm_axis_t axis, int32_t steps);

/** Current logical step position. */
int32_t arm_stepper_get_position_steps(arm_axis_t axis);

/** Current logical position in radians. */
float arm_stepper_get_position_rad(arm_axis_t axis);

/** Target the axis should reach, in radians (0 when idle). */
float arm_stepper_get_target_rad(arm_axis_t axis);

/** Queue a move that ends at the given absolute angle in radians. */
void arm_stepper_step_to_rad(arm_axis_t axis, float rad);

/** Pulses still queued for one axis (not yet emitted). */
uint32_t arm_stepper_pending(arm_axis_t axis);

/**
  * @brief  Pulse period currently programmed for one axis, in microseconds.
  * @note   Read-only diagnostic: it is the interval the generator will use for
  *         the next pulse, so a period that does not follow the cosine ramp is
  *         visible in the diag line instead of having to be guessed from the
  *         sound.
  */
uint32_t arm_stepper_pulse_us(arm_axis_t axis);

/**
  * @brief  Raw generator state of one axis, for the diagnostic line.
  * @param  pending out: pulses still queued.
  * @param  period_us out: programmed pulse period in microseconds.
  * @param  phase  out: 1 while a pulse is currently asserted on the pin.
  */
void arm_stepper_debug_axis(arm_axis_t axis, uint32_t *pending,
                            uint32_t *period_us, uint32_t *phase);

/**
  * @brief  Number of 20 us ticks served since boot.
  * @note   Diagnostic only.  It stops counting whenever every axis is idle
  *         (the tick interrupt is switched off on purpose), so "tk frozen"
  *         together with "pending not zero" means the generator is stuck.
  */
uint32_t arm_stepper_tick_count(void);

/** True when the axis has neither queued pulses nor a pulse in progress. */
bool arm_stepper_is_idle(arm_axis_t axis);

/** True when every axis is idle. */
bool arm_stepper_all_idle(void);

/*================================= Control =================================*/

/** Abort pulse generation on one axis and leave STEP at its idle level.
  * Safe from any context; used by the limit and emergency stop paths.        */
void arm_stepper_stop(arm_axis_t axis);

/** Abort every axis. */
void arm_stepper_stop_all(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_STEPPER_H */
