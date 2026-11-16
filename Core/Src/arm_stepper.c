/**
  ******************************************************************************
  * @file    arm_stepper.c
  * @brief   Software step pulse generator.  See arm_stepper.h for the contract.
  *
  *          One pulse, two ticks
  *          --------------------
  *          TIM7 interrupts every ARM_PULSE_TICK_US (20 us).  For each axis a
  *          small state machine runs:
  *
  *              phase == 0, pending == 0  -> nothing to do
  *              phase == 0, pending >  0  -> count the pulse interval down;
  *                                           when it expires assert STEP
  *                                           (pin LOW in a common anode
  *                                           wiring) and set phase = 1
  *              phase == 1                -> release STEP (pin HIGH) and clear
  *                                           phase
  *
  *          One pulse therefore occupies exactly two ticks: one asserted and one
  *          released, i.e. the pulse width is 20 us.  The shortest interval is
  *          also two ticks (40 us = 25 kHz), far above anything the kinematics
  *          ask for; the longest is 100 ms (10 steps/s).
  *
  *          Why the interval is kept in Q8
  *          ------------------------------
  *          The wanted interval is an arbitrary number of ticks (for example
  *          29.41 ticks for 1700 steps/s).  Rounding it to whole ticks would
  *          quantise the speed by up to 3 % and, worse, the quantisation error
  *          would be constant, so a commanded 1700 steps/s would always come out
  *          as 1724.  The remainder is carried in the low 8 bits instead, which
  *          makes the AVERAGE interval exact and leaves only a +-1 tick
  *          alternation from pulse to pulse.
  *
  *          Where the commanded rate comes from
  *          -----------------------------------
  *          arm_stepper_step() is called once per millisecond by MotionTask for
  *          every axis - including the ticks where no step is due.  It sums the
  *          requested steps over ARM_STEP_RATE_WINDOW_TICKS (128 ms) and divides
  *          by the tick count, so `rate1000` really is "steps per second".
  *          Averaging over a window (instead of using the instantaneous 0/1
  *          steps) is what keeps the pulse period steady: a single request is
  *          only 1/128 of the information.
  *
  *          A rate of zero means the interpolator has stopped asking for steps
  *          while pulses are still queued (the tail of a segment).  Draining it
  *          at the last commanded speed would crawl, so the queue is flushed at
  *          ARM_PULSE_FLUSH_RATE1000 instead - a few milliseconds, not seconds.
  ******************************************************************************
  */

#include "arm_stepper.h"

#include <string.h>

#include "main.h"
#include "cmsis_os.h"

#include "arm_us.h"
#include "arm_interpolation.h"   /* arm_interp_is_finished(), see the tick */

/*============================================================================*/
/* Pulse timer                                                                */
/*============================================================================*/

/* TIM7 is used because it has no output pins at all and nothing else in this
   project owns it:
     TIM2 / TIM3 : PWM pulse timers of the old DMA engine, now silenced
     TIM5        : microsecond time base (arm_us.c)
     TIM6        : HAL tick (stm32f4xx_hal_timebase_tim.c)
   TIM7 is a plain basic timer, so the pulse engine cannot disturb a pin.

     PSC = 84 MHz / 1 MHz - 1 = 83   -> one timer tick == 1 us
     ARR = ARM_PULSE_TICK_US - 1     -> one interrupt  == ARM_PULSE_TICK_US   */
#define ARM_PULSE_TIMER                 TIM7
#define ARM_PULSE_TIMER_CLK_HZ          84000000u
#define ARM_PULSE_PSC                   ((ARM_PULSE_TIMER_CLK_HZ / 1000000u) - 1u)
#define ARM_PULSE_ARR                   (ARM_PULSE_TICK_US - 1u)

/** Interrupt priority.  5 is the "pulse" level of handover doc section 4.6,
  * the same level TIM2/TIM3/DMA1_Stream2/Stream7 uses.  This ISR calls no
  * FreeRTOS API, so sitting at configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY is
  * correct and it can never be preempted by the kernel. */
#define ARM_PULSE_IRQ_PRIORITY          5u

/*============================== Private defines =============================*/

/** Fixed point scale of the pulse interval: one tick == 256 units.          */
#define ARM_PULSE_Q8                    256u

/** Two ticks per pulse: one asserted, one released.                           */
#define ARM_PULSE_PHASE_TICKS           2u

/** Longest interval between pulse starts: 5000 ticks = 100 ms (10 steps/s).  */
#define ARM_PULSE_MAX_TICKS             5000u

/** Highest rate the generator is programmed for.  25000 steps/s is 2 ticks per
  * pulse, i.e. the hardware limit of this scheme.                             */
#define ARM_PULSE_MAX_RATE1000          25000u

/** Rate used to flush a queue left over after the interpolator stopped.
  * 2000 steps/s is a 0.5 ms pulse period: the last few pulses of a segment are
  * gone in milliseconds instead of crawling for seconds.                      */
#define ARM_PULSE_FLUSH_RATE1000        2000u

/** DIR setup time before the first STEP edge after a direction change.
  * The TB6600 input is an opto-coupler; its data sheet asks for a few
  * microseconds between the DIR edge and the STEP edge.  3 ticks = 60 us.     */
#define ARM_PULSE_DIR_SETUP_TICKS       3u

/** Length of the sliding window that measures the commanded step rate.
  *
  * MotionTask ticks once per millisecond, so 128 ticks is 128 ms.  The window
  * has to be that long because the task asks for WHOLE steps: at 150 steps/s a
  * 32 tick window would hold 4 or 5 steps and the measured rate would alternate
  * by 25 %, which is audible.  With 128 ticks it holds 19 or 20, and the
  * remainder carried in `rate_frac` makes even that difference disappear.
  * The price is 128 ms of lag on the cosine ramp of a multi-second segment,
  * which is not visible on the machine.                                       */
#define ARM_STEP_RATE_WINDOW_TICKS      128u

/*============================== Private types ===============================*/

typedef struct
{
  GPIO_TypeDef *port;             /**< STEP port (GPIOA)                       */
  uint16_t      pin;              /**< STEP pin (PA0 / PA7 / PA6)              */

  volatile uint32_t pending;      /**< pulses queued, not yet emitted          */
  volatile uint32_t down_q8;      /**< ticks until the next pulse, Q8          */
  volatile uint8_t  phase;        /**< 1 while the STEP pin is asserted        */

  uint32_t      period_q8;        /**< interval between pulse starts, Q8 ticks */
  uint32_t      rate_frac;        /**< division carry of the rate window       */
  uint32_t      rate_steps;       /**< steps requested inside the window       */
  uint32_t      rate_ticks;       /**< motion ticks inside the window          */
  uint32_t      rate1000;         /**< commanded speed, steps/s                */

  int32_t       position;         /**< logical step position                   */
  int32_t       target;           /**< commanded step position                 */
  uint8_t       enabled;          /**< driver energised                        */
  uint8_t       dir_forward;      /**< level currently on the DIR pin          */
  uint8_t       dir_known;        /**< DIR has been written at least once      */
} arm_axis_engine_t;

/*============================= Private state ===============================*/

static arm_axis_engine_t s_axis[ARM_AXIS_COUNT];

/** 1 while the 20 us tick interrupt is enabled.                              */
static volatile uint8_t  s_tick_running = 0u;

/** Ticks served since boot, for the diagnostic line.                         */
static volatile uint32_t s_tick_count = 0u;

/** 1 while the interpolator still has a segment to run.
  *
  * Refreshed by arm_stepper_step() (called three times per MotionTask cycle,
  * right after arm_interp_update()), so the pulse interrupt can read it as a
  * plain flag instead of calling into another module every 20 us.  It keeps the
  * interval clock running across the millisecond gaps between two steps.      */
static volatile uint8_t  s_interp_running = 0u;

/** Result of the boot time self test.                                        */
static volatile uint8_t  s_selftest_ok = 0u;

/*=========================== Internal prototypes ===========================*/

static void     arm_stepper_tick(void);
static void     arm_stepper_tick_start(void);
static void     arm_stepper_tick_sync(void);
static bool     arm_stepper_busy(void);
static void     arm_stepper_rate_window(arm_axis_engine_t *e, uint32_t mag);
static void     arm_stepper_update_period(arm_axis_engine_t *e);
static void     arm_stepper_pin_idle(const arm_axis_engine_t *e);
static void     arm_stepper_pin_active(const arm_axis_engine_t *e);
static void     arm_stepper_selftest(void);
static bool     arm_stepper_dir_level(arm_axis_t axis, bool forward);
static void     arm_stepper_set_dir(arm_axis_t axis, bool forward);

/*============================================================================*/
/* STEP pin levels                                                            */
/*============================================================================*/

/**
  * @brief  Put STEP at the level that means "no pulse" for the wiring in use.
  * @note   Common anode (ARM_DRIVER_COMMON_ANODE == 1): the opto-coupler cathode
  *         faces the MCU and the pin is open drain, so releasing it (writing 1,
  *         i.e. high impedance pulled up by the driver side) is the idle state.
  *         Common cathode: the pin drives the opto anode, so idle is LOW.
  *         Both are written through BSRR, which is a single atomic store and
  *         therefore safe to use inside the timer interrupt.
  */
static inline void arm_stepper_pin_idle(const arm_axis_engine_t *e)
{
#if (ARM_DRIVER_COMMON_ANODE == 1)
  e->port->BSRR = (uint32_t)e->pin;                       /* release -> HIGH  */
#else
  e->port->BSRR = (uint32_t)e->pin << 16u;                /* drive   -> LOW   */
#endif
}

/** Put STEP at the level that asserts a pulse (the opposite of the idle one). */
static inline void arm_stepper_pin_active(const arm_axis_engine_t *e)
{
#if (ARM_DRIVER_COMMON_ANODE == 1)
  e->port->BSRR = (uint32_t)e->pin << 16u;                /* sink    -> LOW   */
#else
  e->port->BSRR = (uint32_t)e->pin;                       /* drive   -> HIGH  */
#endif
}

/*============================================================================*/
/* Pulse timer and interrupt                                                  */
/*============================================================================*/

/**
  * @brief  TIM7 update, every ARM_PULSE_TICK_US.
  * @note   Lives here rather than in stm32f4xx_it.c because GENERATE CODE
  *         rewrites that file outside the USER CODE blocks.  A strong symbol in
  *         a non-generated file cannot be lost (same reasoning as
  *         TIM5_IRQHandler in arm_us.c, handover doc section 9.3).
  */
void TIM7_IRQHandler(void)
{
  if ((ARM_PULSE_TIMER->SR & TIM_SR_UIF) == 0u)
  {
    return;
  }

  /* SR is rc_w0: writing 0 clears a flag, writing 1 leaves it alone. */
  ARM_PULSE_TIMER->SR = (uint16_t)~TIM_SR_UIF;

  arm_stepper_tick();
}

/**
  * @brief  One pass of the three pulse generators.
  * @note   No division, no floating point, no library call: it runs every 20 us.
  *
  *         The interval clock is advanced even when the queue of an axis is
  *         empty, because the interpolator feeds whole steps only once per
  *         millisecond.  Restarting the interval at every arrival would turn a
  *         steady 2 kHz into an alternating 1 kHz / 3 kHz pattern - the
  *         "burst" that this whole rewrite exists to remove.  The clock
  *         therefore keeps running for the whole segment and only stops when
  *         the interpolator has finished AND every queue is empty.
  */
static void arm_stepper_tick(void)
{
  uint32_t busy = 0u;

  s_tick_count++;

  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    arm_axis_engine_t *e = &s_axis[i];

    if (e->phase != 0u)
    {
      /* The pulse has been asserted for one whole tick: release it.  The
         pending count was already decremented when the pulse started, and this
         tick deliberately does not advance the interval (that is where the
         ARM_PULSE_PHASE_TICKS term of the period comes from). */
      arm_stepper_pin_idle(e);
      e->phase = 0u;
    }
    else
    {
      if (e->down_q8 >= ARM_PULSE_Q8)
      {
        e->down_q8 -= ARM_PULSE_Q8;
      }

      if ((e->pending != 0u) && (e->down_q8 < ARM_PULSE_Q8))
      {
        /* Interval expired: start the pulse.  The Q8 remainder carried in
           down_q8 survives the addition, so the average period is exact. */
        arm_stepper_pin_active(e);
        e->phase = 1u;
        e->pending--;
        e->down_q8 += e->period_q8;
      }
    }

    if ((e->pending != 0u) || (e->phase != 0u))
    {
      busy = 1u;
    }
  }

  /* Nothing queued, no pulse in progress and no trajectory left to run: stop
     paying for the interrupt.  It is restarted by the next arm_stepper_step().
     down_q8 is cleared here as well, so the first pulse of the next segment
     cannot be delayed by a leftover interval of up to 100 ms. */
  if ((busy == 0u) && (s_interp_running == 0u))
  {
    for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
    {
      s_axis[i].down_q8 = 0u;
    }

    s_tick_running = 0u;
    ARM_PULSE_TIMER->DIER &= ~TIM_DIER_UIE;
  }
}

/**
  * @brief  [学习标注] 打开 TIM7 更新中断；先清掉可能积压的 UIF，避免刚开就多进一拍。
  */
static void arm_stepper_tick_start(void)
{
  if (s_tick_running != 0u)
  {
    return;
  }

  s_tick_running = 1u;

  /* Drop a flag that accumulated while the interrupt was off, otherwise the
     first tick would be served immediately and again 20 us later. */
  ARM_PULSE_TIMER->SR = (uint16_t)~TIM_SR_UIF;
  ARM_PULSE_TIMER->DIER |= TIM_DIER_UIE;
}

/** Match the tick interrupt to the current workload (task/stop context only). */
static void arm_stepper_tick_sync(void)
{
  if (arm_stepper_busy())
  {
    arm_stepper_tick_start();
  }
  else
  {
    s_tick_running = 0u;
    ARM_PULSE_TIMER->DIER &= ~TIM_DIER_UIE;
  }
}

/**
  * @brief  [学习标注] 三轴是否还有待发脉冲或正在拉低。
  */
static bool arm_stepper_busy(void)
{
  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    if ((s_axis[i].pending != 0u) || (s_axis[i].phase != 0u))
    {
      return true;
    }
  }

  return false;
}

/*============================================================================*/
/* Initialisation                                                             */
/*============================================================================*/

/**
  * @brief  [学习标注] 配 STEP 引脚、显式停掉旧 TIM2/3 PWM 与 DMA 流、安装 TIM7(20 µs)、跑上电
  *         自检。
  */
void arm_stepper_init(void)
{
  arm_axis_engine_t *e;

  /*---- base rotation: RAMPS Z_STEP_PIN -> PA0 ----*/
  e = &s_axis[ARM_AXIS_ROT];
  memset(e, 0, sizeof(*e));
  e->port = GPIOA;
  e->pin  = GPIO_PIN_0;

  /*---- lower arm (the big/lower joint): RAMPS Y_STEP_PIN -> PA7 ----*/
  e = &s_axis[ARM_AXIS_LOW];
  memset(e, 0, sizeof(*e));
  e->port = GPIOA;
  e->pin  = GPIO_PIN_7;

  /*---- upper arm: RAMPS X_STEP_PIN -> PA6 ----*/
  e = &s_axis[ARM_AXIS_HIGH];
  memset(e, 0, sizeof(*e));
  e->port = GPIOA;
  e->pin  = GPIO_PIN_6;

  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    arm_stepper_pin_idle(&s_axis[i]);
  }

  /* The pulse path of the old DMA engine is retired here.  TIM2 and TIM3 are
     still configured by the generated MX_TIM2_Init()/MX_TIM3_Init() and their
     DMA streams are still initialised by HAL_TIM_Base_MspInit(), so they are
     silenced explicitly instead of being left to generate UPDATE requests that
     nobody serves any more. */
  TIM2->CR1  &= ~TIM_CR1_CEN;
  TIM2->DIER &= ~TIM_DIER_UDE;
  TIM2->CCER &= ~(TIM_CCER_CC1E);
  TIM3->CR1  &= ~TIM_CR1_CEN;
  TIM3->DIER &= ~TIM_DIER_UDE;
  TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);

  DMA1_Stream7->CR &= ~DMA_SxCR_EN;
  DMA1_Stream7->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  DMA1_Stream7->NDTR = 0u;
  HAL_NVIC_DisableIRQ(DMA1_Stream7_IRQn);

  DMA1_Stream2->CR &= ~DMA_SxCR_EN;
  DMA1_Stream2->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  DMA1_Stream2->NDTR = 0u;
  HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);

  /*---- pulse timer: 1 MHz counter, 20 us update, interrupt off for now ----*/
  __HAL_RCC_TIM7_CLK_ENABLE();

  ARM_PULSE_TIMER->CR1  = 0u;
  ARM_PULSE_TIMER->PSC  = ARM_PULSE_PSC;
  ARM_PULSE_TIMER->ARR  = ARM_PULSE_ARR;
  ARM_PULSE_TIMER->DIER = 0u;
  ARM_PULSE_TIMER->EGR  = TIM_EGR_UG;      /* load PSC/ARR immediately */
  ARM_PULSE_TIMER->SR   = (uint16_t)~TIM_SR_UIF;
  ARM_PULSE_TIMER->CR1  = TIM_CR1_CEN;     /* free running from now on */

  HAL_NVIC_SetPriority(TIM7_IRQn, ARM_PULSE_IRQ_PRIORITY, 0u);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

  s_tick_count = 0u;

  arm_stepper_selftest();
}

/**
  * @brief  Prove that the tick interrupt is actually served.
  * @note   A dead pulse engine must be visible, never a frozen firmware: the
  *         result only goes to the debug UART (see arm_app.c).  Freezing here
  *         would hide the failure behind a stopped heartbeat, which is exactly
  *         the trap documented in handover doc section 9.6.
  */
static void arm_stepper_selftest(void)
{
  uint32_t before = s_tick_count;
  uint32_t t0     = arm_us_micros();

  ARM_PULSE_TIMER->SR   = (uint16_t)~TIM_SR_UIF;
  ARM_PULSE_TIMER->DIER |= TIM_DIER_UIE;

  /* 500 us is 25 ticks; the wait is bounded even if the timer never fires. */
  while ((uint32_t)(arm_us_micros() - t0) < 500u)
  {
    /* spin */
  }

  s_selftest_ok = (uint8_t)((s_tick_count != before) ? 1u : 0u);

  if (arm_stepper_busy() == false)
  {
    s_tick_running = 0u;
    ARM_PULSE_TIMER->DIER &= ~TIM_DIER_UIE;
  }
}

/**
  * @brief  [学习标注] 返回上电自检结果（TIM7 中断是否真的被服务）。
  */
bool arm_stepper_selftest_ok(void)
{
  return (s_selftest_ok != 0u);
}

/*============================================================================*/
/* Step rate  ->  pulse interval                                              */
/*============================================================================*/

/**
  * @brief  Turn the measured rate into a pulse interval.
  *
  *    rate1000            steps per second
  *    interval_us         = 1e6 / rate1000
  *    interval_ticks      = interval_us / ARM_PULSE_TICK_US
  *    interval_q8         = 256 * 1e6 / (rate1000 * ARM_PULSE_TICK_US)
  *
  * The two ticks that the pulse itself occupies are subtracted, because
  * down_q8 only starts counting after the pulse has been released.  The result
  * is clamped to [0, ARM_PULSE_MAX_TICKS]: 0 is 25 kHz, 5000 is 10 steps/s.
  */
static void arm_stepper_update_period(arm_axis_engine_t *e)
{
  uint32_t rate = e->rate1000;
  uint32_t q8;

  /* Two situations call for the flush rate instead of the measured one:
   *
   *   1. the window holds no step at all, i.e. this axis is being fed slower
   *      than a 128 tick window can resolve, and
   *   2. the interpolator has FINISHED while pulses are still queued.
   *
   * Case 2 is the "tail" that handover document section 22.2 could not get rid
   * of: the measured rate decays to a few tens of steps per second as the
   * cosine ramp runs out, so the last 8..20 pulses of every segment were played
   * at 10..40 steps/s - a visible creep of up to 200 ms after the arm had
   * already arrived.  Measured on 2026-11-05: `m=11` with `us=25641` (39
   * steps/s) and 8 pulses left.
   *
   * The last commanded rate is kept when it is FASTER than the flush rate, so a
   * fast move does not suddenly crawl through its last few pulses; only a move
   * that ended slowly is sped up to ARM_PULSE_FLUSH_RATE1000. */
  if (s_interp_running == 0u)
  {
    if (rate < ARM_PULSE_FLUSH_RATE1000)
    {
      rate = ARM_PULSE_FLUSH_RATE1000;
    }
  }
  else if (rate == 0u)
  {
    rate = ARM_PULSE_FLUSH_RATE1000;
  }

  if (rate > ARM_PULSE_MAX_RATE1000)
  {
    rate = ARM_PULSE_MAX_RATE1000;
  }

  q8 = (256000000u / (rate * ARM_PULSE_TICK_US));

  if (q8 > (ARM_PULSE_PHASE_TICKS * ARM_PULSE_Q8))
  {
    q8 -= (ARM_PULSE_PHASE_TICKS * ARM_PULSE_Q8);
  }
  else
  {
    q8 = 0u;                                  /* 2 ticks per pulse: 25 kHz    */
  }

  if (q8 > (ARM_PULSE_MAX_TICKS * ARM_PULSE_Q8))
  {
    q8 = (ARM_PULSE_MAX_TICKS * ARM_PULSE_Q8);
  }

  e->period_q8 = q8;
}

/**
  * @brief  Advance the rate measurement window by one motion tick.
  * @param  mag  Number of steps that this tick asked for (0 when none is due).
  * @note   Called for every axis on every MotionTask cycle, including the ticks
  *         where nothing is requested, because the window is a time average.
  */
static void arm_stepper_rate_window(arm_axis_engine_t *e, uint32_t mag)
{
  e->rate_steps += mag;
  e->rate_ticks += 1u;

  if (e->rate_ticks < ARM_STEP_RATE_WINDOW_TICKS)
  {
    return;
  }

  {
    uint32_t num = (e->rate_steps * 1000u) + e->rate_frac;

    e->rate1000   = num / e->rate_ticks;
    e->rate_frac  = num % e->rate_ticks;
    e->rate_steps = 0u;
    e->rate_ticks = 0u;
  }

  arm_stepper_update_period(e);
}

/*============================================================================*/
/* Enable / disable                                                           */
/*============================================================================*/

/**
  * @brief  [学习标注] M17/M18 单轴使能；使能电平由 ARM_ENABLE_ACTIVE_LOW 决定。
  */
void arm_stepper_axis_enable(arm_axis_t axis, bool enable)
{
  arm_axis_engine_t *e = &s_axis[axis];
  GPIO_TypeDef *port = NULL;
  uint16_t pin = 0u;

  switch (axis)
  {
    case ARM_AXIS_ROT:  port = Z_ENABLE_GPIO_Port; pin = Z_ENABLE_Pin;  break;
    case ARM_AXIS_LOW:  port = Y_ENABLE_GPIO_Port; pin = Y_ENABLE_Pin;  break;
    case ARM_AXIS_HIGH: port = X_ENABLE_GPIO_Port; pin = X_ENABLE_Pin;  break;
    default: return;
  }

  /* TB6600 ENA.  The level that energises the driver is the single switch
     ARM_ENABLE_ACTIVE_LOW, so the wiring choice and the module revision can be
     corrected independently of the STEP and DIR logic. */
#if (ARM_ENABLE_ACTIVE_LOW == 1)
  HAL_GPIO_WritePin(port, pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(port, pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
  e->enabled = (uint8_t)(enable ? 1u : 0u);
}

/**
  * @brief  [学习标注] 三轴一起使能/失能。
  */
void arm_stepper_enable_all(bool enable)
{
  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    arm_stepper_axis_enable((arm_axis_t)i, enable);
  }
}

/**
  * @brief  [学习标注] 查询某轴驱动器是否使能。
  */
bool arm_stepper_is_enabled(arm_axis_t axis)
{
  return (s_axis[axis].enabled != 0u);
}

/*============================================================================*/
/* Direction                                                                  */
/*============================================================================*/

/* Hardware polarity of the DIR line, one entry per axis.
 *
 * The base rotation follows the kinematic convention directly, but the two arm
 * joints come out mirrored with this common anode / open drain wiring: measured
 * on the machine on 2026-11-04, G1 Y120 -> Y150 (a command to extend the reach)
 * made the end effector come CLOSER instead of further away, and the upper arm
 * rotated backwards.  Flipping only these two axes restores "positive step =
 * arm opens", while the base keeps the direction that was confirmed correct by
 * counting four 30 degree rotations. */
static bool arm_stepper_dir_level(arm_axis_t axis, bool forward)
{
  static const uint8_t invert[ARM_AXIS_COUNT] =
  {
    0u,   /* ARM_AXIS_ROT  : confirmed correct */
    1u,   /* ARM_AXIS_LOW  : mirrored         */
    1u    /* ARM_AXIS_HIGH : mirrored         */
  };

  if ((uint32_t)axis >= (uint32_t)ARM_AXIS_COUNT)
  {
    return forward;
  }

  return (invert[axis] != 0u) ? (!forward) : forward;
}

/** Convert an axis into the direction pin it drives. */
/**
  * @brief  [学习标注] 写 DIR：先过逐轴取反表，再按共阳极/共阴极决定电平。
  */
static void arm_stepper_set_dir(arm_axis_t axis, bool forward)
{
  GPIO_TypeDef *port = NULL;
  uint16_t pin = 0u;

  switch (axis)
  {
    case ARM_AXIS_ROT:  port = Z_DIR_GPIO_Port; pin = Z_DIR_Pin;  break;
    case ARM_AXIS_LOW:  port = Y_DIR_GPIO_Port; pin = Y_DIR_Pin;  break;
    case ARM_AXIS_HIGH: port = X_DIR_GPIO_Port; pin = X_DIR_Pin;  break;
    default: return;
  }

  forward = arm_stepper_dir_level(axis, forward);

  /* TB6600 DIR.
     With a common anode input the opto-coupler cathode faces the MCU, so the
     logical direction is mirrored: forward is LOW on a common cathode wiring
     and HIGH on a common anode wiring.  The original firmware drove
     digitalWrite(dirPin, LOW) to count up (RampsStepper.cpp:68-84), and the
     sign convention of the whole kinematics depends on keeping that mapping. */
#if (ARM_DRIVER_COMMON_ANODE == 1)
  HAL_GPIO_WritePin(port, pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(port, pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

/*============================================================================*/
/* Motion                                                                     */
/*============================================================================*/

/**
  * @brief  [学习标注] 核心入口：测速窗口 → 插补结束沿 → 失能时虚拟运动 → 方向守卫/回滚 →
  *         DIR 建立时间 → 累计 pending → 开 tick 中断。
  * @note   必须每 1 ms 对每个轴调用，delta=0 也要调，否则测速偏快 6 倍。
  */
int32_t arm_stepper_step(arm_axis_t axis, int32_t steps)
{
  arm_axis_engine_t *e = &s_axis[axis];
  uint32_t mag = (uint32_t)((steps < 0) ? -steps : steps);
  uint32_t want;
  uint32_t primask;
  bool     forward;
  uint8_t  level;

  /* Speed measurement first: it must advance even on the ticks where no step
     is due, otherwise a slow move looks 6x faster than it is (section 20.5). */
  arm_stepper_rate_window(e, mag);

  /* Tells the pulse interrupt whether a trajectory is still running, so the
     interval clock survives the sub-millisecond gaps between two steps.  The
     falling edge is also the moment the queued pulses have to be re-programmed
     for the tail: the measured rate is about to decay to nothing and the last
     few pulses would otherwise creep out over a couple of hundred milliseconds
     (see arm_stepper_update_period()). */
  {
    uint8_t running = (arm_interp_is_finished() ? 0u : 1u);

    if (running != s_interp_running)
    {
      s_interp_running = running;

      if (running == 0u)
      {
        for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
        {
          arm_stepper_update_period(&s_axis[i]);
        }
      }
    }
  }

  if (steps == 0)
  {
    e->target = e->position;
    return 0;
  }

  /* A disabled driver cannot generate torque, so the position is simply moved
     without emitting anything.  This is what makes "move while the steppers
     are off" behave like the original firmware, where the loop kept stepping
     and only the enable line was low. */
  if (e->enabled == 0u)
  {
    e->position += steps;
    e->target    = e->position;
    return steps;
  }

  forward = (steps > 0);
  level   = (uint8_t)(arm_stepper_dir_level(axis, forward) ? 1u : 0u);

  /* The DIR pin and the pending queue are shared with the pulse interrupt, so
     the decision and the update have to be one atomic step.  The window is a
     handful of instructions once per millisecond, so PRIMASK is fine here. */
  primask = __get_PRIMASK();
  __disable_irq();

  if ((e->pending != 0u) && (e->dir_forward != level))
  {
    /* A direction change while pulses are queued would play those pulses in the
       NEW direction - that is how the upper arm was driven into its mechanical
       stop on 2026-11-04.  With this generator the queue is only a handful of
       pulses deep and none of them has been emitted yet, so they can be thrown
       away instead of waiting for them: undo the position they were counted
       into, and the model stays exact.  Only a pulse that is asserted right now
       (phase) cannot be undone, and that lasts one 20 us tick. */
    if (e->phase != 0u)
    {
      __set_PRIMASK(primask);
      return 0;                                /* retried on the next tick   */
    }

    if (e->dir_forward != 0u)
    {
      e->position -= (int32_t)e->pending;
    }
    else
    {
      e->position += (int32_t)e->pending;
    }
    e->pending = 0u;
  }

  if ((e->dir_known == 0u) || (e->dir_forward != level))
  {
    arm_stepper_set_dir(axis, forward);
    e->dir_forward = level;
    e->dir_known   = 1u;

    /* Hold the first pulse of the new direction back far enough for the
       driver's opto-coupler to settle (TB6600 DIR setup time). */
    if (e->down_q8 < (ARM_PULSE_DIR_SETUP_TICKS * ARM_PULSE_Q8))
    {
      e->down_q8 = (ARM_PULSE_DIR_SETUP_TICKS * ARM_PULSE_Q8);
    }
  }

  want = mag;

  if (want > (ARM_STEP_PENDING_MAX - e->pending))
  {
    want = ARM_STEP_PENDING_MAX - e->pending;
  }

  if (want == 0u)
  {
    __set_PRIMASK(primask);
    return 0;
  }

  /* Count every pulse as generated straight away, then let the interrupt drain
     the pending count.  The travel distance stays exact even when the queue is
     momentarily full, because the caller retries the remainder next tick. */
  e->pending  += want;
  e->position += (steps < 0) ? -(int32_t)want : (int32_t)want;
  e->target    = e->position;

  arm_stepper_tick_start();

  __set_PRIMASK(primask);

  /* Return the accepted amount with the SIGN OF THE REQUEST: the caller
     compares it against its own signed delta to count rejected steps. */
  return (steps < 0) ? -(int32_t)want : (int32_t)want;
}

/**
  * @brief  [学习标注] 直接改逻辑步数/目标（不动硬件）；上电与 M210 用。
  */
void arm_stepper_set_position_steps(arm_axis_t axis, int32_t steps)
{
  s_axis[axis].position = steps;
  s_axis[axis].target   = steps;
}

/**
  * @brief  [学习标注] 读逻辑步数（模型值，不等于物理位置）。
  */
int32_t arm_stepper_get_position_steps(arm_axis_t axis)
{
  return s_axis[axis].position;
}

/**
  * @brief  [学习标注] 读逻辑位置（弧度）。
  */
float arm_stepper_get_position_rad(arm_axis_t axis)
{
  return (float)s_axis[axis].position / ARM_RAD_TO_STEP;
}

/**
  * @brief  [学习标注] 读最后一次目标（弧度，空闲时等于当前位置）。
  */
float arm_stepper_get_target_rad(arm_axis_t axis)
{
  return (float)s_axis[axis].target / ARM_RAD_TO_STEP;
}

/**
  * @brief  [学习标注] 按弧度目标算 delta 并下发一步（内部调用 step）。
  */
void arm_stepper_step_to_rad(arm_axis_t axis, float rad)
{
  int32_t target = (int32_t)(rad * ARM_RAD_TO_STEP);
  (void)arm_stepper_step(axis, target - s_axis[axis].position);
}

/**
  * @brief  [学习标注] 该轴还欠多少脉冲（diag 的 pend）。
  */
uint32_t arm_stepper_pending(arm_axis_t axis)
{
  return s_axis[axis].pending;
}

/**
  * @brief  [学习标注] 当前编程脉冲周期（µs，含脉冲自身 2 拍；diag 的 us）。
  */
uint32_t arm_stepper_pulse_us(arm_axis_t axis)
{
  const arm_axis_engine_t *e = &s_axis[axis];

  /* Interval = countdown + the two ticks the pulse itself occupies. */
  return ((e->period_q8 + (ARM_PULSE_PHASE_TICKS * ARM_PULSE_Q8)) *
          ARM_PULSE_TICK_US) / ARM_PULSE_Q8;
}

/**
  * @brief  [学习标注] 一次取出 pending / period_us / phase（指针可为 NULL），给 diag 用。
  */
void arm_stepper_debug_axis(arm_axis_t axis, uint32_t *pending,
                            uint32_t *period_us, uint32_t *phase)
{
  if (pending != NULL)
  {
    *pending = s_axis[axis].pending;
  }
  if (period_us != NULL)
  {
    *period_us = arm_stepper_pulse_us(axis);
  }
  if (phase != NULL)
  {
    *phase = (uint32_t)s_axis[axis].phase;
  }
}

/**
  * @brief  [学习标注] 已服务的 20 µs tick 数（diag 的 tk，唯一的物理证据）。
  */
uint32_t arm_stepper_tick_count(void)
{
  return s_tick_count;
}

/**
  * @brief  [学习标注] 该轴既无队列也无正在拉低的脉冲。
  */
bool arm_stepper_is_idle(arm_axis_t axis)
{
  const arm_axis_engine_t *e = &s_axis[axis];

  return ((e->pending == 0u) && (e->phase == 0u));
}

/**
  * @brief  [学习标注] 三轴都空闲（段结束判据之一）。
  */
bool arm_stepper_all_idle(void)
{
  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    if (arm_stepper_is_idle((arm_axis_t)i) == false)
    {
      return false;
    }
  }

  return true;
}

/*============================================================================*/
/* Emergency stop                                                             */
/*============================================================================*/

/**
  * @brief  [学习标注] 停单轴：清队列/相位/倒计时并拉回空闲电平；无活干时关 tick 中断。
  */
void arm_stepper_stop(arm_axis_t axis)
{
  arm_axis_engine_t *e = &s_axis[axis];
  uint32_t primask = __get_PRIMASK();

  /* Block interrupts so the pulse interrupt cannot start a new pulse between
     the abort and the bookkeeping. */
  __disable_irq();

  e->pending = 0u;
  e->phase   = 0u;
  e->down_q8 = 0u;

  arm_stepper_pin_idle(e);

  __set_PRIMASK(primask);

  /* Switch the tick off once nothing is left anywhere. */
  arm_stepper_tick_sync();
}

/**
  * @brief  [学习标注] 停三轴（急停、故障中止、M18 用）。
  */
void arm_stepper_stop_all(void)
{
  for (uint32_t i = 0u; i < (uint32_t)ARM_AXIS_COUNT; i++)
  {
    arm_stepper_stop((arm_axis_t)i);
  }
}

/*============================================================================*/
/* Power-on pose                                                              */
/*============================================================================*/

/**
  * @brief  [学习标注] 归零：先丢队列（旧坐标系的脉冲作废），再把三轴计数设成 home（rot 0 / low
  *         0 / high −90°）。
  */
void arm_stepper_set_home_pose(void)
{
  /* Drop whatever is still queued before the frame of reference is replaced.
     At power up there is nothing to drop; M210 ("the pose the arm is standing
     in is home again") needs it, because those pulses were counted against the
     old frame and releasing them afterwards would move the arm by an amount
     that no longer appears anywhere in the model. */
  arm_stepper_stop_all();

  /* robotArm.ino:60-63 - rotation 0, lower arm 0, upper arm 90 degrees. */
  arm_stepper_set_position_steps(ARM_AXIS_ROT,  0);
  arm_stepper_set_position_steps(ARM_AXIS_LOW,  0);
  arm_stepper_set_position_steps(ARM_AXIS_HIGH,
                                 (int32_t)(ARM_HOME_HIGH_RAD * ARM_RAD_TO_STEP));
}
