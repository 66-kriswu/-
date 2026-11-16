/**
  ******************************************************************************
  * @file    arm_us.c
  * @brief   Microsecond time base on TIM5, replacing the Arduino micros().
  *
  *          Handover doc section 4.3 asks for TIM5 with PSC = 83 and
  *          ARR = 0xFFFFFFFF, i.e. a free running 1 MHz / 32 bit counter.
  *          TIM5 is NOT present in RobotArm_F407.ioc, so it is installed here
  *          at run time instead.  No pin, no DMA and no interrupt is used by
  *          the counter itself.
  *
  *          The 32 bit counter wraps every 2^32 us = 71.58 minutes, exactly
  *          like the original firmware.  micros32() reproduces that wrap so
  *          the ported interpolation math behaves identically.  An overflow
  *          interrupt keeps a 64 bit extension up to date so that timeouts
  *          inside this firmware can never be confused by the wrap.
  ******************************************************************************
  */

#include "arm_us.h"

#include "main.h"
#include "cmsis_os.h"

/*============================================================================*/
/* Private state                                                              */
/*============================================================================*/

static TIM_HandleTypeDef s_htim5;

static volatile uint32_t s_us_high  = 0u;   /* upper 32 bits of the us counter */
static uint32_t          s_us_epoch = 0u;   /* CNT value at the last overflow  */
static volatile uint8_t  s_ready    = 0u;

/*============================================================================*/
/* Internal: read the raw 32 bit counter                                      */
/*============================================================================*/

/**
  * @brief  [学习标注] 读取 TIM5 计数器原始值（1 tick = 1 µs；32 位，约 71.6 分钟回绕）。
  */
static inline uint32_t arm_us_tim_read(void)
{
  return __HAL_TIM_GET_COUNTER(&s_htim5);
}

/*============================================================================*/
/* Initialisation                                                             */
/*============================================================================*/

/**
  * @brief  [学习标注] 运行时安装 TIM5：PSC=83 → 1 MHz、ARR=0xFFFFFFFF 自由运行，并开溢出中断。
  * @note   TIM5 不在 .ioc 里，所以在这里装（见学习文档 §3.4）。
  */
void arm_us_init(void)
{
  TIM_ClockConfigTypeDef clock_cfg = {0};

  if (s_ready != 0u)
  {
    return;
  }

  /* TIM5 is a 32 bit general purpose timer on APB1.  With APB1 timer clock
     at 84 MHz, PSC = 83 gives an exact 1 MHz tick, so 1 tick == 1 us.
     This is the same PSC value the handover document specifies for TIM2/3. */
  __HAL_RCC_TIM5_CLK_ENABLE();

  s_htim5.Instance               = TIM5;
  s_htim5.Init.Prescaler         = 83u;
  s_htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim5.Init.Period            = 0xFFFFFFFFu;
  s_htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&s_htim5) != HAL_OK)
  {
    Error_Handler();
  }

  clock_cfg.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&s_htim5, &clock_cfg) != HAL_OK)
  {
    Error_Handler();
  }

  /* The wrap interrupt must sit at the lowest FreeRTOS-safe level so it can
     never delay a step pulse.  Its handler only accumulates 2^32. */
  HAL_NVIC_SetPriority(TIM5_IRQn, 15u, 0u);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  __HAL_TIM_CLEAR_FLAG(&s_htim5, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&s_htim5, TIM_IT_UPDATE);

  __HAL_TIM_SET_COUNTER(&s_htim5, 0u);
  __HAL_TIM_ENABLE(&s_htim5);

  s_us_epoch = arm_us_tim_read();
  s_us_high  = 0u;
  s_ready    = 1u;
}

/*============================================================================*/
/* Overflow interrupt, declared in arm_us.h                                   */
/*============================================================================*/

/**
  * @brief  [学习标注] TIM5 溢出中断体：用硬件计数器的实际增量把 32 位 µs 扩成 64 位（丢中断也
  *         只降精度）。
  */
void arm_us_irq_handler(void)
{
  if (__HAL_TIM_GET_FLAG(&s_htim5, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&s_htim5, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&s_htim5, TIM_IT_UPDATE);

      /* Exact 64 bit extension: accumulate the distance travelled since the
         previous wrap based on the counter value at that wrap.  Deriving the
         increment from the hardware counter makes lost interrupts harmless
         (only their precision is reduced, never their correctness). */
      uint32_t now = arm_us_tim_read();
      s_us_high += (uint32_t)(0u - s_us_epoch) + now;
      s_us_epoch = now;
    }
  }
}

/*============================================================================*/
/* Public read helpers                                                        */
/*============================================================================*/

/**
  * @brief  [学习标注] 返回 32 位 µs 计数，语义等同 Arduino micros()（会回绕，差值用无符号模减
  *         ）。
  */
uint32_t arm_us_micros(void)
{
  return arm_us_tim_read();
}

/**
  * @brief  [学习标注] 返回不回绕的 64 位 µs：读扩展值后重读计数器，若发生回绕则重试。
  */
uint64_t arm_us_micros64(void)
{
  uint64_t high;
  uint32_t epoch;
  uint32_t now;

  /* Read the extension, then re-read the hardware counter and verify that no
     wrap happened in between; retry if it did.  Time only moves forward, so
     if the counter went backwards a wrap must have occurred. */
  do
  {
    high  = (uint64_t)s_us_high << 32;
    epoch = s_us_epoch;
    now   = arm_us_tim_read();
  } while (now < epoch);

  return high + (uint64_t)now;
}

/**
  * @brief  [学习标注] 求经过的 µs（无符号模减，回绕安全）。
  */
uint32_t arm_us_elapsed(uint32_t start_us)
{
  return arm_us_micros() - start_us;
}

/**
  * @brief  [学习标注] 判断从 start_us 起是否已超过 timeout_us。
  */
bool arm_us_timed_out(uint32_t start_us, uint32_t timeout_us)
{
  return (arm_us_elapsed(start_us) >= timeout_us);
}

/*============================================================================*/
/* TIM5 interrupt vector                                                      */
/*============================================================================*/

/**
  * @brief  TIM5 global interrupt, i.e. the 32 bit counter wrap.
  * @note   This handler lives here on purpose, NOT in stm32f4xx_it.c.
  *         TIM5 is absent from RobotArm_F407.ioc, so CubeMX never emits a
  *         handler for it; a definition placed in stm32f4xx_it.c is deleted by
  *         the next GENERATE CODE.  That already happened once and left the
  *         vector pointing at Default_Handler while the interrupt was still
  *         enabled, which hangs the whole firmware once the counter wraps
  *         (2^32 us = 71.6 minutes).  The vector name is a plain global symbol,
  *         so defining it in this file overrides the weak startup alias and is
  *         never touched by code generation.
  */
void TIM5_IRQHandler(void)
{
  arm_us_irq_handler();
}
