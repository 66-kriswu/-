/**
  ******************************************************************************
  * @file    arm_us.h
  * @brief   Microsecond time base on TIM5 (replaces Arduino micros()).
  ******************************************************************************
  */

#ifndef ARM_US_H
#define ARM_US_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Install TIM5 as a free running 1 MHz / 32 bit counter.
  * Call once, after the HAL is up and before any task uses arm_us_micros(). */
void arm_us_init(void);

/** Raw 32 bit counter, wraps every 2^32 us (71.58 min) like the original. */
uint32_t arm_us_micros(void);

/** Wrap-free 64 bit microsecond stamp, safe across the 71.58 min rollover. */
uint64_t arm_us_micros64(void);

/** Elapsed microseconds since start_us, 32 bit wrap-aware subtraction. */
uint32_t arm_us_elapsed(uint32_t start_us);

/** True when timeout_us microseconds have elapsed since start_us. */
bool arm_us_timed_out(uint32_t start_us, uint32_t timeout_us);

/** TIM5 overflow handler.  Called from TIM5_IRQHandler, which this firmware
  * installs because TIM5 does not exist in the .ioc file.                    */
void arm_us_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_US_H */
