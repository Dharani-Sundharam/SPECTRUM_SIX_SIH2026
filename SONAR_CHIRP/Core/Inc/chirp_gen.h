/**
 * @file    chirp_gen.h
 * @brief   Linear Frequency Modulated (LFM) Sonar Chirp Generator
 *          for STM32F4VE — TIM6 / DMA / DAC1 CH1
 *
 * @details Generates a Hamming-windowed LFM chirp into a uint16_t buffer
 *          suitable for DMA-driven 12-bit right-aligned DAC playback.
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  Sampling-Rate Derivation  (APB1 timer clock = 84 MHz)          │
 * │                                                                   │
 * │  Target fs = 1.25 MSPS  (well above 2× f1_max = 1.0 MS/s)       │
 * │                                                                   │
 * │  TIM6 clock   = 84 MHz  (APB1 × 2 when APB1 prescaler > 1)      │
 * │  PSC          = 0       (no prescale, counts at 84 MHz)          │
 * │  ARR          = 66      (period = ARR + 1 = 67 ticks)            │
 * │  Actual fs    = 84 000 000 / 67 = 1 253 731 Hz ≈ 1.254 MSPS     │
 * │                                                                   │
 * │  Buffer length N = fs × T = 1 253 731 × 0.002 = 2508 samples    │
 * │  (rounded to CHIRP_BUFFER_LEN = 2508)                            │
 * │                                                                   │
 * │  Adjust TIMER_CLOCK_HZ if your system runs at 100 MHz or        │
 * │  170 MHz; recalculate ARR = round(TIMER_CLOCK_HZ / TARGET_FS)-1 │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * @hardware STM32F4VE
 *           DAC1 CH1  → PA4 (AF, no pull)
 *           TIM6 TRGO → DAC trigger
 *           DMA1 Stream 5 Channel 7 → DAC1 CH1
 *           Debug TX pin → PB0 (high during chirp burst)
 *
 * @author  Senior Embedded DSP Engineer
 * @version 1.0.0
 */

#ifndef CHIRP_GEN_H
#define CHIRP_GEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
/* =========================================================================
 * Clock & timing constants — adjust TIMER_CLOCK_HZ to match your CubeMX
 * system clock configuration.
 * ========================================================================= */

/** TIM6 input clock after APB1 × 2 multiplier.
 *  Default: 84 MHz for a 168 MHz core (APB1 = 42 MHz, ×2 = 84 MHz).
 *  Change to 100000000UL or 170000000UL for other configurations.       */
#define TIMER_CLOCK_HZ          84000000UL

/** Target DAC sample rate in Hz.  Must satisfy fs >= 2 × f1_max.
 *  1.25 MSPS satisfies Nyquist for f1 = 500 kHz with a comfortable margin. */
#define TARGET_FS_HZ            1250000UL

/** TIM6 Prescaler (PSC).  Set to 0 → timer counts at TIMER_CLOCK_HZ.    */
#define TIM6_PSC                0U

/** TIM6 Auto-Reload Register (ARR).
 *  ARR = round(TIMER_CLOCK_HZ / TARGET_FS_HZ) - 1
 *      = round(84 000 000 / 1 250 000) - 1 = 67 - 1 = 66
 *  Actual fs = 84 000 000 / (66 + 1) = 1 253 731 Hz                     */
#define TIM6_ARR                66U

/** Actual sampling frequency derived from timer settings (Hz, float).    */
#define FS_ACTUAL_HZ            ((float)TIMER_CLOCK_HZ / (float)(TIM6_ARR + 1U))

/* =========================================================================
 * Chirp parameters
 * ========================================================================= */

#define CHIRP_F0_HZ             100000.0f   /* 100 kHz start */
#define CHIRP_F1_HZ             500000.0f   /* 500 kHz end */
#define CHIRP_DURATION_S        0.002f      /* 2 milliseconds */

/** Buffer length: N = fs × T  (samples for one full chirp pulse).
 *  Computed at compile time from integer arithmetic; the floating-point
 *  product is 2507.46 → round up to 2508 to cover the full 2 ms.        */
#define CHIRP_BUFFER_LEN        2508U
/* =========================================================================
 * DAC output range — 12-bit right-aligned, avoiding rail saturation.
 * Full range: 0–4095.  Active window: 100–4000 (≈ 5 % headroom each side).
 * ========================================================================= */

#define DAC_OUTPUT_MIN          100U        /**< Floor  (avoids 0 V rail) */
#define DAC_OUTPUT_MAX          4000U       /**< Ceiling (avoids 3.3 V rail) */
#define DAC_OUTPUT_MID          2047U       /**< DC bias ≈ 1.65 V         */

/* =========================================================================
 * Debug / trigger GPIO
 *   Toggles PB0 HIGH for the duration of an active DMA chirp burst so
 *   an oscilloscope can latch the TX envelope on CH2 while CH1 shows PA4.
 * ========================================================================= */

#define CHIRP_DEBUG_GPIO_PORT   GPIOB
#define CHIRP_DEBUG_GPIO_PIN    GPIO_PIN_0

/* =========================================================================
 * External chirp buffer — allocated in chirp_gen.c, referenced in main.c
 * ========================================================================= */

/** DMA source buffer.  Placed in SRAM; must be 32-bit-accessible for DMA. */
extern uint16_t chirp_buffer[CHIRP_BUFFER_LEN];
/* Dynamic variable chirp generation and firing */
uint32_t chirp_gen_compute_var(float f0, float f1, float duration_s, float amplitude);
HAL_StatusTypeDef chirp_fire_var(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t len);
uint32_t geometric_sweep_compute(float f0, float f1, float duration_s, float amplitude);
uint32_t phase_coded_compute(float f0, float duration_s, float amplitude);
/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Pre-compute and store a Hamming-windowed LFM chirp into @p buffer.
 *
 * @param  buffer   Pointer to a uint16_t array of @p length elements.
 *                  Must be non-NULL and large enough to hold @p length words.
 * @param  length   Number of samples to generate (use CHIRP_BUFFER_LEN).
 * @param  f0       Start frequency in Hz.
 * @param  f1       Stop  frequency in Hz.
 * @param  fs       Sampling frequency in Hz (must match TIM6 rate).
 *
 * @note   Call once at startup (or before each burst if parameters change).
 *         CPU-intensive (floating-point loops); do NOT call inside ISR.
 */
void chirp_gen_compute(uint16_t *buffer,
                       uint32_t  length,
                       float     f0,
                       float     f1,
                       float     fs);

/**
 * @brief  Initialise the debug TX GPIO pin (PB0) as push-pull output.
 *         Call after MX_GPIO_Init() so the clock is already enabled.
 */
void chirp_debug_gpio_init(void);

/**
 * @brief  Arm and fire one chirp burst in Normal DMA mode.
 *
 * @details Asserts the debug pin, starts TIM6, and launches DMA.
 *          Returns immediately; the DMA TC callback de-asserts the pin
 *          and stops the timer.  Safe to call from button ISR or task.
 *
 * @param  hdac     Pointer to the HAL DAC handle (configured by CubeMX).
 * @param  htim6    Pointer to the HAL TIM6 handle (configured by CubeMX).
 * @retval HAL_StatusTypeDef  HAL_OK on success, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef chirp_fire(DAC_HandleTypeDef *hdac,
                             TIM_HandleTypeDef *htim6);

/**
 * @brief  DMA Transfer-Complete callback — call from HAL_DAC_ConvCpltCallbackCh1().
 *
 * @details Stops TIM6 (halts DAC triggering), de-asserts the debug pin,
 *          and optionally sets a flag so the application knows the burst ended.
 *
 * @param  hdac   Pointer to the HAL DAC handle.
 * @param  htim6  Pointer to the HAL TIM6 handle.
 */
void chirp_dma_complete_cb(DAC_HandleTypeDef *hdac,
                           TIM_HandleTypeDef *htim6);

/**
 * @brief  Returns 1 if a chirp burst is currently in progress, 0 otherwise.
 *         Poll this from the application loop to prevent re-triggering.
 */
uint8_t chirp_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* CHIRP_GEN_H */
