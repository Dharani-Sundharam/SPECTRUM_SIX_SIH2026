/**
 * @file    chirp_gen.c
 * @brief   LFM Sonar Chirp Generator — Implementation
 *
 * @details Generates a phase-continuous, Hamming-windowed Linear Frequency
 *          Modulated (LFM) chirp and manages DMA-driven single-shot DAC output
 *          on STM32F4VE via TIM6 → DMA1 Stream5 Ch7 → DAC1 CH1.
 */

/* =========================================================================
 * Includes
 * ========================================================================= */

#include "chirp_gen.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* =========================================================================
 * Private constants
 * ========================================================================= */

#define TWO_PI  6.28318530717958647692f

/* =========================================================================
 * Module-level state
 * ========================================================================= */

__attribute__((aligned(4)))
uint16_t chirp_buffer[CHIRP_BUFFER_LEN];

static volatile uint8_t s_chirp_active = 0U;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static inline uint16_t clamp_u16(uint32_t v, uint16_t lo, uint16_t hi)
{
    if (v < (uint32_t)lo) return lo;
    if (v > (uint32_t)hi) return hi;
    return (uint16_t)v;
}

/* =========================================================================
 * Public Functions (Fixed Length)
 * ========================================================================= */

void chirp_gen_compute(uint16_t *buffer, uint32_t length, float f0, float f1, float fs)
{
    if (buffer == NULL || length == 0U || fs <= 0.0f || f0 <= 0.0f || f1 <= f0 || f1 > fs / 2.0f) { return; }

    const float N_f = (float)length;
    const float T = (N_f - 1.0f) / fs;
    const float k = (f1 - f0) / T;
    const float dt = 1.0f / fs;
    const float dac_span = (float)(DAC_OUTPUT_MAX - DAC_OUTPUT_MIN);
    const float dac_min_f = (float)DAC_OUTPUT_MIN;
    const float win_denom = TWO_PI / (N_f - 1.0f);

    float phase = 0.0f;
    float f_inst = f0;
    const float df = k * dt;

    for (uint32_t n = 0U; n < length; n++) {
        float s = sinf(phase);
        float w = 0.54f - 0.46f * cosf(win_denom * (float)n);
        float y = s * w;
        float d_f = (y + 1.0f) * 0.5f * dac_span + dac_min_f + 0.5f;
        uint32_t d = (d_f >= 0.0f) ? (uint32_t)d_f : 0U;

        buffer[n] = clamp_u16(d, DAC_OUTPUT_MIN, DAC_OUTPUT_MAX);

        f_inst += df;
        phase += TWO_PI * f_inst * dt;
        if (phase >= TWO_PI) { phase -= TWO_PI; }
    }
}

void chirp_debug_gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = CHIRP_DEBUG_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CHIRP_DEBUG_GPIO_PORT, &gpio);
    HAL_GPIO_WritePin(CHIRP_DEBUG_GPIO_PORT, CHIRP_DEBUG_GPIO_PIN, GPIO_PIN_RESET);
}

HAL_StatusTypeDef chirp_fire(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim6)
{
    if (hdac == NULL || htim6 == NULL) { return HAL_ERROR; }
    if (s_chirp_active) { return HAL_BUSY; }

    s_chirp_active = 1U;
    HAL_GPIO_WritePin(CHIRP_DEBUG_GPIO_PORT, CHIRP_DEBUG_GPIO_PIN, GPIO_PIN_SET);

    HAL_StatusTypeDef status = HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t *)chirp_buffer, (uint32_t)CHIRP_BUFFER_LEN, DAC_ALIGN_12B_R);
    if (status != HAL_OK) {
        s_chirp_active = 0U;
        HAL_GPIO_WritePin(CHIRP_DEBUG_GPIO_PORT, CHIRP_DEBUG_GPIO_PIN, GPIO_PIN_RESET);
        return HAL_ERROR;
    }

    status = HAL_TIM_Base_Start(htim6);
    if (status != HAL_OK) {
        HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
        s_chirp_active = 0U;
        HAL_GPIO_WritePin(CHIRP_DEBUG_GPIO_PORT, CHIRP_DEBUG_GPIO_PIN, GPIO_PIN_RESET);
        return HAL_ERROR;
    }
    return HAL_OK;
}

void chirp_dma_complete_cb(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim6)
{
    if (hdac == NULL || htim6 == NULL) { return; }
    HAL_TIM_Base_Stop(htim6);
    HAL_GPIO_WritePin(CHIRP_DEBUG_GPIO_PORT, CHIRP_DEBUG_GPIO_PIN, GPIO_PIN_RESET);
    s_chirp_active = 0U;
}

uint8_t chirp_is_active(void)
{
    return s_chirp_active;
}

/* =========================================================================
 * Public Functions (Dynamic/Variable Modes)
 * ========================================================================= */

uint32_t chirp_gen_compute_var(float f0, float f1, float duration_s, float amplitude)
{
    uint32_t samples = (uint32_t)(duration_s * FS_ACTUAL_HZ);
    if (samples > CHIRP_BUFFER_LEN) samples = CHIRP_BUFFER_LEN;
    else if (samples < 10) samples = 10;
    if (amplitude > 1.0f) amplitude = 1.0f;
    if (amplitude < 0.1f) amplitude = 0.1f;

    const float k = (f1 - f0) / duration_s;
    const float dt = 1.0f / FS_ACTUAL_HZ;
    const float dac_mid = 2047.5f;
    const float dac_max_span = 2047.0f * amplitude;

    for (uint32_t n = 0; n < samples; n++) {
        float t = n * dt;
        float phase = TWO_PI * (f0 * t + 0.5f * k * t * t);
        float hamming = 0.54f - 0.46f * cosf(TWO_PI * (float)n / (float)(samples - 1));
        float val = dac_mid + (dac_max_span * hamming * cosf(phase));

        if (val < 0.0f) val = 0.0f;
        if (val > 4095.0f) val = 4095.0f;
        chirp_buffer[n] = (uint16_t)val;
    }
    return samples;
}

uint32_t geometric_sweep_compute(float f0, float f1, float duration_s, float amplitude)
{
    uint32_t samples = (uint32_t)(duration_s * FS_ACTUAL_HZ);
    if (samples > CHIRP_BUFFER_LEN) samples = CHIRP_BUFFER_LEN;
    else if (samples < 10) samples = 10;
    if (amplitude > 1.0f) amplitude = 1.0f;
    if (amplitude < 0.1f) amplitude = 0.1f;

    const float dt = 1.0f / FS_ACTUAL_HZ;
    const float dac_mid = 2047.5f;
    const float dac_max_span = 2047.0f * amplitude;

    float ln_ratio = logf(f1 / f0);
    float phase_coef = TWO_PI * (f0 * duration_s) / ln_ratio;
    float base_ratio = f1 / f0;

    for (uint32_t n = 0; n < samples; n++) {
        float t = n * dt;
        float t_norm = t / duration_s;
        float phase = phase_coef * (powf(base_ratio, t_norm) - 1.0f);
        float hamming = 0.54f - 0.46f * cosf(TWO_PI * (float)n / (float)(samples - 1));

        float val = dac_mid + (dac_max_span * hamming * cosf(phase));
        if (val < 0.0f) val = 0.0f;
        if (val > 4095.0f) val = 4095.0f;
        chirp_buffer[n] = (uint16_t)val;
    }
    return samples;
}

uint32_t phase_coded_compute(float f0, float duration_s, float amplitude)
{
    uint32_t samples = (uint32_t)(duration_s * FS_ACTUAL_HZ);
    if (samples > CHIRP_BUFFER_LEN) samples = CHIRP_BUFFER_LEN;
    else if (samples < 13) samples = 13;
    if (amplitude > 1.0f) amplitude = 1.0f;
    if (amplitude < 0.1f) amplitude = 0.1f;

    const float dt = 1.0f / FS_ACTUAL_HZ;
    const float dac_mid = 2047.5f;
    const float dac_max_span = 2047.0f * amplitude;

    int8_t barker13[13] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
    uint32_t samples_per_chip = samples / 13;

    for (uint32_t n = 0; n < samples; n++) {
        float t = n * dt;
        float phase = TWO_PI * f0 * t;

        uint32_t chip_idx = n / samples_per_chip;
        if (chip_idx > 12) chip_idx = 12;

        if (barker13[chip_idx] == -1) {
            phase += 3.14159265358979f;
        }

        float hamming = 0.54f - 0.46f * cosf(TWO_PI * (float)n / (float)(samples - 1));
        float val = dac_mid + (dac_max_span * hamming * cosf(phase));

        if (val < 0.0f) val = 0.0f;
        if (val > 4095.0f) val = 4095.0f;
        chirp_buffer[n] = (uint16_t)val;
    }
    return samples;
}

HAL_StatusTypeDef chirp_fire_var(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t len)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_StatusTypeDef status = HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t *)chirp_buffer, len, DAC_ALIGN_12B_R);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        return status;
    }
    return HAL_TIM_Base_Start(htim);
}
