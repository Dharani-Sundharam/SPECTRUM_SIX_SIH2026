///**
// * @file    sensors.c
// * @brief   Reads 3 potentiometers via ADC1 (PA1, PA2, PA3)
// *          and maps them to chirp parameters.
// *
// * Mapping:
// *   PA1 (ADC1 CH1) → Center frequency  100 kHz – 500 kHz
// *   PA2 (ADC1 CH2) → Pulse duration    0.5 ms  – 5.0 ms
// *   PA3 (ADC1 CH3) → Amplitude         0.1     – 1.0
// */
//
//#include "sensors.h"
//#include <stddef.h>
//
///* ── Private handle pointer ── */
//static ADC_HandleTypeDef *s_hadc = NULL;
//
///* ── Helper: read one ADC channel, returns 0–4095 ── */
//static uint32_t adc_read_channel(uint32_t channel)
//{
//    ADC_ChannelConfTypeDef cfg = {0};
//    cfg.Channel      = channel;
//    cfg.Rank         = 1;
//    cfg.SamplingTime = ADC_SAMPLETIME_84CYCLES; /* stable for pot impedance */
//    HAL_ADC_ConfigChannel(s_hadc, &cfg);
//    HAL_ADC_Start(s_hadc);
//    HAL_ADC_PollForConversion(s_hadc, 10);      /* 10 ms timeout            */
//    uint32_t val = HAL_ADC_GetValue(s_hadc);
//    HAL_ADC_Stop(s_hadc);
//    return val;
//}
//
///* ── Helper: map raw 0–4095 to float [out_min, out_max] ── */
//static float adc_map(uint32_t raw, float out_min, float out_max)
//{
//    float norm = (float)raw / 4095.0f;          /* 0.0 – 1.0                */
//    return out_min + norm * (out_max - out_min);
//}
//
///* ================================================================
// * sensors_init
// * Call once after MX_GPIO_Init() and MX_ADC1_Init()
// * ================================================================ */
//void sensors_init(ADC_HandleTypeDef *hadc)
//{
//    s_hadc = hadc;
//}
//
///* ================================================================
// * sensors_read
// * Reads all 3 pots and returns ready-to-use ChirpParams.
// * Blocks ~0.3 ms total (3 × 84-cycle sample @ 84 MHz ADC clock).
// * ================================================================ */
//ChirpParams sensors_read(void)
//{
//    ChirpParams p = {0};
//
//    if (s_hadc == NULL) {
//        /* Safe defaults if called before init */
//        p.f0        = 100000.0f;
//        p.f1        = 500000.0f;
//        p.duration  = 0.002f;
//        p.amplitude = 1.0f;
//        return p;
//    }
//
//    /* ── POT1 → PA1 (ADC1 CH1): Center frequency ──────────────────
//     * Maps 0–3.3V  →  100 kHz – 500 kHz center frequency.
//     * We keep bandwidth fixed at 200 kHz around center so the
//     * sweep always stays within the 100–500 kHz band.
//     *   center = 100k – 500k
//     *   f0     = center - 100k  (clamped to 100k min)
//     *   f1     = center + 100k  (clamped to 500k max)
//     * ─────────────────────────────────────────────────────────── */
//    uint32_t raw1   = adc_read_channel(ADC_CHANNEL_1);
//    float center    = adc_map(raw1, 100000.0f, 500000.0f);
//    p.f0 = center - 100000.0f;
//    p.f1 = center + 100000.0f;
//    if (p.f0 < 100000.0f) { p.f0 = 100000.0f; }
//    if (p.f1 > 500000.0f) { p.f1 = 500000.0f; }
//    if (p.f1 <= p.f0)     { p.f1 = p.f0 + 50000.0f; } /* min 50 kHz BW    */
//
//    /* ── POT2 → PA2 (ADC1 CH2): Pulse duration ────────────────────
//     * Maps 0–3.3V  →  0.5 ms – 5.0 ms
//     * ─────────────────────────────────────────────────────────── */
//    uint32_t raw2 = adc_read_channel(ADC_CHANNEL_2);
//    p.duration    = adc_map(raw2, 0.0005f, 0.005f);
//
//    /* ── POT3 → PA3 (ADC1 CH3): Amplitude scale ───────────────────
//     * Maps 0–3.3V  →  0.1 – 1.0
//     * Applied as a multiplier on DAC_OUTPUT_MAX during generation.
//     * ─────────────────────────────────────────────────────────── */
//    uint32_t raw3 = adc_read_channel(ADC_CHANNEL_3);
//    p.amplitude   = adc_map(raw3, 0.1f, 1.0f);
//
//    return p;
//}


/**
 * @file    sensors.c
 * @brief   Reads 3 potentiometers via ADC1 (PA1, PA2, PA3)
 *          and maps them to chirp parameters.
 *
 * Mapping:
 *   PA1 (ADC1 CH1) → Center frequency  100 kHz – 500 kHz
 *   PA2 (ADC1 CH2) → Pulse duration    0.5 ms  – 5.0 ms
 *   PA3 (ADC1 CH3) → Amplitude         0.1     – 1.0
 */

#include "sensors.h"
#include <stddef.h>

/* ── Private handle pointer ── */
static ADC_HandleTypeDef *s_hadc = NULL;

/* ── Helper: read one ADC channel, returns 0–4095 ── */
static uint32_t adc_read_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = channel;
    cfg.Rank         = 1;
    cfg.SamplingTime = ADC_SAMPLETIME_84CYCLES;  /* stable for pot impedance */
    HAL_ADC_ConfigChannel(s_hadc, &cfg);
    HAL_ADC_Start(s_hadc);
    HAL_ADC_PollForConversion(s_hadc, 10);       /* 10 ms timeout            */
    uint32_t val = HAL_ADC_GetValue(s_hadc);
    HAL_ADC_Stop(s_hadc);
    return val;
}

/* ── Helper: map raw 0–4095 to float [out_min, out_max] ── */
static float adc_map(uint32_t raw, float out_min, float out_max)
{
    float norm = (float)raw / 4095.0f;          /* 0.0 – 1.0                */
    return out_min + norm * (out_max - out_min);
}

/* ================================================================
 * sensors_init
 * Call once after MX_GPIO_Init() and MX_ADC1_Init()
 * ================================================================ */
void sensors_init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
}

/* ================================================================
 * sensors_read
 * Reads all 3 pots and returns ready-to-use ChirpParams.
 * ================================================================ */
ChirpParams sensors_read(void)
{
    ChirpParams p = {0};

    if (s_hadc == NULL) {
        p.f0        = 100000.0f;
        p.f1        = 500000.0f;
        p.duration  = 0.002f;
        p.amplitude = 1.0f;
        return p;
    }

    /* ── POT1 → PA1 (ADC1 CH1): Sweep bandwidth ──────────────────────
     * f0 stays FIXED at 100 kHz (lowest usable sonar frequency).
     * Pot controls how far the sweep extends: pot=0 gives a narrow
     * 100k-150k sweep, pot=max gives the full 100k-500k band.
     * This makes the sweep visibly "grow" as you turn the pot —
     * much clearer on a scope than a sliding-center-frequency window.
     * ───────────────────────────────────────────────────────────── */
    uint32_t raw1 = adc_read_channel(ADC_CHANNEL_1);
    p.raw1 = raw1;
    p.f0 = 100000.0f;
    p.f1 = adc_map(raw1, 150000.0f, 500000.0f);

    /* ── POT2 → PA2 (ADC1 CH2): Pulse duration ── */
    uint32_t raw2 = adc_read_channel(ADC_CHANNEL_2);
    p.raw2 = raw2;
    p.duration    = adc_map(raw2, 0.0005f, 0.005f);

    /* ── POT3 → PA3 (ADC1 CH3): Amplitude scale ── */
    uint32_t raw3 = adc_read_channel(ADC_CHANNEL_3);
    p.raw3 = raw3;
    p.amplitude   = adc_map(raw3, 0.1f, 1.0f);

    return p;
}
