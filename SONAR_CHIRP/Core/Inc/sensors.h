///**
// * @file    sensors.h
// * @brief   Potentiometer → Chirp Parameter Mapper
// *          PA1 = Frequency, PA2 = Duration, PA3 = Amplitude
// */
//
//#ifndef SENSORS_H
//#define SENSORS_H
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#include "stm32f4xx_hal.h"
//
///* ── Output structure — one read fills all three ── */
//typedef struct {
//    float f0;          /* Start frequency Hz  (100k – 500k) */
//    float f1;          /* Stop  frequency Hz  (f0+BW)       */
//    float duration;    /* Pulse duration  s   (0.5m – 5ms)  */
//    float amplitude;   /* Scale factor    0.1 – 1.0         */
//} ChirpParams;
//
///* Call once after MX_GPIO_Init */
//void sensors_init(ADC_HandleTypeDef *hadc);
//
///* Call on button press — blocks ~1 ms to read 3 channels */
//ChirpParams sensors_read(void);
//
//#ifdef __cplusplus
//}
//#endif
//#endif /* SENSORS_H */


/**
 * @file    sensors.h
 * @brief   Potentiometer → Chirp Parameter Mapper
 *          PA1 = Frequency, PA2 = Duration, PA3 = Amplitude
 */

#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ── Output structure — one read fills all three ── */
typedef struct {
    float f0;          /* Start frequency Hz  (100k – 500k) */
    float f1;          /* Stop  frequency Hz  (f0+BW)       */
    float duration;    /* Pulse duration  s   (0.5m – 5ms)  */
    float amplitude;   /* Scale factor    0.1 – 1.0         */
    uint32_t raw1;      /* Raw ADC1 CH1 (0-4095) — debug only */
    uint32_t raw2;      /* Raw ADC1 CH2 (0-4095) — debug only */
    uint32_t raw3;      /* Raw ADC1 CH3 (0-4095) — debug only */
} ChirpParams;

/* Call once after MX_GPIO_Init */
void sensors_init(ADC_HandleTypeDef *hadc);

/* Call on button press — blocks ~1 ms to read 3 channels */
ChirpParams sensors_read(void);

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H */
