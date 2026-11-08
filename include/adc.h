/**
 * \file            adc.h
 * \brief           ADC1 acquisition: phase-current, bus-voltage and throttle sensing
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/** \brief Number of ranks in the ordinary (slow-monitor) scan. */
#define ADC_ORDINARY_RANKS 5u

/** \brief Depth of the ordinary DMA ring (scans kept per rank for averaging). */
#define ADC_ORDINARY_DEPTH 40u

/**
 * \brief           Motor phase for the preempt (synchronous current) group
 * \note            ch5/6/7 (PA5/PA6/PA7) are sampled by the preempt group on every TMR1 CH4
 *                      event and latched in the ADC1 preempt data registers.
 */
typedef enum {
    ADC_PHASE_U = 0, /*!< phase U current -> ADC1 preempt data 1 (ch5, PA5) */
    ADC_PHASE_V = 1, /*!< phase V current -> ADC1 preempt data 2 (ch6, PA6) */
    ADC_PHASE_W = 2, /*!< phase W current -> ADC1 preempt data 3 (ch7, PA7) */
} adc_phase_t;

/**
 * \brief           Initialise ADC1 acquisition (ordinary DMA scan + preempt current group)
 */
void adc_init(void);

/**
 * \brief           Latest raw phase-current sample from the preempt group
 * \param[in]       phase: motor phase \ref adc_phase_t
 * \return          12-bit right-aligned preempt conversion (mid-scale ~2048 at zero current)
 */
uint16_t adc_phase_current_raw(adc_phase_t phase);

/**
 * \brief           Mean of the ordinary DMA ring for one scan rank
 * \param[in]       rank: ordinary scan rank 0..ADC_ORDINARY_RANKS-1
 * \return          averaged 12-bit sample, or 0 for an out-of-range rank
 */
uint16_t adc_ordinary_mean(uint8_t rank);

#endif /* ADC_H */
