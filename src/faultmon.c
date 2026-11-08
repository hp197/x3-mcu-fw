/**
 * \file            faultmon.c
 * \brief           Protection: over-current, over-voltage and stall handling
 * \note            Two parts. At startup \ref faultmon_calibrate measures the phase-current
 *                      amplifier zero offsets with the bridge off: each phase is averaged over
 *                      many conversions and must sit within a tight window of mid-scale, else the
 *                      unit refuses to arm (fault code \ref FAULT_CURRENT_OFFSET). At run time
 *                      \ref faultmon_check watches the phase currents, the bus voltage and the
 *                      commutation for an over-current / over-voltage / stall condition and cuts
 *                      the drive through \ref control_trip. There is no hardware brake input; the
 *                      over-current trip is in software.
 *
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "faultmon.h"

#include "adc.h"
#include "control.h"

/* ------------------------------------------------------------------------- */
/* Thresholds (protection limits).                                   */
/* ------------------------------------------------------------------------- */

#define FM_CAL_SAMPLES  512u  /*!< conversions per phase at calibration            */
#define FM_CAL_AVERAGED 256u  /*!< the last-N conversions that are averaged        */
#define FM_CAL_MIDSCALE 2048u /*!< expected zero reading (12-bit mid-scale)        */
#define FM_CAL_WINDOW   100u  /*!< allowed +/- deviation of the zero offset        */

#define FM_OC_TRIP      1800 /*!< per-phase over-current trip, offset-removed counts */
#define FM_OC_DEBOUNCE  3u   /*!< consecutive over-current samples before a trip  */

#define FM_OV_TRIP      3600u /*!< bus-voltage over-voltage trip, ordinary counts   */
#define FM_UV_TRIP      600u  /*!< bus-voltage brown-out trip, ordinary counts      */
#define FM_BUS_RANK     0u    /*!< ordinary-scan rank carrying the bus voltage      */

#define FM_STALL_TICKS  400u /*!< commanded-but-not-turning ticks before a stall trip */

/* ------------------------------------------------------------------------- */
/* State.                                                                     */
/* ------------------------------------------------------------------------- */

static uint16_t prv_offset[3];  /*!< measured phase-current zero offsets */
static uint8_t prv_oc_count;    /*!< over-current debounce counter       */
static uint32_t prv_stall_tick; /*!< stall debounce counter              */

/* ------------------------------------------------------------------------- */
/* Public.                                                                    */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Reset the protection latches and debounce state
 */
void faultmon_init(void) {
    uint8_t i = 0;

    for (i = 0; i < 3u; i++) {
        prv_offset[i] = FM_CAL_MIDSCALE;
    }
    prv_oc_count = 0;
    prv_stall_tick = 0;
}

/**
 * \brief           Measure the phase-current zero offsets at startup
 * \return          \ref FAULT_NONE on success, else \ref FAULT_CURRENT_OFFSET
 * \note            With the bridge off, each phase is sampled \ref FM_CAL_SAMPLES times and the
 *                      last \ref FM_CAL_AVERAGED are averaged. The average must sit within
 *                      \ref FM_CAL_WINDOW of \ref FM_CAL_MIDSCALE or the phase is rejected.
 */
uint8_t faultmon_calibrate(void) {
    uint8_t result = FAULT_NONE;
    uint8_t phase = 0;

    for (phase = 0; phase < 3u; phase++) {
        uint32_t acc = 0;
        uint16_t n = 0;
        uint16_t mean = 0;
        int32_t dev = 0;

        for (n = 0; n < FM_CAL_SAMPLES; n++) {
            uint16_t sample = adc_phase_current_raw((adc_phase_t)phase);

            if (n >= (FM_CAL_SAMPLES - FM_CAL_AVERAGED)) {
                acc += sample;
            }
        }
        mean = (uint16_t)(acc / FM_CAL_AVERAGED);
        prv_offset[phase] = mean;

        dev = (int32_t)mean - (int32_t)FM_CAL_MIDSCALE;
        if (dev < 0) {
            dev = -dev;
        }
        if (dev > (int32_t)FM_CAL_WINDOW) {
            result = FAULT_CURRENT_OFFSET;
        }
    }
    return result;
}

/**
 * \brief           Run the runtime protection checks
 * \note            Over-current (debounced), over-/under-voltage and stall; any confirmed fault
 *                      cuts the drive through \ref control_trip.
 */
void faultmon_check(void) {
    uint16_t bus = adc_ordinary_mean(FM_BUS_RANK);
    uint8_t phase = 0;
    uint8_t over = 0;

    for (phase = 0; phase < 3u; phase++) {
        int32_t i = (int32_t)adc_phase_current_raw((adc_phase_t)phase) - (int32_t)prv_offset[phase];

        if (i < 0) {
            i = -i;
        }
        if (i > FM_OC_TRIP) {
            over = 1;
        }
    }

    prv_oc_count = over ? (uint8_t)(prv_oc_count + 1u) : 0u;
    if (prv_oc_count >= FM_OC_DEBOUNCE) {
        control_trip(FAULT_OVER_CURRENT);
    } else if (bus > FM_OV_TRIP) {
        control_trip(FAULT_OVER_VOLTAGE);
    } else if (bus < FM_UV_TRIP) {
        control_trip(FAULT_UNDER_VOLTAGE);
    } else {
        /* Stall: driven (commutation step changing was expected) but no speed for a while. */
        if (control_speed_raw() == 0) {
            prv_stall_tick++;
            if (prv_stall_tick >= FM_STALL_TICKS) {
                control_trip(FAULT_STALL);
            }
        } else {
            prv_stall_tick = 0;
        }
    }
}

/**
 * \brief           The phase-current zero offset measured at init
 * \param[in]       phase: 0..2 (U/V/W)
 * \return          the offset in raw ADC counts
 */
uint16_t faultmon_current_offset(uint8_t phase) {
    uint16_t out = FM_CAL_MIDSCALE;

    if (phase < 3u) {
        out = prv_offset[phase];
    }
    return out;
}
