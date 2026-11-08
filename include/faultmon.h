/**
 * \file            faultmon.h
 * \brief           Protection: over-current, over-voltage and stall handling
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef FAULTMON_H
#define FAULTMON_H

#include <stdint.h>

/** \brief Fault codes reported over CAN and latched into the control state. */
typedef enum {
    FAULT_NONE = 0x00,          /*!< no fault                                         */
    FAULT_OVER_CURRENT = 0x02,  /*!< phase current over the trip threshold            */
    FAULT_OVER_VOLTAGE = 0x03,  /*!< bus voltage over the trip threshold              */
    FAULT_UNDER_VOLTAGE = 0x04, /*!< bus voltage under the brown-out threshold        */
    FAULT_STALL = 0x08,         /*!< no commutation while driven (rotor stalled)      */
    FAULT_CURRENT_OFFSET = 0x0D /*!< phase-current zero-offset out of window at init  */
} fault_code_t;

/** \brief Reset the protection latches and debounce state. */
void faultmon_init(void);

/**
 * \brief           Measure the phase-current zero offsets at startup
 * \return          \ref FAULT_NONE on success, else \ref FAULT_CURRENT_OFFSET
 */
uint8_t faultmon_calibrate(void);

/**
 * \brief           Run the runtime protection checks (over-current / over-voltage / stall)
 * \note            Cuts the drive via \ref control_trip on a confirmed fault.
 */
void faultmon_check(void);

/** \brief The phase-current zero offset measured at init, raw ADC counts. */
uint16_t faultmon_current_offset(uint8_t phase);

#endif /* FAULTMON_H */
