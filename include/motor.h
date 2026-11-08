/**
 * \file            motor.h
 * \brief           TMR1 complementary PWM: the 3-phase bridge
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/** \brief PWM carrier period (TMR1 reload); center-aligned, carrier = TMR1clk / (2*(PERIOD+1)). */
#define MOTOR_PWM_PERIOD 0x1193u /*!< 4499 -> 144 MHz / (2 * 4500) = 16 kHz carrier */

/** \brief Motor phase, mapped to the TMR1 output channels (U=CH1, V=CH2, W=CH3). */
typedef enum { MOTOR_PHASE_U = 0, MOTOR_PHASE_V = 1, MOTOR_PHASE_W = 2 } motor_phase_t;

/** \brief Bring up the TMR1 3-phase complementary-PWM bridge (outputs start disabled). */
void motor_init(void);

/** \brief Set one phase's compare (0..MOTOR_PWM_PERIOD); the commutation/current loop drives this. */
void motor_set_duty(motor_phase_t phase, uint16_t compare);

/** \brief Enable the six bridge outputs (the three complementary pairs). */
void motor_output_enable(void);

/** \brief Disable the six bridge outputs (freewheel). */
void motor_output_disable(void);

#endif /* MOTOR_H */
