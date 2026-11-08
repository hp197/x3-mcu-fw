/**
 * \file            control.h
 * \brief           Commutation, current and speed control loops
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/** \brief Run state of the controller (the run-state byte of the control block). */
typedef enum {
    CTRL_STATE_INIT = 0, /*!< pre-init / stopped                   */
    CTRL_STATE_IDLE = 1, /*!< initialised, bridge off              */
    CTRL_STATE_RUN = 2,  /*!< running: loops active, bridge driven */
    CTRL_STATE_FAULT = 7 /*!< latched fault, bridge cut            */
} ctrl_state_t;

/** \brief Initialise the control loops, the commutation state and the channel table. */
void control_init(void);

/** \brief Leave init and enter the running state. */
void control_start(void);

/** \brief Run one control update (read command, step the outer loop); the super-loop body. */
void main_service(void);

/** \brief Latest electrical speed estimate, raw timer-count units (0 when stopped). */
int32_t control_speed_raw(void);

/** \brief Latest commutation step, 0..5 (the live six-step sector). */
uint8_t control_comm_step(void);

/** \brief Cut the drive and latch a fault (called by the protection monitor). */
void control_trip(uint8_t fault_code);

#endif /* CONTROL_H */
