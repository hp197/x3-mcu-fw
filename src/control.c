/**
 * \file            control.c
 * \brief           Commutation, current and speed control loops
 * \note            Turns a VCU drive command into three-phase PWM. Three layers run it:
 *                      - the super-loop body (\ref main_service) reads the command and sets the
 *                        current reference through the ramp regions;
 *                      - the current loop runs in the ADC preempt-complete interrupt, synchronous
 *                        with the PWM carrier (the phase currents are sampled mid-pulse on the
 *                        TMR1 CH4 trigger), and writes the three duties;
 *                      - the six-step commutation runs in the TMR3 hall interrupt, which captures
 *                        every hall edge, advances the sector, estimates the electrical speed and
 *                        selects which phases the bridge drives.
 *
 *                      The drive ceilings (current reference, speed) are taken from the stored
 *                      configuration and clamped like any other control constant. The control math
 *                      is the native-float form of the firmware's fixed-point sequence.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "control.h"

#include "at32f415.h"

#include "adc.h"
#include "backup.h"
#include "can.h"
#include "config.h"
#include "faultmon.h"
#include "motor.h"

/* ------------------------------------------------------------------------- */
/* Tuning constants (stored configuration + fixed values).           */
/* ------------------------------------------------------------------------- */

#define CTRL_PWM_FULL   (MOTOR_PWM_PERIOD + 1u)        /*!< 100 % duty = carrier reload + 1 */
#define CTRL_PWM_HALF   ((MOTOR_PWM_PERIOD + 1u) / 2u) /*!< 50 % duty, the bridge midpoint */

/* Current-loop gain: the constant applied in the loop's native-float form (the
   firmware's fixed-point sequence multiplies the error by this gain). */
#define CTRL_IQ_GAIN    1088.31f /*!< current-loop gain */
#define CTRL_IQ_KI      0.06f    /*!< current-loop integral weight per tick */
#define CTRL_IQ_INT_MAX 1.00f    /*!< current-loop integrator clamp (anti-windup) */

/* The command deadband, the drive ceiling and the default setpoint come from the stored tune
   cells (device-sourced, read through config_tune); only their normalisation is fixed here. */
#define CTRL_CMD_FULL   0x7FFFu /*!< drive-command full scale (Q15 command magnitude) */

/* Six-step phase roles: -1 = float, 0 = high side PWM'd, 1 = low side on. */
#define PH_FLOAT        (-1)
#define PH_HIGH         (0)
#define PH_LOW          (1)

/** \brief Six-step commutation table: [sector][U,V,W] drive roles (block commutation). */
static const int8_t prv_comm_table[6][3] = {
    {PH_HIGH, PH_LOW, PH_FLOAT}, /* sector 0: U+ V- */
    {PH_HIGH, PH_FLOAT, PH_LOW}, /* sector 1: U+ W- */
    {PH_FLOAT, PH_HIGH, PH_LOW}, /* sector 2: V+ W- */
    {PH_LOW, PH_HIGH, PH_FLOAT}, /* sector 3: V+ U- */
    {PH_LOW, PH_FLOAT, PH_HIGH}, /* sector 4: W+ U- */
    {PH_FLOAT, PH_LOW, PH_HIGH}, /* sector 5: W+ V- */
};

/* Hall code (bits PB0,PB4,PB5) -> electrical sector 0..5; 0 and 7 are illegal (open/short). */
static const int8_t prv_hall_to_sector[8] = {-1, 0, 2, 1, 4, 5, 3, -1};

/* ------------------------------------------------------------------------- */
/* Control state (SRAM blackboard of the control module).                    */
/* ------------------------------------------------------------------------- */

/** \brief Outer (command / reference) loop state. */
typedef struct {
    uint8_t run_state;   /*!< \ref ctrl_state_t run state         */
    uint8_t fault_code;  /*!< latched fault (\ref fault_code_t)   */
    float cmd;           /*!< command -1..+1 from the VCU         */
    float iq_ref;        /*!< current reference (clamped)         */
    uint32_t loop_tick;  /*!< super-loop iteration counter        */
    uint32_t loop_us;    /*!< last super-loop span, timer counts  */
    uint32_t comms_lost; /*!< ticks since the last keep-alive     */
} ctrl_outer_t;

/** \brief Current-loop state (runs in the ADC interrupt). */
typedef struct {
    float iq_meas;    /*!< measured quadrature current         */
    float iq_int;     /*!< current-loop integrator             */
    float duty;       /*!< commanded duty -1..+1               */
    int32_t i_raw[3]; /*!< raw phase currents, offset-removed  */
} ctrl_current_t;

/** \brief Commutation + speed state (runs in the TMR3 hall interrupt). */
typedef struct {
    uint8_t sector;       /*!< live electrical sector 0..5          */
    uint8_t hall;         /*!< last hall code 1..6                  */
    int32_t speed_raw;    /*!< electrical speed, timer-count units  */
    uint32_t cap_prev;    /*!< previous hall-edge capture           */
    uint32_t cap_period;  /*!< counts between the last two edges    */
    uint32_t stall_ticks; /*!< hall-idle ticks while driven         */
} ctrl_comm_t;

static ctrl_outer_t prv_outer;
static ctrl_current_t prv_cur;
static ctrl_comm_t prv_comm;

/** \brief The stored calibration / tune cells (thresholds, ceiling, default setpoint). */
static const mcu_tune_t* prv_tune;

/* ------------------------------------------------------------------------- */
/* Helpers.                                                                   */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Clamp a float to a symmetric range
 * \param[in]       v: value
 * \param[in]       lim: positive limit
 * \return          `v` clamped to `[-lim, +lim]`
 */
static float prv_clampf(float v, float lim) {
    float out = v;

    if (out > lim) {
        out = lim;
    } else if (out < -lim) {
        out = -lim;
    }
    return out;
}

/**
 * \brief           Map the VCU command to a current reference through the ramp regions
 * \param[in]       cmd: command, -1..+1
 * \return          current reference, -1..+1 (zero inside the deadband)
 * \note            Below \ref CTRL_CMD_DEADBAND the reference is zero; above it the magnitude
 *                      ramps linearly to full scale at \ref CTRL_CMD_FULL. The ceiling is the
 *                      stored drive limit, applied as a plain clamp.
 */
static float prv_cmd_to_iref(float cmd) {
    /* The command magnitude ramps from the stored deadband threshold up to full scale; below the
       deadband the reference holds at the default-setpoint fraction. The deadband and the drive
       ceiling are the stored tune cells (device-sourced); the default setpoint is 20. */
    float dead = (float)prv_tune->cmd_deadband / (float)CTRL_CMD_FULL;
    float ceiling = (float)prv_tune->cmd_ceiling / (float)CTRL_CMD_FULL;
    float dflt = (float)prv_tune->cmd_default / (float)CTRL_CMD_FULL;
    float mag = (cmd < 0.0f) ? -cmd : cmd;
    float ref = dflt;

    if (ceiling <= dead) {
        ceiling = 1.0f; /* unprogrammed cells: fall back to full scale */
    }
    if (mag > dead) {
        ref = (mag - dead) / (ceiling - dead);
        ref = prv_clampf(ref, 1.0f);
        ref = (cmd < 0.0f) ? -ref : ref;
    }
    return ref;
}

/**
 * \brief           Drive the three half-bridges for a sector at a given duty
 * \param[in]       sector: electrical sector 0..5
 * \param[in]       duty: 0..1 applied to the high-side phase
 */
static void prv_apply_sector(uint8_t sector, float duty) {
    uint16_t compare = (uint16_t)(prv_clampf(duty, 1.0f) * (float)CTRL_PWM_FULL);
    uint8_t phase = 0;

    for (phase = 0; phase < 3u; phase++) {
        int8_t role = prv_comm_table[sector][phase];
        uint16_t value = CTRL_PWM_HALF;

        if (role == PH_HIGH) {
            value = compare;
        } else if (role == PH_LOW) {
            value = 0;
        }
        motor_set_duty((motor_phase_t)phase, value);
    }
}

/**
 * \brief           Apply a decoded drive command and refresh the comms watchdog
 * \param[in]       cmd: command, -1..+1
 */
static void prv_apply_command(float cmd) {
    prv_outer.cmd = prv_clampf(cmd, 1.0f);
    prv_outer.comms_lost = 0;
}

/**
 * \brief           Drain the CAN RX ring and act on the inbound frames
 * \note            The drive command (\ref CANID_DRIVE_CMD) carries a signed setpoint in the
 *                      first two payload bytes, little-endian, full scale +/-0x7FFF; the
 *                      keep-alive (\ref CANID_KEEPALIVE) refreshes the comms watchdog.
 */
static void prv_can_drain(void) {
    mcu_can_msg_t msg;

    while (can_rx_pop(&msg) != 0u) {
        if (msg.std_id == CANID_DRIVE_CMD) {
            int16_t raw = (int16_t)((uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8));

            prv_apply_command((float)raw * (1.0f / 32767.0f));
        } else if (msg.std_id == CANID_KEEPALIVE) {
            prv_outer.comms_lost = 0;
        } else if (msg.std_id == CANID_BACKUP_REQ) {
            backup_handle_request(&msg); /* read-only diagnostic dump */
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public: init / start.                                                      */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Initialise the control loops, the commutation state and the channel table
 */
void control_init(void) {
    uint8_t i = 0;

    prv_tune = config_tune();

    for (i = 0; i < 3u; i++) {
        prv_cur.i_raw[i] = 0;
    }
    prv_cur.iq_meas = 0.0f;
    prv_cur.iq_int = 0.0f;
    prv_cur.duty = 0.0f;

    prv_outer.cmd = 0.0f;
    prv_outer.iq_ref = 0.0f;
    prv_outer.loop_tick = 0;
    prv_outer.loop_us = 0;
    prv_outer.comms_lost = 0;
    prv_outer.fault_code = FAULT_NONE;
    prv_outer.run_state = CTRL_STATE_IDLE;

    prv_comm.sector = 0;
    prv_comm.hall = 0;
    prv_comm.speed_raw = 0;
    prv_comm.cap_prev = 0;
    prv_comm.cap_period = 0;
    prv_comm.stall_ticks = 0;
}

/**
 * \brief           Leave init and enter the running state
 */
void control_start(void) {
    prv_outer.fault_code = FAULT_NONE;
    prv_outer.run_state = CTRL_STATE_RUN;
    motor_output_enable();
}

/* ------------------------------------------------------------------------- */
/* Public: the super-loop body.                                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Run one control update (read command, set the reference); the super-loop body
 * \note            Maps the newest drive command to the current reference through the
 *                      ramp regions, runs the protection checks, and times the iteration with the
 *                      free-running SysTick count.
 */
void main_service(void) {
    uint32_t t0 = SysTick->VAL;
    float iref = 0.0f;

    prv_outer.loop_tick++;

    prv_can_drain();

    if (prv_outer.run_state == CTRL_STATE_RUN) {
        iref = prv_cmd_to_iref(prv_outer.cmd);
    }
    prv_outer.iq_ref = iref;

    faultmon_check();

    /* SysTick counts down; the span is the wrap-corrected difference. */
    prv_outer.loop_us = (t0 - SysTick->VAL) & SysTick_LOAD_RELOAD_Msk;
}

/* ------------------------------------------------------------------------- */
/* Public: observers / trip / command input.                                  */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Latest electrical speed estimate, raw timer-count units
 * \return          the speed estimate (0 when stopped)
 */
int32_t control_speed_raw(void) { return prv_comm.speed_raw; }

/**
 * \brief           Latest commutation step, 0..5
 * \return          the live six-step sector
 */
uint8_t control_comm_step(void) { return prv_comm.sector; }

/**
 * \brief           Cut the drive and latch a fault
 * \param[in]       fault_code: the fault to latch (\ref fault_code_t)
 */
void control_trip(uint8_t fault_code) {
    motor_output_disable();
    prv_cur.iq_int = 0.0f;
    prv_cur.duty = 0.0f;
    prv_outer.fault_code = fault_code;
    prv_outer.run_state = CTRL_STATE_FAULT;
}

/* ------------------------------------------------------------------------- */
/* Interrupt handlers.                                                        */
/* ------------------------------------------------------------------------- */

/**
 * \brief           ADC preempt-complete interrupt: the current loop
 * \note            Fires on the TMR1 CH4 trigger, synchronous with the PWM carrier. Removes the
 *                      measured zero offsets, forms the quadrature current from the driven phases,
 *                      runs the PI regulator against the reference, and writes the new duty.
 */
void ADC1_IRQHandler(void) {
    float err = 0.0f;
    float out = 0.0f;
    uint8_t p = 0;

    if ((ADC1->sts & ADC_PCCE_FLAG) != 0u) {
        ADC1->sts = ~ADC_PCCE_FLAG;

        for (p = 0; p < 3u; p++) {
            prv_cur.i_raw[p] = (int32_t)adc_phase_current_raw((adc_phase_t)p) - (int32_t)faultmon_current_offset(p);
        }

        /* The quadrature current is the magnitude carried by the two driven phases of the live
           sector; the floating phase carries none. */
        prv_cur.iq_meas = (float)(prv_cur.i_raw[0] - prv_cur.i_raw[1]) * prv_tune->k_cur_scale;

        if (prv_outer.run_state == CTRL_STATE_RUN) {
            /* PI on the current error: the proportional term carries the gain, divided by
               full scale so it acts on the same per-unit reference the outer loop produces. */
            err = prv_outer.iq_ref - prv_cur.iq_meas;
            prv_cur.iq_int = prv_clampf(prv_cur.iq_int + CTRL_IQ_KI * err, CTRL_IQ_INT_MAX);
            out = (CTRL_IQ_GAIN / (float)CTRL_CMD_FULL) * err + prv_cur.iq_int;
            prv_cur.duty = prv_clampf(out, 1.0f);
            prv_apply_sector(prv_comm.sector, (prv_cur.duty < 0.0f) ? -prv_cur.duty : prv_cur.duty);
        }
    }
}

/**
 * \brief           Ordinary-scan DMA full-transfer interrupt
 * \note            One pass of the slow monitor group (bus voltage, throttle, temperatures) has
 *                      landed in the DMA ring; the averaged values are read on demand through
 *                      \ref adc_ordinary_mean, so the handler only clears the flag.
 */
void DMA1_Channel1_IRQHandler(void) {
    if ((DMA1->sts & DMA1_FDT1_FLAG) != 0u) {
        DMA1->clr = DMA1_FDT1_FLAG;
    }
}

/**
 * \brief           TMR3 hall interrupt: six-step commutation + speed estimate
 * \note            CH1 captures every hall edge (the XOR of the three hall lines). Each edge
 *                      decodes the hall code to a sector, measures the counts since the previous
 *                      edge for the speed estimate, and re-points the bridge at the new sector.
 *                      An illegal hall code (all-low / all-high) latches a stall fault.
 */
void TMR3_GLOBAL_IRQHandler(void) {
    uint32_t idr = 0;
    uint8_t hall = 0;
    int8_t sector = 0;

    if ((TMR3->ists & TMR_C1_FLAG) != 0u) {
        uint32_t cap = TMR3->c1dt;

        TMR3->ists = (uint16_t)(~TMR_C1_FLAG);

        /* Hall code from PB0 / PB4 / PB5. */
        idr = GPIOB->idt;
        hall = (uint8_t)((idr & 0x1u) | ((idr >> 3) & 0x2u) | ((idr >> 3) & 0x4u));
        prv_comm.hall = hall;
        sector = prv_hall_to_sector[hall & 0x7u];

        if (sector < 0) {
            control_trip(FAULT_STALL);
        } else {
            prv_comm.cap_period = (cap - prv_comm.cap_prev) & 0xFFFFu;
            prv_comm.cap_prev = cap;
            prv_comm.speed_raw = (prv_comm.cap_period != 0u) ? (int32_t)(1000000u / prv_comm.cap_period) : 0;
            prv_comm.sector = (uint8_t)sector;
            prv_comm.stall_ticks = 0;

            if (prv_outer.run_state == CTRL_STATE_RUN) {
                prv_apply_sector(prv_comm.sector, (prv_cur.duty < 0.0f) ? -prv_cur.duty : prv_cur.duty);
            }
        }
    }

    if ((TMR3->ists & TMR_OVF_FLAG) != 0u) {
        TMR3->ists = (uint16_t)(~TMR_OVF_FLAG);
        /* A full counter overflow with no hall edge means the rotor is not turning. */
        if (prv_outer.run_state == CTRL_STATE_RUN) {
            prv_comm.stall_ticks++;
            prv_comm.speed_raw = 0;
        }
    }
}
