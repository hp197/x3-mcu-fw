/**
 * \file            motor.c
 * \brief           TMR1 complementary PWM: the 3-phase bridge
 * \note            TMR1 drives the three half-bridges as a center-aligned complementary-PWM group.
 *                      CH1/CH2/CH3 (and their N outputs) are the phases; CH4 is an internal compare
 *                      used only to trigger the ADC phase-current sampling in the middle of the PWM
 *                      period. The commutation / current loop (control module) drives the per-phase
 *                      compare and enables the outputs; this module owns the TMR1 hardware setup and
 *                      the output primitives.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "motor.h"

#include "at32f415.h"

#define MOTOR_PWM_DUTY_INIT    (((MOTOR_PWM_PERIOD + 1u) / 2u) - 1u) /*!< 2249: 50 % at reset */
#define MOTOR_ADC_TRIGGER_CMP  10u   /*!< CH4 compare: fire the ADC near the carrier peak */
#define MOTOR_PWM_DEADTIME     0x99u /*!< dead-time generator setup (DTC) */
#define MOTOR_PHASE_CHANNEL(p) ((tmr_channel_select_type)((uint32_t)(p) * 2u)) /*!< U/V/W -> CH1/2/3 */

/**
 * \brief           Configure the six PWM output pins as TMR1 alternate-function push-pull
 * \note            PA8/PA9/PA10 = CH1/CH2/CH3, PB13/PB14/PB15 = CH1N/CH2N/CH3N. CH4 is internal
 *                      (the ADC trigger), so PA11 is left alone.
 */
static void prv_pwm_pins_init(void) {
    gpio_init_type gpio_init_struct;

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOB, &gpio_init_struct);
}

/**
 * \brief           Configure one phase channel (CH1/CH2/CH3) for complementary PWM
 * \param[in]       channel: the main channel of the pair (TMR_SELECT_CHANNEL_1/2/3)
 */
static void prv_phase_channel_init(tmr_channel_select_type channel) {
    tmr_output_config_type oc;

    tmr_output_default_para_init(&oc);
    oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    oc.oc_output_state = TRUE;
    oc.occ_output_state = TRUE;
    oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    oc.oc_idle_state = FALSE;
    oc.occ_idle_state = FALSE;

    tmr_output_channel_config(TMR1, channel, &oc);
    tmr_channel_value_set(TMR1, channel, MOTOR_PWM_DUTY_INIT);
}

/**
 * \brief           Configure CH4 as the ADC-trigger compare (not a physical output)
 */
static void prv_adc_trigger_init(void) {
    tmr_output_config_type oc;

    tmr_output_default_para_init(&oc);
    oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    oc.oc_output_state = TRUE;
    oc.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
    oc.oc_idle_state = TRUE;

    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, MOTOR_ADC_TRIGGER_CMP);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
}

/**
 * \brief           Bring up the TMR1 3-phase complementary-PWM bridge
 */
void motor_init(void) {
    tmr_brkdt_config_type brkdt;

    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    prv_pwm_pins_init();

    /* Center-aligned carrier: 144 MHz / (2 * (PERIOD + 1)) = 16 kHz, period buffered (ARPE). */
    tmr_base_init(TMR1, MOTOR_PWM_PERIOD, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_1);
    tmr_period_buffer_enable(TMR1, TRUE);

    prv_phase_channel_init(TMR_SELECT_CHANNEL_1);
    prv_phase_channel_init(TMR_SELECT_CHANNEL_2);
    prv_phase_channel_init(TMR_SELECT_CHANNEL_3);
    prv_adc_trigger_init();

    tmr_brkdt_default_para_init(&brkdt);
    brkdt.deadtime = MOTOR_PWM_DEADTIME;
    tmr_brkdt_config(TMR1, &brkdt);

    tmr_output_enable(TMR1, TRUE); /* main output enable (MOE) */
    tmr_counter_enable(TMR1, TRUE);

    motor_output_disable(); /* bridge stays off until the control loop drives it */
}

/**
 * \brief           Set one phase's compare value
 * \param[in]       phase: motor phase U/V/W
 * \param[in]       compare: 0..MOTOR_PWM_PERIOD
 */
void motor_set_duty(motor_phase_t phase, uint16_t compare) {
    tmr_channel_value_set(TMR1, MOTOR_PHASE_CHANNEL(phase), compare);
}

/**
 * \brief           Enable the six bridge outputs (the three complementary pairs)
 */
void motor_output_enable(void) {
    uint32_t channel;

    for (channel = TMR_SELECT_CHANNEL_1; channel <= TMR_SELECT_CHANNEL_3C; channel++) {
        tmr_channel_enable(TMR1, (tmr_channel_select_type)channel, TRUE);
    }
}

/**
 * \brief           Disable the six bridge outputs (freewheel)
 */
void motor_output_disable(void) {
    uint32_t channel;

    for (channel = TMR_SELECT_CHANNEL_1; channel <= TMR_SELECT_CHANNEL_3C; channel++) {
        tmr_channel_enable(TMR1, (tmr_channel_select_type)channel, FALSE);
    }
}
