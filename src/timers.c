/**
 * \file            timers.c
 * \brief           Hall-sensor interface timer (TMR3)
 * \note            TMR3 is wired as the hall interface for the brushless motor. The three hall
 *                      sensors come in on PB0, PB4 and PB5; channel 1 is set to the XOR of the
 *                      three inputs, so a capture happens on every hall edge regardless of which
 *                      sensor moved. The counter runs free (period 0xFFFF, HCLK/4 via the clock
 *                      divider) and the channel-1 / overflow interrupt feeds commutation and the
 *                      speed estimate. The capture interrupt body lives in the control module.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "timers.h"

#include "at32f415.h"

/** \brief The three hall-sensor inputs: PB0, PB4, PB5 (combined as channel 1's XOR). */
#define HALL_INPUT_PINS (GPIO_PINS_0 | GPIO_PINS_4 | GPIO_PINS_5)

/**
 * \brief           Configure the three hall-sensor pins (PB0/PB4/PB5) as digital inputs
 */
static void prv_hall_inputs_init(void) {
    gpio_init_type gpio_init_struct;

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = HALL_INPUT_PINS;
    gpio_init(GPIOB, &gpio_init_struct);
}

/**
 * \brief           Configure TMR3 as the XOR hall interface with capture interrupts
 */
static void prv_tmr3_hall_init(void) {
    tmr_input_config_type input_struct;

    /* Free-running time base for the capture: full 16-bit span, HCLK/4 counting clock. */
    tmr_base_init(TMR3, 0xFFFFu, 0u);
    tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV4);

    /* Channel 1 input capture on the rising edge, no prescale, no filter. */
    tmr_input_default_para_init(&input_struct);
    input_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
    input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
    input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_input_channel_init(TMR3, &input_struct, TMR_CHANNEL_INPUT_DIV_1);

    /* Channel 1 sees the XOR of the three hall inputs; the trigger follows that combined edge. */
    tmr_channel1_input_select(TMR3, TMR_CHANEL1_2_3_CONNECTED_C1IRAW_XOR);
    tmr_trigger_input_select(TMR3, TMR_SUB_INPUT_SEL_C1INC);
    tmr_overflow_request_source_set(TMR3, TRUE);

    tmr_interrupt_enable(TMR3, TMR_OVF_INT | TMR_C1_INT, TRUE);
    tmr_flag_clear(TMR3, TMR_OVF_FLAG | TMR_C1_FLAG);
    tmr_counter_enable(TMR3, TRUE);
}

/**
 * \brief           Bring up the hall-sensor interface timer (TMR3)
 */
void timers_init(void) {
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);

    prv_hall_inputs_init();
    prv_tmr3_hall_init();
}
