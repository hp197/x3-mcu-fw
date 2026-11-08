/**
 * \file            main.c
 * \brief           x3 MCU (motor-controller) entry point
 * \note            Points VTOR at the application vector table, brings up the clock and the
 *                      peripherals in order, then runs the control super-loop. The
 *                      time-critical work (commutation, the current loop) runs from the TMR1 and
 *                      ADC interrupts; the loop services the control update and the CAN link.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "at32f415.h"

#include "adc.h"
#include "board.h" /* APP_VECTOR_TABLE */
#include "can.h"
#include "clock.h"
#include "config.h"
#include "control.h"
#include "faultmon.h"
#include "gpio.h"
#include "motor.h"
#include "timers.h"

/**
 * \brief           Program entry
 * \return          never returns
 */
int main(void) {
    /* The BSP SystemInit points VTOR at the bootloader's table (0x08000000); select the app
       table here (0x08001000). */
    SCB->VTOR = APP_VECTOR_TABLE;

    clock_config(); /* reset the clock tree to HICK, then HEXT -> PLL -> 144 MHz SYSCLK */
    systick_init(); /* free-running SysTick time base */
    flash_init();   /* flash controller: wait states, unlock for the config writes */

    __disable_irq();

    config_init();   /* load the identity / configuration record, seed the defaults */
    faultmon_init(); /* reset the protection latches before the drive comes up */

    /* Every peripheral, once, in order: GPIO groups, ADC + DMA acquisition, the
       TMR1 motor bridge, the TMR3/TMR4 time bases, CAN1. */
    gpio_setup();  /* gate-driver enables, hall / fault input pins, external interrupts */
    adc_init();    /* phase-current / bus-voltage / throttle sensing (ADC1 + DMA) */
    motor_init();  /* TMR1 complementary PWM, the 3-phase bridge */
    timers_init(); /* TMR3 / TMR4 time base, hall capture */
    can_init();    /* the command / telemetry link to the VCU */

    /* The peripherals armed their interrupt sources; enable them in the NVIC. The current loop
       (ADC, fired by the TMR1 trigger) preempts the commutation (TMR3 hall) and the DMA tick. */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_2);
    board_irq_enable(ADC1_IRQn, BOARD_IRQ_PRIO(0, 0));
    board_irq_enable(TMR3_GLOBAL_IRQn, BOARD_IRQ_PRIO(1, 0));
    board_irq_enable(DMA1_Channel1_IRQn, BOARD_IRQ_PRIO(2, 0));

    control_init(); /* commutation / current / speed loop state, protection latches */

    __enable_irq();

    /* Phase-current zero-offset calibration runs with the interrupts live but the bridge still
       off; a bad offset latches the fault instead of starting the drive. */
    if (faultmon_calibrate() == FAULT_NONE) {
        control_start(); /* leave init, enter the running state */
    } else {
        control_trip(FAULT_CURRENT_OFFSET);
    }

    for (;;) {
        main_service(); /* control update: read sensors, step the loops */
        can_service();  /* CAN TX pump + RX dispatch */
    }
}
