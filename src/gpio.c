/**
 * \file            gpio.c
 * \brief           GPIO: gate-driver enables, control outputs and feedback inputs
 * \note            Configures the discrete I/O the motor controller drives directly, on GPIOB and
 *                      GPIOC. The TMR1 complementary-PWM pins and the ADC analog pins are owned by
 *                      their own modules (the motor bridge and the sampler); this module sets up
 *                      the plain push-pull outputs (gate-driver enables / control / indicator
 *                      lines) and the floating feedback inputs, then drives the outputs to their
 *                      reset levels.
 *
 *                      The AT32F415 GPIO config is the classic 4-bit-per-pin CFGLR (pins 0..7) /
 *                      CFGHR (pins 8..15) layout; \ref prv_gpio_config writes those fields. The
 *                      per-pin roles in the comments are the likely board functions.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "gpio.h"

#include "at32f415.h"

/* 4-bit CFGLR/CFGHR field values (MODE[1:0] | CNF[1:0]). */
#define MCU_GPIO_CFG_IN_FLOAT 0x4u /*!< input, no pull (CNF = floating)              */
#define MCU_GPIO_CFG_OUT_PP   0x1u /*!< output, push-pull, 10 MHz                    */

/* Push-pull outputs: gate-driver enables / control / indicator lines. */
#define MCU_OUT_C_PINS                                                                                                 \
    (GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12 | GPIO_PINS_14 | GPIO_PINS_15)
#define MCU_OUT_B_HIGH_PINS GPIO_PINS_7                 /*!< driven high at reset      */
#define MCU_OUT_B_LOW_PINS  (GPIO_PINS_2 | GPIO_PINS_6) /*!< driven low at reset       */

/* Floating feedback inputs (likely hall-sensor / fault / enable sense). */
#define MCU_IN_C_PINS       (GPIO_PINS_3 | GPIO_PINS_9)

/**
 * \brief           Write the 4-bit CFGLR/CFGHR field of the selected pins
 * \param[in,out]   port: GPIO port
 * \param[in]       pins: pin mask (GPIO_PINS_*)
 * \param[in]       cfg: 4-bit field value (\ref MCU_GPIO_CFG_OUT_PP, \ref MCU_GPIO_CFG_IN_FLOAT)
 */
static void prv_gpio_config(gpio_type* port, uint32_t pins, uint32_t cfg) {
    volatile uint32_t* cfgr = &port->cfglr; /* cfglr (pins 0..7), then cfghr (pins 8..15) */
    uint32_t pin;
    uint32_t shift;

    for (pin = 0u; pin < 16u; pin++) {
        if (((pins >> pin) & 1u) != 0u) {
            shift = (pin & 7u) * 4u;
            cfgr[pin >> 3] = (cfgr[pin >> 3] & ~(0xFu << shift)) | (cfg << shift);
        }
    }
}

/**
 * \brief           Configure the discrete GPIO and drive the outputs to their reset levels
 */
void gpio_setup(void) {
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    prv_gpio_config(GPIOC, MCU_OUT_C_PINS, MCU_GPIO_CFG_OUT_PP);
    prv_gpio_config(GPIOB, MCU_OUT_B_HIGH_PINS | MCU_OUT_B_LOW_PINS, MCU_GPIO_CFG_OUT_PP);
    prv_gpio_config(GPIOC, MCU_IN_C_PINS, MCU_GPIO_CFG_IN_FLOAT);

    /* Reset levels: one GPIOB line high, every other output low. */
    GPIOB->scr = MCU_OUT_B_HIGH_PINS;
    GPIOB->clr = MCU_OUT_B_LOW_PINS;
    GPIOC->clr = MCU_OUT_C_PINS;
}
