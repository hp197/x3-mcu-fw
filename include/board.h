/**
 * \file            board.h
 * \brief           Shared board configuration for the x3 MCU application
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef BOARD_H
#define BOARD_H

#include "at32f415.h"
#include "model.h"

/* System clock: the 16 MHz HEXT crystal via PLL gives HEXT * 9 = 144 MHz. SystemInit resets the
   tree to the internal HICK; main() then brings up HEXT + PLL in clock_config(). */
#define SYSCLK_HZ                144000000u
#define PCLK1_HZ                 (SYSCLK_HZ / 2u) /*!< APB1 = HCLK / 2 = 72 MHz */
#define PCLK2_HZ                 (SYSCLK_HZ / 2u) /*!< APB2 = HCLK / 2 = 72 MHz */

/* Application vector table. The bootloader owns 0x08000000; the app image (and its vector table)
   starts at 0x08001000 (see x3-mcu-fw.ld). */
#define APP_VECTOR_TABLE         0x08001000u

/** \brief Enable one peripheral clock: the write of crm_periph_clock_enable(periph, TRUE) */
#define BOARD_CLOCK_ON(periph)   (CRM_REG(periph) |= CRM_REG_BIT(periph))

/**
 * \brief           NVIC priority byte of a (preemption, sub) priority pair
 * \note            Encodes a pair under NVIC_PRIORITY_GROUP_2 (2 preemption + 2 sub bits).
 */
#define BOARD_IRQ_PRIO(pre, sub) ((uint8_t)((((uint32_t)(pre) << 2) | (uint32_t)(sub)) << (8u - __NVIC_PRIO_BITS)))

/**
 * \brief           Set the priority of an IRQ and enable it (the writes of nvic_irq_enable)
 * \param[in]       irqn: device IRQ number (>= 0)
 * \param[in]       prio: priority byte, \ref BOARD_IRQ_PRIO
 */
static inline void board_irq_enable(IRQn_Type irqn, uint8_t prio) {
    NVIC->IP[(uint32_t)irqn] = prio;
    NVIC->ISER[(uint32_t)irqn >> 5] = 1u << ((uint32_t)irqn & 0x1Fu);
}

#endif /* BOARD_H */
