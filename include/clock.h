/**
 * \file            clock.h
 * \brief           System clock bring-up
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef CLOCK_H
#define CLOCK_H

/** \brief Bring up HEXT -> PLL x9 -> 144 MHz SYSCLK. */
void clock_config(void);

/** \brief Re-lock HEXT + PLL and switch SYSCLK back after deep sleep. */
void clock_resume(void);

/** \brief Start the free-running SysTick time base. */
void systick_init(void);

#endif /* CLOCK_H */
