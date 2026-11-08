/**
 * \file            timers.h
 * \brief           Hall-sensor interface timer (TMR3)
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TIMERS_H
#define TIMERS_H

/**
 * \brief           Bring up the hall-sensor interface timer (TMR3)
 * \note            TMR3 runs in XOR hall mode: the three hall inputs are combined on channel 1,
 *                      so every hall transition captures the counter and raises the channel-1 /
 *                      overflow interrupt that drives commutation and the speed estimate.
 */
void timers_init(void);

#endif /* TIMERS_H */
