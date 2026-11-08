/**
 * \file            model.h
 * \brief           MCU model identity (banner)
 * \note            The x3 MCU ships a single model. The slot-0 banner is the 16-byte ASCII token
 *                      tools identify the image by (payload offset 0x400), the counterpart of the
 *                      VCU's SCOOTER_VCU_<code>.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef MODEL_H
#define MODEL_H

#define MCU_MODEL_NAME   "x3-mcu"
#define MCU_MODEL_BANNER "SCOOTER_MCU_0001" /*!< slot-0 banner: SCOOTER_MCU_ + the 4-char code */

#endif /* MODEL_H */
