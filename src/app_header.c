/**
 * \file            app_header.c
 * \brief           The slot-0 image banner (0x08001400)
 * \note            The image carries the 16-byte ASCII token `SCOOTER_MCU_0001` at 0x08001400
 *                      (no terminator); no firmware code reads it. Tools identify the x3 MCU image
 *                      by it (the counterpart of the VCU's `SCOOTER_VCU_<code>`). x3-mcu-fw.ld
 *                      links it at 0x08001400; application code follows immediately after.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "config.h"
#include "model.h"

#define APP_BANNER_LEN 16u /*!< banner length: SCOOTER_ (8) + MCU (3) + _ (1) + 4-char code */

/** \brief Image banner of the model, e.g. `SCOOTER_MCU_0001` (0x08001400, no terminator) */
const char k_app_banner[APP_BANNER_LEN] APP_HEADER_SECTION("banner") = MCU_MODEL_BANNER;
