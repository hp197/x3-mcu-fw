/**
 * \file            config.h
 * \brief           Image layout, the slot-0 header section, and the configuration / identity store
 * \note            The MCU slot-0 header is just the banner at 0x08001400 (src/app_header.c),
 *                      placed by x3-mcu-fw.ld's .app_header section. The controller also keeps a
 *                      512-byte configuration / identity record in flash (mirrored A/B pages, a
 *                      signature and a UID-derived serial); config_init loads it and seeds the
 *                      factory defaults.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Image layout                                                              */
/* ------------------------------------------------------------------------- */

#define APP_SLOT_BASE            0x08001000u /*!< application vector table / slot-0 base   */
#define APP_BANNER_ADDR          0x08001400u /*!< slot-0 banner (SCOOTER_MCU_0001), 16 B   */

/**
 * \brief           Place a symbol in a slot-0 header section (x3-mcu-fw.ld .app_header.*)
 * \note            The banner is the only header cell on the MCU; tools identify the image by it
 *                      (16 ASCII bytes at payload offset 0x400).
 */
#define APP_HEADER_SECTION(part) __attribute__((section(".app_header." part), used))

/* ------------------------------------------------------------------------- */
/* Configuration / identity store                                            */
/* ------------------------------------------------------------------------- */

#define IDENTITY_REC_SIZE        512u               /*!< flash record size, int16-sum signed      */
#define IDENTITY_MODEL_LEN       16u                /*!< ASCII model / serial template length     */
#define IDENTITY_DEFAULT_MODEL   "Z025000000000000" /*!< factory "not activated" model */
#define MCU_UID_ADDR             0x1FFFF7E8u        /*!< 96-bit factory unique id (AT32/STM32)    */

/**
 * \brief           Parsed calibration / tune cells (from the flash calibration block)
 * \note            The device-specific cells are read from the flash calibration block at boot
 *                      (big-endian 16-bit) and the fixed cells are the defaults. Scales
 *                      are the real units; the raw counts are left as stored. The control loop
 *                      selects its setpoint between the command deadband and ceiling (the two
 *                      cells it reads, plus the fixed default). Cells with no in-firmware consumer
 *                      keep an offset name -- their role is not established.
 */
typedef struct {
    uint16_t cal_marker;   /*!< +0x00 calibration-block marker byte                     */
    float scale_small;     /*!< +0x04 small scale (stored / 1e6)                        */
    float scale_mid_a;     /*!< +0x08 mid scale A (stored / 1000)                       */
    float k_cur_scale;     /*!< +0x0c current/voltage scale constant (0.273)            */
    float scale_mid_b;     /*!< +0x10 mid scale B (stored / 1000)                       */
    uint16_t val_14;       /*!< +0x14 stored cell                                       */
    uint16_t val_16;       /*!< +0x16 stored cell (two bytes)                           */
    uint16_t val_18;       /*!< +0x18 stored cell                                       */
    uint16_t def_1c;       /*!< +0x1c fixed default (50)                                */
    uint16_t val_1e;       /*!< +0x1e stored cell                                       */
    uint16_t val_20;       /*!< +0x20 stored cell                                       */
    uint16_t val_22;       /*!< +0x22 stored cell                                       */
    uint16_t val_24;       /*!< +0x24 stored cell                                       */
    uint16_t def_26;       /*!< +0x26 fixed default (40)                                */
    uint16_t def_28;       /*!< +0x28 fixed default (115)                               */
    uint16_t val_2a;       /*!< +0x2a stored cell                                       */
    uint16_t val_2c;       /*!< +0x2c stored cell                                       */
    uint16_t rsv_2e;       /*!< +0x2e reserved                                          */
    uint16_t val_30;       /*!< +0x30 stored cell (stored >> 1)                         */
    uint16_t val_32;       /*!< +0x32 stored cell (stored >> 1)                         */
    uint16_t val_34;       /*!< +0x34 stored cell (stored >> 1)                         */
    uint16_t rsv_36;       /*!< +0x36 reserved                                          */
    uint16_t def_38;       /*!< +0x38 fixed default (153)                               */
    uint16_t rsv_3a;       /*!< +0x3a reserved                                          */
    uint16_t cmd_thresh1;  /*!< +0x3c region threshold 1 (parsed; no consumer yet)      */
    uint16_t cmd_deadband; /*!< +0x3e deadband: reference is the default below it (control.c) */
    uint16_t cmd_thresh3;  /*!< +0x40 region threshold 3 (parsed; no consumer yet)      */
    uint16_t cmd_ceiling;  /*!< +0x42 drive ceiling: the reference ramps to full at it (control.c) */
    uint16_t cmd_default;  /*!< +0x44 fixed default (20): reference held below the deadband (control.c) */
    uint16_t def_46;       /*!< +0x46 fixed default (10)                                */
    uint16_t def_48;       /*!< +0x48 fixed default (10)                                */
    uint16_t val_4a;       /*!< +0x4a stored cell                                       */
    uint16_t val_4c;       /*!< +0x4c stored cell (one byte)                            */
    uint16_t rsv_4e;       /*!< +0x4e reserved                                          */
    float k_02;            /*!< +0x50 fixed constant (0.2)                              */
    uint16_t val_54;       /*!< +0x54 stored cell (0)                                   */
    uint16_t val_56;       /*!< +0x56 fixed constant (0x3fc0)                           */
    float k_13;            /*!< +0x58 fixed constant (1.3)                              */
    float k_14;            /*!< +0x5c fixed constant (14.0)                             */
    int32_t val_60;        /*!< +0x60 stored cell (stored << 13)                        */
} mcu_tune_t;

/** \brief Prepare the flash controller: performance register (wait states + prefetch). */
void flash_init(void);

/** \brief Load the identity / configuration record, verify it, and seed the defaults. */
void config_init(void);

/** \brief The 16-byte model / serial string (default IDENTITY_DEFAULT_MODEL when not activated). */
const char* config_model(void);

/**
 * \brief           Whether the unit is paired / activated (the UID consistency flag)
 * \return          1 when paired, 0 when running on the factory defaults
 */
uint8_t config_is_paired(void);

/** \brief The three 32-bit words of the factory UID (device serial source). */
const uint32_t* config_uid(void);

/** \brief The parsed calibration / tune cells (the control loop reads its thresholds here). */
const mcu_tune_t* config_tune(void);

#endif /* CONFIG_H */
