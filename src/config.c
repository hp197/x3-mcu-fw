/**
 * \file            config.c
 * \brief           Configuration / identity store (FLASH + UID serial)
 * \note            The controller keeps a 512-byte identity record in flash, mirrored across two
 *                      pages (A then B), plus a calibration / tune block. At boot config_init
 *                      seeds the factory defaults (the "Z025..." model template and a UID-derived
 *                      serial), then loads the stored record and keeps it when a page is present.
 *                      flash_init prepares the flash controller.
 *
 *                      The field-level meaning of the per-tune cells, the record signature check,
 *                      and the XTEA block cipher the firmware carries for the protected config are
 *                      only partly mapped; those paths are left for a later pass and the defaults
 *                      stand until a record is validated.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "config.h"

#include <string.h>

#include "at32f415.h"

/** \brief Identity record pages: A is tried first, B is the mirror fallback. */
#define IDENTITY_PAGE_A     0x0801F000u
#define IDENTITY_PAGE_B     0x0801F400u

/** \brief Calibration / tune block (256 bytes), parsed into the scaled runtime cells. */
#define CONFIG_CAL_BLOCK    0x0801FC00u

/** \brief Flash performance register: 4 wait states + prefetch, the value for 144 MHz HCLK. */
#define FLASH_PSR_VALUE     0x00000154u

/** \brief Record byte 0 is the activation marker: a blank page reads 0x00 or 0xFF. */
#define RECORD_BLANK_LO     0x00u
#define RECORD_BLANK_HI     0xFFu

/** \brief Offset of the model / serial string inside the record. */
#define RECORD_MODEL_OFFSET 0x40u

/** \brief Parsed identity / configuration the other subsystems read. */
static struct {
    char model[IDENTITY_MODEL_LEN]; /*!< 16-char model / serial (default or stored)  */
    uint32_t uid[3];                /*!< factory UID words (device-serial source)     */
    uint32_t uid_checksum;          /*!< one's-complement sum of the three UID words  */
    uint8_t paired;                 /*!< 1 when a stored record is present, else 0     */
} g_cfg;

/** \brief Parsed calibration / tune cells (the control loop reads its thresholds here). */
static mcu_tune_t g_tune;

/**
 * \brief           Read a big-endian 16-bit value from the calibration block
 * \param[in]       p: pointer to the two stored bytes (most significant first)
 * \return          the assembled 16-bit value
 */
static uint16_t prv_rd16(const uint8_t* p) { return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]); }

/**
 * \brief           Seed the fixed tune cells and zero the device-sourced cells
 */
static void prv_tune_defaults(void) {
    memset(&g_tune, 0, sizeof(g_tune));
    g_tune.k_cur_scale = 0.273f;
    g_tune.def_1c = 50u;
    g_tune.def_26 = 40u;
    g_tune.def_28 = 115u;
    g_tune.def_38 = 153u;
    g_tune.def_44 = 20u;
    g_tune.def_46 = 10u;
    g_tune.def_48 = 10u;
    g_tune.k_02 = 0.2f;
    g_tune.val_56 = 0x3fc0u;
    g_tune.k_13 = 1.3f;
    g_tune.k_14 = 14.0f;
}

/**
 * \brief           Parse the flash calibration block into the device-sourced tune cells
 * \param[in]       p: the calibration block (CONFIG_CAL_BLOCK)
 * \note            Stored cells are big-endian 16-bit; the scales divide by the
 *                      constants. Only called when the block is present (byte 0 not blank).
 */
static void prv_tune_parse(const uint8_t* p) {
    g_tune.cal_marker = p[0x50];
    g_tune.scale_small = (float)prv_rd16(&p[0x51]) / 1.0e6f;
    g_tune.scale_mid_a = (float)prv_rd16(&p[0x53]) / 1000.0f;
    g_tune.scale_mid_b = (float)prv_rd16(&p[0x55]) / 1000.0f;
    g_tune.val_14 = prv_rd16(&p[0x83]);
    g_tune.val_16 = (uint16_t)((uint16_t)p[0x61] | ((uint16_t)p[0x62] << 8));
    g_tune.val_18 = p[0x0e];
    g_tune.val_1e = prv_rd16(&p[0x04]);
    g_tune.val_20 = prv_rd16(&p[0x06]);
    g_tune.val_22 = p[0x08];
    g_tune.val_24 = p[0x09];
    g_tune.val_2a = p[0x0c];
    g_tune.val_2c = p[0x0d];
    g_tune.val_30 = (uint16_t)(prv_rd16(&p[0x10]) >> 1);
    g_tune.val_32 = (uint16_t)(prv_rd16(&p[0x12]) >> 1);
    g_tune.val_34 = (uint16_t)(prv_rd16(&p[0x14]) >> 1);
    g_tune.cmd_thresh1 = prv_rd16(&p[0x3a]);
    g_tune.cmd_thresh2 = p[0x3c];
    g_tune.cmd_thresh3 = prv_rd16(&p[0x3d]);
    g_tune.cmd_thresh4 = p[0x3f];
    g_tune.val_4a = p[0x67];
    g_tune.val_4c = p[0x0f];
    g_tune.val_60 = (int32_t)((uint32_t)prv_rd16(&p[0x53]) << 13);
}

/* ------------------------------------------------------------------------- */

/**
 * \brief           Prepare the flash controller
 * \note            Sets the flash performance register (wait states + prefetch). The clock
 *                      bring-up programs the same register for the target HCLK; this sets the
 *                      running value explicitly, matching the boot sequence.
 */
void flash_init(void) { FLASH->psr = FLASH_PSR_VALUE; }

/**
 * \brief           Read the factory UID and form the device checksum
 * \note            Copies the three 96-bit UID words and forms the one's-complement sum the stored
 *                      record is checked against. The UID is the device-serial source.
 */
static void prv_uid_read(void) {
    const volatile uint32_t* uid = (const volatile uint32_t*)MCU_UID_ADDR;
    uint32_t sum = 0u;
    uint32_t i = 0u;

    while (i < 3u) {
        g_cfg.uid[i] = uid[i];
        sum += g_cfg.uid[i];
        i++;
    }
    g_cfg.uid_checksum = ~sum;
}

/**
 * \brief           Seed the factory "not activated" defaults
 * \note            The model / serial becomes the IDENTITY_DEFAULT_MODEL template and the unit is
 *                      marked not paired until a stored record is loaded.
 */
static void prv_set_defaults(void) {
    memcpy(g_cfg.model, IDENTITY_DEFAULT_MODEL, IDENTITY_MODEL_LEN);
    g_cfg.paired = 0u;
}

/**
 * \brief           Select the valid identity record page
 * \return          Pointer to page A when present, else page B when present, else NULL
 * \note            A page is "present" when its activation byte is neither blank value. Page A is
 *                      the live record and page B the mirror fallback.
 */
static const uint8_t* prv_select_record(void) {
    const uint8_t* rec = (const uint8_t*)IDENTITY_PAGE_A;

    if ((rec[0] == RECORD_BLANK_LO) || (rec[0] == RECORD_BLANK_HI)) {
        rec = (const uint8_t*)IDENTITY_PAGE_B;
        if ((rec[0] == RECORD_BLANK_LO) || (rec[0] == RECORD_BLANK_HI)) {
            rec = NULL;
        }
    }
    return rec;
}

/**
 * \brief           Load the identity / configuration record, seed the defaults
 * \note            Seeds the factory defaults, reads the UID serial, then selects the stored
 *                      record page and copies the model from it; otherwise the defaults stand and
 *                      the unit reports not paired. Validating the record signature and parsing the
 *                      calibration block (CONFIG_CAL_BLOCK) are a later pass.
 */
void config_init(void) {
    const uint8_t* rec;

    prv_set_defaults();
    prv_uid_read();

    rec = prv_select_record();
    if (rec != NULL) {
        memcpy(g_cfg.model, &rec[RECORD_MODEL_OFFSET], IDENTITY_MODEL_LEN);
        g_cfg.paired = 1u;
    }

    /* Calibration / tune: the fixed cells always stand; the device-sourced cells come from the
       flash calibration block when it is present (its marker byte is not blank). */
    prv_tune_defaults();
    {
        const uint8_t* cal = (const uint8_t*)CONFIG_CAL_BLOCK;
        if ((cal[0] != RECORD_BLANK_LO) && (cal[0] != RECORD_BLANK_HI)) {
            prv_tune_parse(cal);
        }
    }
}

/* ------------------------------------------------------------------------- */

const char* config_model(void) { return g_cfg.model; }

uint8_t config_is_paired(void) { return g_cfg.paired; }

const uint32_t* config_uid(void) { return g_cfg.uid; }

const mcu_tune_t* config_tune(void) { return &g_tune; }
