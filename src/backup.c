/**
 * \file            backup.c
 * \brief           Read-only backup of the per-unit persistent data (diagnostic)
 * \note            Streams the identity / OTA-descriptor / calibration flash pages and the factory
 *                      UID out over CAN so a service app (through the VCU) can keep a copy for a
 *                      blank-chip restore. Read-only by construction: it only ever reads flash and
 *                      sends it; it never writes, and there is no counterpart that accepts bytes
 *                      back. The source regions are a fixed allow-list -- the per-unit data pages
 *                      and the UID, never arbitrary flash or the program image.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "backup.h"

#include <stdint.h>

#include "can.h"

/* Flash layout of the per-unit data (the last four 1 KB pages of flash + the factory UID). The
   firmware image itself is not backed up here: it is the same for every unit and is reflashed
   from the published image, so only the per-unit data a blank chip would be missing is dumped. */
#define BACKUP_IDENTITY_A 0x0801F000u /*!< identity record, page A                 */
#define BACKUP_IDENTITY_B 0x0801F400u /*!< identity record, page B (mirror)        */
#define BACKUP_OTA_DESC   0x0801F800u /*!< OTA boot descriptor page                */
#define BACKUP_CAL_TUNE   0x0801FC00u /*!< calibration / tune page                 */
#define BACKUP_PAGE_SIZE  1024u       /*!< flash page / erase granularity          */
#define BACKUP_UID_ADDR   0x1FFFF7E8u /*!< 96-bit factory UID                      */
#define BACKUP_UID_SIZE   12u

/** \brief Request sub-command markers (first request byte). */
#define BACKUP_REQ_COUNT  0xFFu /*!< ask for the part count       */
#define BACKUP_REQ_META   0xFEu /*!< ask for one part's size/dest */

/**
 * \brief           One backup part: a contiguous source region and where a restore writes it
 * \note            \ref dest_addr is informational, for the restore tool; the UID part is factory
 *                      read-only, so its destination is 0 (not restorable).
 */
typedef struct {
    uint32_t src_addr;  /*!< memory-mapped source address to read from */
    uint32_t dest_addr; /*!< flash destination a restore would write (0 = not restorable) */
    uint16_t size;      /*!< region length in bytes                    */
} backup_part_t;

/** \brief The fixed set of parts a blank-chip restore needs (the read allow-list). */
static const backup_part_t k_parts[BACKUP_PART_COUNT] = {
    {BACKUP_IDENTITY_A, BACKUP_IDENTITY_A, BACKUP_PAGE_SIZE},
    {BACKUP_IDENTITY_B, BACKUP_IDENTITY_B, BACKUP_PAGE_SIZE},
    {BACKUP_OTA_DESC, BACKUP_OTA_DESC, BACKUP_PAGE_SIZE},
    {BACKUP_CAL_TUNE, BACKUP_CAL_TUNE, BACKUP_PAGE_SIZE},
    {BACKUP_UID_ADDR, 0u, BACKUP_UID_SIZE},
};

/**
 * \brief           Send the part-count reply
 */
static void prv_send_count(void) {
    uint8_t rsp[2];

    rsp[0] = BACKUP_REQ_COUNT;
    rsp[1] = (uint8_t)BACKUP_PART_COUNT;
    can_send(CANID_BACKUP_RSP, rsp, 2u);
}

/**
 * \brief           Send one part's size and restore destination
 * \param[in]       part: part index, 0 .. BACKUP_PART_COUNT-1
 */
static void prv_send_meta(uint8_t part) {
    const backup_part_t* p = &k_parts[part];
    uint8_t rsp[8];

    rsp[0] = BACKUP_REQ_META;
    rsp[1] = part;
    rsp[2] = (uint8_t)(p->size & 0xFFu);
    rsp[3] = (uint8_t)(p->size >> 8);
    rsp[4] = (uint8_t)(p->dest_addr & 0xFFu);
    rsp[5] = (uint8_t)((p->dest_addr >> 8) & 0xFFu);
    rsp[6] = (uint8_t)((p->dest_addr >> 16) & 0xFFu);
    rsp[7] = (uint8_t)((p->dest_addr >> 24) & 0xFFu);
    can_send(CANID_BACKUP_RSP, rsp, 8u);
}

/**
 * \brief           Send one chunk of a part's bytes at the requested offset
 * \param[in]       part: part index
 * \param[in]       offset: byte offset into the part
 */
static void prv_send_chunk(uint8_t part, uint16_t offset) {
    const backup_part_t* p = &k_parts[part];
    const volatile uint8_t* src = (const volatile uint8_t*)p->src_addr;
    uint8_t rsp[3u + BACKUP_CHUNK_LEN];
    uint16_t remain;
    uint8_t n;
    uint8_t i;

    rsp[0] = part;
    rsp[1] = (uint8_t)(offset & 0xFFu);
    rsp[2] = (uint8_t)(offset >> 8);

    remain = (offset < p->size) ? (uint16_t)(p->size - offset) : 0u;
    n = (remain < BACKUP_CHUNK_LEN) ? (uint8_t)remain : (uint8_t)BACKUP_CHUNK_LEN;
    for (i = 0; i < n; i++) {
        rsp[3u + i] = src[offset + i];
    }
    can_send(CANID_BACKUP_RSP, rsp, (uint8_t)(3u + n));
}

void backup_handle_request(const mcu_can_msg_t* msg) {
    uint8_t sel;
    uint16_t offset;

    if ((msg == 0) || (msg->dlc < 1u)) {
        return;
    }

    sel = msg->data[0];
    if (sel == BACKUP_REQ_COUNT) {
        prv_send_count();
    } else if (sel == BACKUP_REQ_META) {
        if ((msg->dlc >= 2u) && (msg->data[1] < BACKUP_PART_COUNT)) {
            prv_send_meta(msg->data[1]);
        }
    } else if ((sel < BACKUP_PART_COUNT) && (msg->dlc >= 3u)) {
        offset = (uint16_t)((uint16_t)msg->data[1] | ((uint16_t)msg->data[2] << 8));
        prv_send_chunk(sel, offset);
    }
}
