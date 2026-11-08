/**
 * \file            backup.h
 * \brief           Read-only backup of the per-unit persistent data (diagnostic)
 * \note            A maintenance channel that lets the VCU (and, through it, a service app) read
 *                      out everything a blank chip needs restored: the identity record pages, the
 *                      OTA descriptor page, the calibration / tune page and the factory UID. It is
 *                      strictly read-only -- there is deliberately no restore / write path here;
 *                      the bytes are only dumped, never accepted back.
 *
 *                      Protocol (\ref CANID_BACKUP_REQ -> \ref CANID_BACKUP_RSP, both standard id):
 *                      - request `[0xFF]`                 -> response `[0xFF, num_parts]`
 *                      - request `[0xFE, part]`           -> response `[0xFE, part, size_lo, size_hi,
 *                                                            dest0, dest1, dest2, dest3]`
 *                      - request `[part, off_lo, off_hi]` -> response `[part, off_lo, off_hi,
 *                                                            data0 .. data4]` (up to 5 bytes)
 *                      The reader asks for the part count, then each part's size and restore
 *                      destination address, then streams the bytes offset by offset.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef BACKUP_H
#define BACKUP_H

#include "can.h"

/** \brief Number of backup parts a blank-chip restore needs. */
#define BACKUP_PART_COUNT 5u

/** \brief Payload data bytes carried per read response (after the 3-byte part/offset header). */
#define BACKUP_CHUNK_LEN  5u

/**
 * \brief           Handle one inbound backup request frame and send its response
 * \param[in]       msg: the received \ref CANID_BACKUP_REQ frame
 */
void backup_handle_request(const mcu_can_msg_t* msg);

#endif /* BACKUP_H */
